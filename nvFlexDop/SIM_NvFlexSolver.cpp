

#include "SIM_NvFlexSolver.h"

#include "SIM_NvFlexData.h" //for static library

#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_ForceGravity.h>
#include <GU/GU_Detail.h>
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Default.h>
#include <PRM/PRM_Range.h>

#include <GA/GA_PageIterator.h>
#include <GA/GA_PageHandle.h>
#include <GA/GA_SplittableRange.h>

#include <algorithm>

#include "NvFlexHTriangleMesh.h"

#include <NvFlexDevice.h>

static void nvFlexErrorCallbackPrint(NvFlexErrorSeverity type, const char *msg, const char *file, int line) {
	switch (type) {
	case eNvFlexLogError:
		std::cout << "NvF ERROR: "; break;
	case eNvFlexLogWarning:
		std::cout << "NvF WARNING: "; break;
	case eNvFlexLogDebug:
		std::cout << "NvF DEBUG: "; break;
	case eNvFlexLogAll:
		std::cout << "NvF ALL: "; break;
	}
	if (msg != NULL)std::cout << msg;
	std::cout << " :: ";
	if (file != NULL)std::cout << file;
	std::cout << " :: ";
	std::cout << line;
	std::cout << std::endl;

}


SIM_NvFlexSolver::SIM_Result SIM_NvFlexSolver::solveObjectsSubclass(SIM_Engine & engine, SIM_ObjectArray & objs, SIM_ObjectArray & newobjs, SIM_ObjectArray & feedbackobjs, const SIM_Time & timestep)
{

	for (exint obji = 0; obji < objs.entries(); ++obji) {
		SIM_Object* obj = objs(obji);

		SIM_NvFlexData* nvdata = SIM_DATA_GET(*obj, "NvFlexData", SIM_NvFlexData);
		if (nvdata == NULL) {
			addError(obj, SIM_BADSUBDATA, "NvFlexData is not found on the object", UT_ERROR_WARNING);
			continue;
		}
		if (!nvdata->isNvValid()) {
			addError(obj, SIM_BADSUBDATA, "NvFlexData is in invalid state (maybe insufficient GPU resources). try resetting the simulation.", UT_ERROR_WARNING);
			continue;
		}

		NvFlexHContextAutoGetter contextAutoGetAndRelease(nvdata->nvFlexLibrary);

		std::shared_ptr<SIM_NvFlexData::NvFlexContainerWrapper> consolv = nvdata->nvdata;

		

		// Getting old geometry and shoving it into NvFlex buffers
		const SIM_Geometry *geo=SIM_DATA_GETCONST(*obj, "Geometry", SIM_Geometry);
		if (geo != NULL) {
			GU_DetailHandleAutoReadLock lock(geo->getGeometry());
			if (lock.isValid()) {
				int nactives = -1;
				const GU_Detail *gdp = lock.getGdp();
				int64 ndid = gdp->getP()->getDataId();
				int64 ntopdid = gdp->getTopology().getDataId();
				std::cout << "id = " << ndid << std::endl;
				if (ndid != nvdata->_lastGdpPId || ntopdid != nvdata->_lastGdpTId) {
					std::cout << "found geo, new id !! old P id: " << nvdata->_lastGdpPId << ". old topo id:" << nvdata->_lastGdpTId << std::endl;

					//we just search for attribs, not creating them cuz for now we work with RO geometry

					GA_ROHandleV3 phnd(gdp->getP());
					GA_ROHandleV3 vhnd(gdp->findPointAttribute("v"));
					GA_ROHandleI ihnd(gdp->findPointAttribute("iid"));
					GA_ROHandleI phshnd(gdp->findPointAttribute("phs"));
					GA_ROHandleF mhnd(gdp->findPointAttribute("imass"));
					GA_ROHandleV3 rhnd(gdp->findPointAttribute("restP"));
					const bool hasRest = rhnd.isValid();

					int* indices = nvdata->_indices.get();
					if (nactives == -1)nactives = NvFlexExtGetActiveList(consolv->container(), indices); //do not reread indices if they have already been read before in this geo lock block

					if (phnd.isValid() && vhnd.isValid() && ihnd.isValid() && phshnd.isValid() && mhnd.isValid()) {

						GA_Size ngdpoints = gdp->getNumPoints();
						bool reget = false;
						if (nactives < ngdpoints) {
							int nptscount = NvFlexExtAllocParticles(consolv->container(), ngdpoints - nactives, indices); //whoa! carefull with that! your luck the mapped buffer is not reallocated during this operation!
							/*for (int npi = 0; npi < nptscount; ++npi) {
								pdat.phases[indices[npi]] = eNvFlexPhaseSelfCollide | eNvFlexPhaseFluid;
							}*/
							reget = true;
						}
						else if (nactives > ngdpoints) {
							NvFlexExtFreeParticles(consolv->container(), nactives - ngdpoints, indices);
							reget = true;
						}
						if (reget) nactives = NvFlexExtGetActiveList(consolv->container(), indices);

						NvFlexExtParticleData pdat = NvFlexExtMapParticleData(consolv->container());

						GA_Offset bst, bed;
						bool stoploop = false;
						for (GA_Iterator it(gdp->getPointRange()); it.blockAdvance(bst, bed);) { //TODO: make it threaded after debugged
							for (GA_Offset off = bst; off < bed; ++off) {
								UT_Vector3F p = phnd.get(off);
								UT_Vector3F v = vhnd.get(off);

								GA_Index idx = gdp->pointIndex(off);
								if (idx >= nactives) {
									stoploop = true;
									break;
								}
								int iid = indices[idx];
								int iid4 = iid * 4;
								int iid3 = iid * 3;
								pdat.particles[iid4 + 0] = p.x();
								pdat.particles[iid4 + 1] = p.y();
								pdat.particles[iid4 + 2] = p.z();
								pdat.particles[iid4 + 3] = mhnd.get(off);
								if (hasRest) {
									UT_Vector3F rst = rhnd.get(off);
									pdat.restParticles[iid4 + 0] = rst.x();
									pdat.restParticles[iid4 + 1] = rst.y();
									pdat.restParticles[iid4 + 2] = rst.z();
									pdat.restParticles[iid4 + 3] = 1.0f; //cannot find in manual what it expects here
								}

								pdat.velocities[iid3 + 0] = v.x();
								pdat.velocities[iid3 + 1] = v.y();
								pdat.velocities[iid3 + 2] = v.z();

								pdat.phases[iid] = phshnd.get(off);
							}
							if (stoploop)break;
						}

						NvFlexExtUnmapParticleData(consolv->container());

						//Push NvFlex data to GPU. since it's async - we need to do it as far from the solver tick as possible to use this time to do CPU work
						NvFlexExtPushToDevice(consolv->container()); //This pushes all from particle data returned by map. so collisions, springs and triangles we can push separately.
						//Also note that as long as we don't call anything with nvFlexExtAssets - we are free to rebind springs manually.

					}
				}

				//NOW PRIMITIVES

				GA_Size nprims = gdp->getNumPrimitives();
				if(nprims>0){//Create and Push SPRINGS and TRIANGLES and RIGIDS
					GA_ROHandleF rlhnd(gdp->findPrimitiveAttribute("restlength"));
					GA_ROHandleF sthnd(gdp->findPrimitiveAttribute("strength"));
					//--
					GA_ROHandleV3 nphnd(gdp->findPointAttribute("N"));
					GA_ROHandleV3 nvhnd(gdp->findVertexAttribute("N"));
					GA_ROHandleV3 nrhnd(gdp->findPrimitiveAttribute("N"));
					const short triNormalType = nrhnd.isValid() ? 3 : (nvhnd.isValid() ? 2 : (nphnd.isValid() ? 1 : 0));
					//--
					GA_ROHandleV3 prtrshnd(gdp->findPrimitiveAttribute("rgd_translation"));
					GA_ROHandleV4 prrothnd(gdp->findPrimitiveAttribute("rgd_rotation"));
					GA_ROHandleV3 vrrsphnd(gdp->findVertexAttribute("rgd_restP"));
					GA_ROHandleV3 vrrsnhnd(gdp->findVertexAttribute("rgd_restN"));
					GA_ROHandleF vrsdfhnd(gdp->findVertexAttribute("rgd_sdf"));
					GA_ROHandleF prstfhnd(gdp->findPrimitiveAttribute("rgd_stiffness"));
					GA_ROHandleI  prgdhnd(gdp->findPrimitiveAttribute("rgd_isrigid"));

					int* indices = nvdata->_indices.get();
					if (nactives == -1)nactives = NvFlexExtGetActiveList(consolv->container(), indices); //do not reread indices if they have already been read before in this geo lock block

					const bool doSprings = rlhnd.isValid() && sthnd.isValid() && (sthnd.getAttribute()->getDataId() != nvdata->_lastGdpStrId || ntopdid != nvdata->_lastGdpTId);
					const bool doTriangles = ntopdid != nvdata->_lastGdpTId;
					const bool doRigids = prtrshnd.isValid() && prrothnd.isValid() && vrrsphnd.isValid() && vrrsnhnd.isValid() && vrsdfhnd.isValid() && prstfhnd.isValid() && prgdhnd.isValid() && (ntopdid != nvdata->_lastGdpTId);
					if (doSprings || doTriangles || doRigids) {
						//calculate primitives of different types
						GA_Size totalspringcount = 0;
						GA_Size totaltricount = 0;
						GA_Size totalrigidcount = 0;
						std::vector<int> rgdPrimSizes;
						rgdPrimSizes.reserve(nprims);//reserve size of total prim count. not particulary good, but it's main RAM and we can assume we have an ass load of it.
						for (GA_Iterator it(gdp->getPrimitiveRange()); !it.atEnd(); ++it) {
							GA_Offset off = *it;
							GA_Size vtxcount = gdp->getPrimitiveVertexCount(off);
							const bool isRigid = prgdhnd.isValid() && prgdhnd.get(off);
							if (isRigid) {
								++totalrigidcount;
								rgdPrimSizes.push_back(vtxcount);
							} else {
								if (vtxcount == 2)++totalspringcount;
								else if (vtxcount == 3)++totaltricount;
							}
						}
						consolv->resizeSpringData(totalspringcount);  std::cout << "total springs count: " << totalspringcount << std::endl;
						consolv->resizeTriangleData(totaltricount);  std::cout << "total triangles count: " << totaltricount << std::endl;
						consolv->resizeRigidData(totalrigidcount, rgdPrimSizes);  std::cout << "total rigids count: " << totalrigidcount << std::endl;

						auto sprdat = consolv->mapSpringData();
						auto tridat = consolv->mapTriangleData();
						auto rgddat = consolv->mapRigidData();
						GA_Size springcount = 0; //TODO: replace with SYS_AtomicCounter in threaded implementation
						GA_Size trianglecount = 0;
						GA_Size rgdcount = 0;
						GA_Size rgdindoff = 0;
						for (GA_Iterator it(gdp->getPrimitiveRange()); !it.atEnd(); ++it) {
							GA_Offset off = *it;
							GA_Size vtxcount = gdp->getPrimitiveVertexCount(off);
							const bool isRigid = prgdhnd.isValid() && prgdhnd.get(off);
							if (isRigid) {
								rgddat.offsets[rgdcount] = rgdindoff;
								for (GA_Iterator vit = gdp->getPrimitive(off)->getVertexRange().begin(); !vit.atEnd(); ++vit) {
									GA_Offset vtxoff = *vit;
									UT_Vector3 vrestP = vrrsphnd.get(vtxoff);
									UT_Vector3 vrestN = vrrsnhnd.get(vtxoff);
									float vsdf = vrsdfhnd.get(vtxoff);

									rgddat.indices[rgdindoff] = indices[gdp->pointIndex(gdp->vertexPoint(vtxoff))];
									rgddat.restPositions[rgdindoff * 3 + 0] = vrestP.x();
									rgddat.restPositions[rgdindoff * 3 + 1] = vrestP.y();
									rgddat.restPositions[rgdindoff * 3 + 2] = vrestP.z();
									rgddat.restNormals[rgdindoff * 4 + 0] = vrestN.x();
									rgddat.restNormals[rgdindoff * 4 + 1] = vrestN.y();
									rgddat.restNormals[rgdindoff * 4 + 2] = vrestN.z();
									rgddat.restNormals[rgdindoff * 4 + 3] = vsdf;

									++rgdindoff;
								}
								float pstiff = prstfhnd.get(off);
								UT_Vector3F ptrs = prtrshnd.get(off);
								UT_Vector4F prot = prrothnd.get(off);
								rgddat.stiffness[rgdcount] = pstiff;
								rgddat.translations[rgdcount * 3 + 0] = ptrs.x();
								rgddat.translations[rgdcount * 3 + 1] = ptrs.y();
								rgddat.translations[rgdcount * 3 + 2] = ptrs.z();
								rgddat.rotations[rgdcount * 4 + 0] = prot.x();
								rgddat.rotations[rgdcount * 4 + 1] = prot.y();
								rgddat.rotations[rgdcount * 4 + 2] = prot.z();
								rgddat.rotations[rgdcount * 4 + 3] = prot.w();

								++rgdcount;
							}
							else {
								if (vtxcount == 2) {
									GA_OffsetListRef vtxs = gdp->getPrimitiveVertexList(off);
									GA_Offset vt0 = vtxs(0);
									GA_Offset vt1 = vtxs(1);

									//TODO: check that if we hit pts limit - we dont write geo indices above the limit!!
									//at this point indices should still be valid
									sprdat.springIds[springcount * 2 + 0] = indices[gdp->pointIndex(gdp->vertexPoint(vt0))];
									sprdat.springIds[springcount * 2 + 1] = indices[gdp->pointIndex(gdp->vertexPoint(vt1))];
									sprdat.springRls[springcount] = rlhnd.get(off);
									sprdat.springSts[springcount] = sthnd.get(off);

									++springcount;
								}
								else if (vtxcount == 3) {
									GA_OffsetListRef vtxs = gdp->getPrimitiveVertexList(off);
									GA_Offset vt0 = vtxs(0);
									GA_Offset vt1 = vtxs(1);
									GA_Offset vt2 = vtxs(2);

									GA_Offset pt0 = gdp->vertexPoint(vt0);
									GA_Offset pt1 = gdp->vertexPoint(vt1);
									GA_Offset pt2 = gdp->vertexPoint(vt2);

									GA_Size tricnt3 = trianglecount * 3;
									tridat.triangleIds[tricnt3 + 0] = indices[gdp->pointIndex(pt0)];
									tridat.triangleIds[tricnt3 + 1] = indices[gdp->pointIndex(pt1)];
									tridat.triangleIds[tricnt3 + 2] = indices[gdp->pointIndex(pt2)];

									if (triNormalType > 0) {
										UT_Vector3F n;
										if (triNormalType == 1) {
											n = nphnd.get(pt0);
											n += nphnd.get(pt1);
											n += nphnd.get(pt2);
											n.normalize();
										}
										else if (triNormalType == 2) {
											n = nvhnd.get(vt0);
											n += nvhnd.get(vt1);
											n += nvhnd.get(vt2);
											n.normalize();
										}
										else if (triNormalType == 3) {
											n = nrhnd.get(off);
										}
										tridat.triangleNms[tricnt3 + 0] = n.x();
										tridat.triangleNms[tricnt3 + 1] = n.y();
										tridat.triangleNms[tricnt3 + 2] = n.z();
									}

									++trianglecount;
								}
							}
						}
						rgddat.offsets[rgdcount] = rgdindoff;
						consolv->unmapSpringData();
						consolv->unmapTriangleData();
						consolv->unmapRigidData();


						consolv->pushSpringsToDevice();//Note that we should do this only if change occured in springs. for now we do not detect those changes, so we push always.
						consolv->pushTrianglesToDevice(triNormalType > 0);
						consolv->pushRigidsToDevice();

					}
					else {//TODO: imagine geometry changed and no such attribs now - we need to destroy nvFles springs/triangles/rigids and push zero arrays to device as well!
						//condition changed, for now this destruction of constraints is commented out, the case described in TODO above is not very probable
						/*consolv->resizeSpringData(0);
						consolv->resizeTriangleData(0);
						consolv->resizeRigidData(0,std::vector<int>());
						consolv->pushSpringsToDevice();
						consolv->pushTrianglesToDevice(false);
						consolv->pushRigidsToDevice();*/
					}

				}
				else {//END SPRINGS AND TRIANGLES AND RIGIDS
					consolv->resizeSpringData(0);
					consolv->resizeTriangleData(0);
					consolv->resizeRigidData(0, std::vector<int>());
					consolv->pushSpringsToDevice();
					consolv->pushTrianglesToDevice(false);
					consolv->pushRigidsToDevice();
				}

			}
		}

		
		
		
		// Updating collision Geometry.
		// TODO: kill/deactivate meshes that are no longer in relationships
		{
			NvFlexHCollisionData* colldata = consolv->collisionData();
			colldata->mapall();
			/*
			colldata->addSphere("test");
			colldata->getSphere("test").collgeo->radius = 1.0f;
			colldata->getSphere("test").position->y = 1.0f;
			colldata->getSphere("test").prevposition->y = 1.0f;
			*/

			//find collision relationships and build collisions
			SIM_ConstObjectArray affs;
			obj->getConstAffectors(affs, "SIM_RelationshipCollide");
			for (exint afi = 0; afi < affs.entries(); ++afi) {
				const SIM_Object* aff = affs(afi);
				if (aff == obj)continue;
				//std::cout << aff->getName() << " : " << aff->getObjectId() << std::endl;
				const SIM_Geometry*affgeo = SIM_DATA_GETCONST(*aff, SIM_GEOMETRY_DATANAME, SIM_Geometry);
				if (affgeo == NULL)continue;

				GU_DetailHandleAutoReadLock hlk(affgeo->getGeometry());
				const GU_Detail *gdp = hlk.getGdp();
				int64 pDataId=gdp->getP()->getDataId();

				std::string objidname = std::to_string(aff->getObjectId());

				if(pDataId != colldata->getStoredHash(objidname)){
					std::cout << "updating collision mesh " << objidname << std::endl;
					colldata->setStoredHash(objidname, pDataId);
					colldata->addTriangleMesh(objidname);
					NvfTrimeshGeo trigeo=colldata->getTriangleMesh(objidname);

					NvFlexHTriangleMeshAutoMapper tmeshlock(trigeo.collgeo);


					GA_Offset off;
					tmeshlock.setVertexCount(gdp->getNumPoints());
					Vec3* trigeop = tmeshlock.vertices();
					float* trigeolw = tmeshlock.lower();
					float* trigeoup = tmeshlock.upper();
					trigeoup[0] = trigeoup[1] = trigeoup[2] = -FLT_MAX;
					trigeolw[0] = trigeolw[1] = trigeolw [2] = FLT_MAX;
					GA_FOR_ALL_PTOFF(gdp, off) {
						UT_Vector3 p=gdp->getPos3(off);
						Vec3* currtgp = trigeop + gdp->pointIndex(off);
						currtgp->x = p.x();
						currtgp->y = p.y();
						currtgp->z = p.z();
						trigeolw[0] = std::min(p.x(), trigeolw[0]);
						trigeolw[1] = std::min(p.y(), trigeolw[1]);
						trigeolw[2] = std::min(p.z(), trigeolw[2]);
						trigeoup[0] = std::max(p.x(), trigeoup[0]);
						trigeoup[1] = std::max(p.y(), trigeoup[1]);
						trigeoup[2] = std::max(p.z(), trigeoup[2]);
					}

					//now we need to calculate triangles
					GA_Size tricount = 0;
					for (GA_Iterator it(gdp->getPrimitiveRange()); !it.atEnd(); ++it) {
						tricount += std::max(gdp->getPrimitiveVertexCount(*it) - 2, GA_Size(0));
					}
					tmeshlock.setTrianglesCount(tricount);
					//now set triangles!
					size_t i = 0;
					int* trigeot = tmeshlock.triangles();
					for (GA_Iterator it(gdp->getPrimitiveRange()); !it.atEnd(); ++it) {
						GA_OffsetListRef pvlr=gdp->getPrimitiveVertexList(*it);
						GA_Index sttidx = -1;
						GA_Index prvidx = -1;
						for (int vi = 0; vi < pvlr.entries(); ++vi) {
							GA_Index idx = gdp->pointIndex(gdp->vertexPoint(pvlr(vi)));//gdp->vertexIndex(pvlr(vi));//
							if (vi == 0)sttidx = idx;
							else if (vi > 1) {
								//invert order cuz houdini goes clockwise
								trigeot[i++] = sttidx;
								trigeot[i++] = idx;
								trigeot[i++] = prvidx;
							}
							prvidx = idx;
						}
					}

				}
			}

			colldata->unmapall();
			colldata->setCollisionData(consolv->solver());
		}


		nvparams.numIterations = getIterations();
		int substeps = getSubsteps();
		NvFlexGetParams(consolv->solver(), &nvparams);
		updateSolverParams();
		//Find and apply gravity
		{
			SIM_ConstDataArray gravities;
			obj->filterConstSubData(gravities, 0, SIM_DataFilterByType("SIM_ForceGravity"), SIM_FORCES_DATANAME, SIM_DataFilterNone());
			for (exint i = 0; i < gravities.entries(); ++i) {
				const SIM_ForceGravity* force = SIM_DATA_CASTCONST(gravities(i), SIM_ForceGravity);
				if (force == NULL)continue;
				UT_Vector3 outForce, outTorque;
				force->getForce(*obj, UT_Vector3(), UT_Vector3(), UT_Vector3(), 1.0f, outForce,outTorque);

				nvparams.gravity[0] += outForce.x();
				nvparams.gravity[1] += outForce.y();
				nvparams.gravity[2] += outForce.z();
			}
		}
		NvFlexSetParams(consolv->solver(), &nvparams);

		//NvFlexExtTickContainer(consolv->container(), timestep, substeps, false);
		std::cout << "timestep " << timestep << std::endl;
		NvFlexUpdateSolver(consolv->solver(), timestep, substeps, false);

		NvFlexExtPullFromDevice(consolv->container());
		if (consolv->getRigidCount() > 0)consolv->pullRigidsFromDevice();

		SIM_GeometryCopy *newgeo=SIM_DATA_CREATE(*obj, "Geometry", SIM_GeometryCopy, SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE);
		if (newgeo == NULL)continue;//TODO: show error;
		GU_DetailHandleAutoWriteLock lock(newgeo->getOwnGeometry());
		if (lock.isValid()) {
			GU_Detail *gdp = lock.getGdp();

			int* const iindex = nvdata->_indices.get(); //TODO: indices dont change - if we got them before solve - keep them!
			const int nactives = NvFlexExtGetActiveList(consolv->container(), iindex); //HERE I REEEEALLY HOPE nooe accesses it right now
			
			const bool recreateGeo = nactives != gdp->getNumPoints(); //This basically should never happen with current workflow

			if (recreateGeo) {
				std::cout << "recreate==true. geo inconsistent. " << nactives << " vs " << gdp->getNumPoints() << std::endl;
			}

			if(recreateGeo)gdp->stashAll();

			//GA_RWAttributeRef vatt = gdp->findPointAttribute("v");//
			GA_RWAttributeRef vatt = gdp->findFloatTuple(GA_ATTRIB_POINT, "v", 3, 3);
			if (!vatt.isValid()) {
				vatt = gdp->addFloatTuple(GA_ATTRIB_POINT, "v", 3, GA_Defaults(0));
				vatt.setTypeInfo(GA_TYPE_VECTOR);
			}
			//GA_RWAttributeRef iidatt = gdp->findPointAttribute("iid");
			GA_RWAttributeRef iidatt = gdp->findIntTuple(GA_ATTRIB_POINT, "iid", 1, 1);
			if (!iidatt.isValid()) {
				iidatt = gdp->addIntTuple(GA_ATTRIB_POINT, "iid", 1, GA_Defaults(-1));
			}
			//GA_RWAttributeRef phsatt = gdp->findPointAttribute("phs");
			GA_RWAttributeRef phsatt = gdp->findIntTuple(GA_ATTRIB_POINT, "phs", 1, 1);
			if (!phsatt.isValid()) {
				phsatt = gdp->addIntTuple(GA_ATTRIB_POINT, "phs", 1, GA_Defaults(0));
			}
			GA_RWHandleV3 vhd(vatt);
			GA_RWHandleI iidhd(iidatt);
			GA_RWHandleI phshd(phsatt);

			
			NvFlexExtParticleData pdat = NvFlexExtMapParticleData(consolv->container());	//mapping
			
			// get indices and go through active indices!
			if(recreateGeo)GA_Offset off = gdp->appendPointBlock(nactives);

			GA_Offset ostt, oend;
			for (GA_Iterator oit(gdp->getPointRange()); oit.blockAdvance(ostt, oend);) { //TODO: make it threaded after debugged
				for (GA_Offset curroff = ostt; curroff < oend; ++curroff) {
					UT_Vector3 pp;
					int ii = iindex[gdp->pointIndex(curroff)];
					pp.assign(pdat.particles[ii * 4 + 0], pdat.particles[ii * 4 + 1], pdat.particles[ii * 4 + 2]);
					gdp->setPos3(curroff, pp);
					pp.assign(pdat.velocities[ii * 3 + 0], pdat.velocities[ii * 3 + 1], pdat.velocities[ii * 3 + 2]);
					vhd.set(curroff, pp);
					iidhd.set(curroff, ii);
					phshd.set(curroff, pdat.phases[ii]);
				}
			}
			NvFlexExtUnmapParticleData(consolv->container());//unmapping


			//Now update rigids
			GA_ROHandleI prgdhnd(gdp->findPrimitiveAttribute("rgd_isrigid"));
			if (prgdhnd.isValid() && consolv->getRigidCount() > 0) {
				GA_RWAttributeRef ptrsat = gdp->findFloatTuple(GA_ATTRIB_PRIMITIVE, "rgd_translation", 3, 3);
				if (!ptrsat.isValid()) {
					ptrsat = gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, "rgd_translation", 3);
				}
				GA_RWAttributeRef protat = gdp->findFloatTuple(GA_ATTRIB_PRIMITIVE, "rgd_rotation", 4, 4);
				if (!protat.isValid()) {
					protat = gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, "rgd_rotation", 3);
				}
				GA_RWHandleV3 ptrshnd(ptrsat);
				GA_RWHandleV4 prothnd(protat);

				auto rgdtransdata = consolv->mapRigidTransData();

				GA_Size rigidNum = 0;
				for (GA_Iterator pit(gdp->getPrimitiveRange()); !pit.atEnd(); ++pit) {
					GA_Offset off = *pit;
					if (prgdhnd.get(off)) {
						UT_Vector3F trs;
						UT_Vector4F rot;
						trs.assign(rgdtransdata.translations[rigidNum * 3 + 0], rgdtransdata.translations[rigidNum * 3 + 1], rgdtransdata.translations[rigidNum * 3 + 2]);
						rot.assign(rgdtransdata.rotations[rigidNum * 4 + 0], rgdtransdata.rotations[rigidNum * 4 + 1], rgdtransdata.rotations[rigidNum * 4 + 2], rgdtransdata.rotations[rigidNum * 4 + 3]);
						ptrshnd.set(off, trs);
						prothnd.set(off, rot);
						++rigidNum;
					}
				}

				consolv->unmapRigidTransData();

			}
			//END UPDATE RIGIDS

			if(recreateGeo)gdp->destroyStashed();
			gdp->bumpAllDataIds();
			//gdp->getAttributes().bumpAllDataIds(GA_ATTRIB_POINT);
			//gdp->getAttributes().bumpAllDataIds(GA_ATTRIB_PRIMITIVE);
			nvdata->_lastGdpPId = gdp->getP()->getDataId();
			nvdata->_lastGdpTId = gdp->getTopology().getDataId();
			{
				GA_Attribute *str=gdp->findPrimitiveAttribute("strength");
				if (str != NULL)nvdata->_lastGdpStrId = str->getDataId();
			}

		}

		
	}

	return SIM_SOLVER_SUCCESS;
}

void SIM_NvFlexSolver::initializeSubclass()
{
	SIM_Solver::initializeSubclass();
	//if(SIM_NvFlexData::nvFlexLibrary == NULL)SIM_NvFlexData::nvFlexLibrary=NvFlexInit();
	
	//if(!nvparams)nvparams.reset(new NvFlexParams); //we don't need to reset it every every time i guess
	
	//NvFlexGetParams(nvsolver->solver(), nvparams.get());
	updateSolverParams();

	//NvFlexSetParams(nvsolver->solver(), nvparams.get());
}

void SIM_NvFlexSolver::updateSolverParams() {

	nvparams.radius = getRadius();

	nvparams.gravity[0] = 0;
	nvparams.gravity[1] = 0;
	nvparams.gravity[2] = 0;
	nvparams.fluidRestDistance = nvparams.radius * getFluidRestDistanceMult();
	nvparams.solidRestDistance = nvparams.fluidRestDistance; //TODO: make a parameter for this
	nvparams.numIterations = getIterations();
	nvparams.maxSpeed = getMaxSpeed(); //FLT_MAX;
	nvparams.maxAcceleration = getMaxAcceleration(); //1000.0f;
	nvparams.sleepThreshold = getSleepThreshold();//0.0f;
	nvparams.fluid = (bool)getDoFluid();//true;

	nvparams.viscosity = getViscosity();
	nvparams.dynamicFriction = getDynamicFriction();
	nvparams.staticFriction = getStaticFriction();
	nvparams.particleFriction = getParticleFriction();//0.0f; // scale friction between particles by default
	nvparams.freeSurfaceDrag = getFreeSurfaceDrag(); //0.0f;
	nvparams.drag = getDrag();// 0.0f;
	nvparams.lift = getLift();// 0.0f;

	nvparams.numPlanes = getPlanesCount();
	(Vec4&)nvparams.planes[0] = Vec4(0.0f, 1.0f, 0.0f, 0.0f);
	(Vec4&)nvparams.planes[1] = Vec4(0.0f, 0.0f, 1.0f, 4);
	(Vec4&)nvparams.planes[2] = Vec4(1.0f, 0.0f, 0.0f, 2);
	(Vec4&)nvparams.planes[3] = Vec4(-1.0f, 0.0f, 0.0f, 2);
	(Vec4&)nvparams.planes[4] = Vec4(0.0f, 0.0f, -1.0f, 4);
	//(Vec4&)nvparams->planes[5] = Vec4(0.0f, -1.0f, 0.0f, g_sceneUpper.y);

	nvparams.anisotropyScale = 0.0f;
	nvparams.anisotropyMin = 0.1f;
	nvparams.anisotropyMax = 2.0f;
	nvparams.smoothing = 0.0f;

	nvparams.shapeCollisionMargin = getShapeCollisionMargin();
	nvparams.particleCollisionMargin = getParticleCollisionMargin();
	nvparams.collisionDistance = getCollisionDistance();
	nvparams.shockPropagation = getShockPropagation();

	nvparams.relaxationMode = eNvFlexRelaxationLocal;
	nvparams.relaxationFactor = getRelaxationFactor();// 1.0f;
	nvparams.solidPressure = getSolidPressure();// 0.1f;
	nvparams.adhesion = getAdhesion();
	nvparams.cohesion = getCohesion();
	nvparams.surfaceTension = getSurfaceTension();
	nvparams.vorticityConfinement = getVorticityConfinement();// 0.0f;
	nvparams.buoyancy = getBuoyancy();// 1.0f;

	//nvparams.restitution = 0;
	//nvparams.sleepThreshold = 0;
	//nvparams.shockPropagation = 0;
	//nvparams.dissipation = 0;
	//nvparams.damping = 0;

	UT_Vector3F wind = getWind();
	nvparams.wind[0] = wind.x();
	nvparams.wind[1] = wind.y();
	nvparams.wind[2] = wind.z();
}

void SIM_NvFlexSolver::makeEqualSubclass(const SIM_Data * source)
{
	SIM_Solver::makeEqualSubclass(source);
	const SIM_NvFlexSolver *src = SIM_DATA_CASTCONST(source, SIM_NvFlexSolver);
	if (src == NULL) {
		return;
	}

	nvparams = src->nvparams;
}





const SIM_DopDescription* SIM_NvFlexSolver::getDescriptionForFucktory() {
	static PRM_Name radius_name("radius", "Radius");
	static PRM_Name iterations_name("iterations", "Constraint Iterations Count");
	static PRM_Name substeps_name("substeps", "Substeps Count");
	static PRM_Name maxSpeed_name("maxSpeed", "Maximum Particle Speed");
	static PRM_Name maxAcceleration_name("maxAcceleration", "Maximum Particle Acceleration");
	static PRM_Name sleepThreshold_name("sleepThreshold", "Particle Sleep Threshold");
	static PRM_Name doFluid_name("fluid", "Do Fluids");

	static PRM_Name fluidRestDistanceMult_name("fluidRestDistanceMult", "Rest Distance Multiplier");
	static PRM_Name planesCount_name("planesCount", "Planes Count");
	static PRM_Name adhesion_name("adhesion", "Adhesion");
	static PRM_Name cohesion_name("cohesion", "Cohesion");
	static PRM_Name surfaceTension_name("surfaceTension", "Surface Tension");
	static PRM_Name viscosity_name("viscosity", "Viscosity");
	static PRM_Name relaxationFactor_name("relaxationFactor", "Relaxation Factor");
	static PRM_Name solidPressure_name("solidPressure", "Solid Pressure");
	static PRM_Name vorticityConfinement_name("vorticityConfinement", "Vorticity Confinement");
	static PRM_Name buoyancy_name("buoyancy", "Buoyancy");

	static PRM_Name dynamicfriction_name("dynamicfriction", "Dynamic Friction");
	static PRM_Name staticfriction_name("staticfriction", "Static Friction");
	static PRM_Name particleFriction_name("particleFriction", "Particle Friction");
	static PRM_Name freeSurfaceDrag_name("freeSurfaceDrag", "Free Surface Drag");
	static PRM_Name drag_name("drag", "Cloth Drag");
	static PRM_Name lift_name("lift", "Cloth Lift");
	static PRM_Name wind_name("wind", "Wind");

	static PRM_Name shapeCollisionMargin_name("shapeCollisionMargin", "Shape Collision Margin");
	static PRM_Name particleCollisionMargin_name("particleCollisionMargin", "Particle Collision Margin");
	static PRM_Name collisionDistance_name("collisionDistance", "Collision Distance");

	static PRM_Name shockPropagation_name("shockPropagation", "Shock Propagation");
	

	static PRM_Default radius_default(0.2f);
	static PRM_Default iterations_default(3);
	static PRM_Default substeps_default(6);
	static PRM_Default maxSpeed_default(FLT_MAX);
	static PRM_Default maxAcceleration_default(1000.0f);
	static PRM_Default fluidRestDistanceMult_defaults(0.55f);
	static PRM_Default planesCount_defaults(1);
	static PRM_Default adhesion_defaults(0.0f);
	static PRM_Default cohesion_defaults(0.025f);
	static PRM_Default surfaceTension_defaults(0.0f);
	static PRM_Default viscosity_defaults(0.0f);
	static PRM_Default relaxationFactor_default(1.0f);
	static PRM_Default solidPressure_default(0.1f);
	static PRM_Default vorticityConfinement_default(0.0f);
	static PRM_Default buoyancy_default(1.0f);

	static PRM_Default dynamicfriction_defaults(0.1f);
	static PRM_Default staticfriction_defaults(0.0f);
	static PRM_Default particleFriction_defaults(0.0f);
	static PRM_Default shapeCollisionMargin_defaults(0.05f);
	static PRM_Default particleCollisionMargin_defaults(0.0f);
	static PRM_Default collisionDistance_defaults(0.0275f);

	static PRM_Default zero_defaults(0.0f);
	static PRM_Default one_defaults(1.0f);

	static PRM_Range iterations_range(PRM_RANGE_RESTRICTED, 1, PRM_RANGE_UI, 16);
	static PRM_Range substeps_range(PRM_RANGE_RESTRICTED, 1, PRM_RANGE_UI, 16);
	static PRM_Range maxSpeed_range(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, FLT_MAX);
	static PRM_Range maxAcceleration_range(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 1000);
	static PRM_Range planesCount_range(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_RESTRICTED, 5);

	static PRM_Range zeroOne_range(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 1.0f);


	//seps
	static PRM_Name sep0("sep0", "sep0");
	static PRM_Name sep1("sep1", "sep1");
	static PRM_Name sep2("sep2", "sep2");
	static PRM_Name sep3("sep3", "sep3");
	//endseps

	static PRM_Template prms[] = {
		PRM_Template(PRM_FLT, 1, &radius_name, &radius_default),
		PRM_Template(PRM_INT, 1, &iterations_name, &iterations_default, 0, &iterations_range),
		PRM_Template(PRM_INT, 1, &substeps_name, &substeps_default, 0, &substeps_range),
		PRM_Template(PRM_FLT_LOG, 1, &maxSpeed_name, &maxSpeed_default, 0, &maxSpeed_range),
		PRM_Template(PRM_FLT, 1, &maxAcceleration_name, &maxAcceleration_default, 0, &maxAcceleration_range),
		PRM_Template(PRM_FLT, 1, &sleepThreshold_name, &zero_defaults),
		PRM_Template(PRM_TOGGLE, 1, &doFluid_name,&one_defaults),
		PRM_Template(PRM_SEPARATOR, 1, &sep0),
		PRM_Template(PRM_FLT, 1, &fluidRestDistanceMult_name, &fluidRestDistanceMult_defaults),
		PRM_Template(PRM_INT, 1, &planesCount_name,&planesCount_defaults,0,&planesCount_range),
		PRM_Template(PRM_SEPARATOR, 1, &sep1),
		PRM_Template(PRM_FLT, 1, &adhesion_name, &adhesion_defaults),
		PRM_Template(PRM_FLT, 1, &cohesion_name, &cohesion_defaults),
		PRM_Template(PRM_FLT, 1, &surfaceTension_name, &surfaceTension_defaults),
		PRM_Template(PRM_FLT, 1, &viscosity_name, &viscosity_defaults),
		PRM_Template(PRM_FLT, 1, &relaxationFactor_name, &relaxationFactor_default),
		PRM_Template(PRM_FLT, 1, &solidPressure_name, &solidPressure_default),
		PRM_Template(PRM_FLT, 1, &vorticityConfinement_name, &vorticityConfinement_default),
		PRM_Template(PRM_FLT, 1, &buoyancy_name, &buoyancy_default),
		PRM_Template(PRM_SEPARATOR, 1, &sep2),
		PRM_Template(PRM_FLT, 1, &dynamicfriction_name, &dynamicfriction_defaults),
		PRM_Template(PRM_FLT, 1, &staticfriction_name, &staticfriction_defaults),
		PRM_Template(PRM_FLT, 1, &particleFriction_name, &particleFriction_defaults),
		PRM_Template(PRM_FLT, 1, &freeSurfaceDrag_name, &zero_defaults, 0, &zeroOne_range),
		PRM_Template(PRM_FLT, 1, &drag_name, &zero_defaults, 0, &zeroOne_range),
		PRM_Template(PRM_FLT, 1, &lift_name, &zero_defaults, 0, &zeroOne_range),
		PRM_Template(PRM_FLT, 3, &wind_name, 0),
		PRM_Template(PRM_SEPARATOR, 1, &sep3),
		PRM_Template(PRM_FLT, 1, &shapeCollisionMargin_name, &shapeCollisionMargin_defaults),
		PRM_Template(PRM_FLT, 1, &particleCollisionMargin_name, &particleCollisionMargin_defaults),
		PRM_Template(PRM_FLT, 1, &collisionDistance_name, &collisionDistance_defaults),
		PRM_Template(PRM_FLT, 1, &shockPropagation_name, &zero_defaults),
		PRM_Template()
	};

	static SIM_DopDescription desc(true, "nvflexSolver", "NvFlex Solver", "Solver", classname(), prms);
	return &desc;
}


SIM_NvFlexSolver::SIM_NvFlexSolver(const SIM_DataFactory * fack) :SIM_Solver(fack), SIM_OptionsUser(this){}

SIM_NvFlexSolver::~SIM_NvFlexSolver(){}