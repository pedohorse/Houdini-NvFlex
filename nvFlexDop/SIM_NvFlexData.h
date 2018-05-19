#pragma once
#include <SIM/SIM_Data.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_DataUtils.h>
#include <SIM/SIM_DopDescription.h>

#include <NvFlex.h>
#include <NvFlexExt.h>
#include <../core/types.h>
#include <../core/maths.h>

#include "NvFlexHCollisionData.h"

//a little wrapper to keep track of the library
class NvFlexHLibraryHolder {
public:
	NvFlexHLibraryHolder();
	virtual ~NvFlexHLibraryHolder();
protected:
	static NvFlexLibrary* nvFlexLibrary;
	static bool cudaContextCreated;
private:
	static GA_Size _instanceCount;
};

//wrapper done

class SIM_NvFlexSolver; //fwd decl

class SIM_NvFlexData:public SIM_Data, public SIM_OptionsUser, public NvFlexHLibraryHolder
{
public:
	class NvFlexContainerWrapper {
	public:
		typedef struct NvFlexHSpringData {
			int* const springIds;
			float* const springRls;
			float* const springSts;

			NvFlexHSpringData(int*sid, float*srl, float*sts):springIds(sid),springRls(srl),springSts(sts){}
		} NvFlexHSpringData;

		typedef struct NvFlexHTriangleData {
			int* const triangleIds;
			float* const triangleNms;

			NvFlexHTriangleData(int*tid, float*tnm):triangleIds(tid),triangleNms(tnm){}
		} NvFlexHTriangleData;

		typedef struct NvFlexHRigidData {
			int* offsets; //numRigids+1
			int* indices;
			float* restPositions; //numRigids*3
			float* restNormals; //numRigids*4 (normal.xyz;sdf)
			float* stiffness; //numRigids
			float* rotations; //numRigids*4 (quat)
			float* translations;

			NvFlexHRigidData(int*off, int*ind, float*rep, float*ren, float*stf, float*rot, float*trs) :offsets(off), indices(ind), restPositions(rep), restNormals(ren), stiffness(stf), rotations(rot), translations(trs) {};
		} NvFlexRigidData;

		typedef struct NvFlexHRigidTransData {
			int rigidsCount;
			float* rotations; //numRigids*4 (quat)
			float* translations;
			NvFlexHRigidTransData(float*trs, float*rot, int count) :translations(trs), rotations(rot), rigidsCount(count) {};
		} NvFlexHRigidTransData;

		explicit NvFlexContainerWrapper(NvFlexLibrary*lib, int maxParticles, int MaxDiffuseParticles, int maxNeighbours = 96):_springIndices(lib),_springRestLengths(lib),_springStrenghts(lib), _triangleIndices(lib),_triangleNormals(lib), _rgdOffsets(lib), _rgdIndices(lib), _rgdRestPositions(lib), _rgdRestNormals(lib), _rgdStiffness(lib), _rgdRotations(lib), _rgdTranslations(lib) {
			_slv = NvFlexCreateSolver(lib, maxParticles, MaxDiffuseParticles, maxNeighbours);
			if (_slv == NULL)throw std::runtime_error("NULL NVFLEX SOLVER!");
			_cont = NvFlexExtCreateContainer(lib, _slv, maxParticles);
			if (_cont == NULL)throw std::runtime_error("NULL NVFLEX CONTAINER!");
			_colld = new NvFlexHCollisionData(lib);
		}
		NvFlexContainerWrapper(NvFlexContainerWrapper&) = delete;
		~NvFlexContainerWrapper() {
			//NvFlexAcquireContext(SIM_NvFlexData::nvFlexLibrary);
			//no aquire cuz we assume the destructor wrapper is responsible for that
			NvFlexExtDestroyContainer(_cont);
			NvFlexDestroySolver(_slv);
			delete _colld;
			//NvFlexRestoreContext(SIM_NvFlexData::nvFlexLibrary);
		}

		NvFlexSolver* solver() { return _slv; }
		NvFlexExtContainer * container() { return _cont; }
		NvFlexHCollisionData* collisionData() { return _colld; }

		//springs
		int getSpringsCount()const { return _springRestLengths.size(); }
		void resizeSpringData(int newSize) {
			/// be sure data is NOT MAPPED before here
			/// cuz all previous pointers will be invalidated
			_springIndices.map();
			_springRestLengths.map();
			_springStrenghts.map();

			_springIndices.resize(2*newSize);
			_springRestLengths.resize(newSize);
			_springStrenghts.resize(newSize);

			_springIndices.unmap();
			_springRestLengths.unmap();
			_springStrenghts.unmap();
		}
		NvFlexHSpringData mapSpringData(){
			_springIndices.map();
			_springRestLengths.map();
			_springStrenghts.map();
			return NvFlexHSpringData(_springIndices.mappedPtr, _springRestLengths.mappedPtr, _springStrenghts.mappedPtr);
		}
		void unmapSpringData() {
			_springIndices.unmap();
			_springRestLengths.unmap();
			_springStrenghts.unmap();
		}
		void pushSpringsToDevice() {
			NvFlexSetSprings(_slv, _springIndices.buffer, _springRestLengths.buffer, _springStrenghts.buffer, _springRestLengths.size());
		}

		//triangles
		int getTrianglesCount()const { return _triangleIndices.size() / 3; }
		void resizeTriangleData(int newSize) {
			/// be sure data is NOT MAPPED before here
			/// cuz all previous pointers will be invalidated
			_triangleIndices.map();
			_triangleNormals.map();

			_triangleIndices.resize(3*newSize);
			_triangleNormals.resize(3*newSize);

			_triangleIndices.unmap();
			_triangleNormals.unmap();
		}
		NvFlexHTriangleData mapTriangleData() {
			_triangleIndices.map();
			_triangleNormals.map();
			return NvFlexHTriangleData(_triangleIndices.mappedPtr, _triangleNormals.mappedPtr);
		}
		void unmapTriangleData() {
			_triangleIndices.unmap();
			_triangleNormals.unmap();
		}
		void pushTrianglesToDevice(bool pushNormals = true) {
			NvFlexSetDynamicTriangles(_slv, _triangleIndices.buffer, pushNormals ? _triangleNormals.buffer : NULL, _triangleIndices.size() / 3);
		}

		//rigids
		int getRigidCount()const { return _rgdStiffness.size(); }
		int getRigidIndicesCount()const { return _rgdIndices.size(); }
		void resizeRigidData(const int numbodies, const std::vector<int>& bodysizes) {
			// data must not be mapped !
			// pointers will be fucked !
			
			//TODO: assert numbodies == bodysizes.size()
			_rgdOffsets.map();
			_rgdIndices.map();
			_rgdRestPositions.map();
			_rgdRestNormals.map();
			_rgdStiffness.map();
			_rgdRotations.map();
			_rgdTranslations.map();


			_rgdOffsets.resize(numbodies + 1);
			exint ind = 0;
			for (size_t i = 0; i < numbodies; ++i) {
				_rgdOffsets[i] = ind;
				ind += bodysizes[i];
			}
			_rgdOffsets[numbodies] = ind;

			_rgdIndices.resize(ind);

			_rgdRestPositions.resize(ind * 3);
			_rgdRestNormals.resize(ind * 4);
			_rgdStiffness.resize(numbodies);
			_rgdRotations.resize(numbodies * 4);
			_rgdTranslations.resize(numbodies * 3);

			_rgdOffsets.unmap();
			_rgdIndices.unmap();
			_rgdRestPositions.unmap();
			_rgdRestNormals.unmap();
			_rgdStiffness.unmap();
			_rgdRotations.unmap();
			_rgdTranslations.unmap();
		}
		NvFlexHRigidData mapRigidData() {
			_rgdOffsets.map();
			_rgdIndices.map();
			_rgdRestPositions.map();
			_rgdRestNormals.map();
			_rgdStiffness.map();
			_rgdRotations.map();
			_rgdTranslations.map();
			return NvFlexHRigidData(_rgdOffsets.mappedPtr, _rgdIndices.mappedPtr, _rgdRestPositions.mappedPtr, _rgdRestNormals.mappedPtr, _rgdStiffness.mappedPtr, _rgdRotations.mappedPtr, _rgdTranslations.mappedPtr);
		}
		void unmapRigidData() {
			_rgdOffsets.unmap();
			_rgdIndices.unmap();
			_rgdRestPositions.unmap();
			_rgdRestNormals.unmap();
			_rgdStiffness.unmap();
			_rgdRotations.unmap();
			_rgdTranslations.unmap();
		}
		NvFlexHRigidTransData mapRigidTransData() { //maps just the translation+rotation data instead of the whole bunch
			_rgdRotations.map();
			_rgdTranslations.map();
			return NvFlexHRigidTransData(_rgdTranslations.mappedPtr, _rgdRotations.mappedPtr, _rgdStiffness.size());
		}
		void unmapRigidTransData() {
			_rgdRotations.unmap();
			_rgdTranslations.unmap();
		}
		void pushRigidsToDevice() {
			NvFlexSetRigids(_slv, _rgdOffsets.buffer, _rgdIndices.buffer, _rgdRestPositions.buffer, _rgdRestNormals.buffer, _rgdStiffness.buffer, _rgdRotations.buffer, _rgdTranslations.buffer, _rgdStiffness.size(), _rgdIndices.size());
		}
		void pullRigidsFromDevice() {
			//pull rigid transformations recalculated by solver
			//WARNING! buffers MUST already be properly resized!
			NvFlexGetRigidTransforms(_slv, _rgdRotations.buffer, _rgdTranslations.buffer);
		}

	private:
		NvFlexHCollisionData* _colld;
		NvFlexSolver* _slv;
		NvFlexExtContainer* _cont;

		//springs
		NvFlexVector<int> _springIndices;
		NvFlexVector<float> _springRestLengths;
		NvFlexVector<float> _springStrenghts;
		//triangles
		NvFlexVector<int> _triangleIndices;
		NvFlexVector<float> _triangleNormals;
		//rigids
		NvFlexVector<int> _rgdOffsets; //numRigids+1
		NvFlexVector<int> _rgdIndices;
		NvFlexVector<float> _rgdRestPositions; //numIndices*3
		NvFlexVector<float> _rgdRestNormals; //numIndices*4 (normal.xyz;sdf)
		NvFlexVector<float> _rgdStiffness; //numRigids
		NvFlexVector<float> _rgdRotations; //numRigids*4 (quat)
		NvFlexVector<float> _rgdTranslations; //numRigids*3
	};

	
	//static NvFlexLibrary* nvFlexLibrary;

	GETSET_DATA_FUNCS_I("maxpts", MaxPtsCount);

	std::shared_ptr<NvFlexContainerWrapper> nvdata;
public:
	inline bool isNvValid() { return _valid; }

protected:
	explicit SIM_NvFlexData(const SIM_DataFactory*fack);
	virtual ~SIM_NvFlexData();

	void initializeSubclass();
	void makeEqualSubclass(const SIM_Data* source);

private:
	bool _valid;
private: //for a friend
	std::shared_ptr<int> _indices;
	int64 _lastGdpPId,_lastGdpTId;

	friend class SIM_NvFlexSolver;
	friend void delete_NvFlexContainerWrapper(SIM_NvFlexData::NvFlexContainerWrapper *wrp);
	friend bool acquireCudaContext();
	friend bool releaseCudaContext();

private:
	static const SIM_DopDescription* getDescriptionForFucktory();

	DECLARE_STANDARD_GETCASTTOTYPE()
	DECLARE_DATAFACTORY(SIM_NvFlexData, SIM_Data, "data for nvFlex sim", getDescriptionForFucktory());

};

