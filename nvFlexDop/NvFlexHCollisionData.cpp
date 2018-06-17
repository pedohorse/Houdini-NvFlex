#include "NvFlexHCollisionData.h"
#include "NvFlexHTriangleMesh.h"


bool NvFlexHCollisionData::hasKey(const std::string &key) {
	return collmap.find(key) != collmap.end();
}

bool NvFlexHCollisionData::removeItem(const std::string &key) {
	// buffers must be mapped!
	if (!hasKey(key))return false;
	int id = collmap.at(key);
	collmap.erase(key);
	hashmap.erase(key);
	for (auto it = collmap.begin(); it != collmap.end(); ++it) {
		int cid = it->second;
		if (cid > id) collmap[it->first] -= 1;
	}
	if (flagvec[id] & eNvFlexShapeFlagTypeMask == eNvFlexShapeTriangleMesh) {
		NvFlexTriangleMeshId mid = colgeovec[id].triMesh.mesh;
		delete meshmap[mid];
		meshmap.erase(mid);
	}
	for (int i = id; i < colgeovec.size() - 1; ++i) {
		//shift down all (we assume there's not that much of them, so it's okay
		colgeovec[i] = colgeovec[i + 1];
		positionvec[i] = positionvec[i + 1];
		rotationvec[i] = rotationvec[i + 1];
		prevpositionvec[i] = prevpositionvec[i + 1];
		prevrotationvec[i] = prevrotationvec[i + 1];
		flagvec[i] = flagvec[i + 1];
	}
	resizeall(colgeovec.size() - 1);
	return true;
}

bool NvFlexHCollisionData::addSphere(const std::string &key) {
	// buffers must be mapped!
	if (hasKey(key))return false;
	int oldsize = colgeovec.size();
	collmap[key] = oldsize;
	hashmap[key] = -2;
	resizeall(oldsize + 1);
	int nid = colgeovec.size() - 1;
	flagvec[nid] = NvFlexMakeShapeFlags(eNvFlexShapeSphere, true);
	rotationvec[nid] = Quat();
	prevrotationvec[nid] = Quat();
	return true;
}

NvfSphereGeo NvFlexHCollisionData::getSphere(const std::string &key) {
	// buffers must be mapped!
	if (!hasKey(key))return NvfSphereGeo();
	int offset = collmap.at(key);
	return NvfSphereGeo((NvFlexSphereGeometry*)(colgeovec.mappedPtr + offset), positionvec.mappedPtr + offset, rotationvec.mappedPtr + offset, prevpositionvec.mappedPtr + offset, prevrotationvec.mappedPtr + offset);
}

bool NvFlexHCollisionData::addTriangleMesh(const std::string &key) {
	// buffers must be mapped!
	if (hasKey(key))return false;
	int oldsize = colgeovec.size();
	collmap[key] = oldsize;
	hashmap[key] = -2;
	resizeall(oldsize + 1);
	int nid = colgeovec.size() - 1;
	flagvec[nid] = NvFlexMakeShapeFlags(eNvFlexShapeTriangleMesh, true);
	colgeovec[nid].triMesh.scale[0] = 1.0f;
	colgeovec[nid].triMesh.scale[1] = 1.0f;
	colgeovec[nid].triMesh.scale[2] = 1.0f;
	rotationvec[nid] = Quat();
	prevrotationvec[nid] = Quat();
	positionvec[nid] = Vec4(0, 0, 0, 1);
	prevpositionvec[nid] = Vec4(0, 0, 0, 1);
	NvFlexHTriangleMesh* newmesh = new NvFlexHTriangleMesh(colgeovec.lib);
	NvFlexTriangleMeshId meshid = newmesh->getId();
	meshmap[meshid] = newmesh;
	colgeovec[nid].triMesh.mesh = meshid;
	return true;
}

NvfTrimeshGeo NvFlexHCollisionData::getTriangleMesh(const std::string &key) {
	// buffers must be mapped!
	if (!hasKey(key))return NvfTrimeshGeo();
	int offset = collmap.at(key);
	NvFlexTriangleMeshId mid = colgeovec[offset].triMesh.mesh;
	return NvfTrimeshGeo(meshmap.at(mid), positionvec.mappedPtr + offset, rotationvec.mappedPtr + offset, prevpositionvec.mappedPtr + offset, prevrotationvec.mappedPtr + offset);
}


int NvFlexHCollisionData::size() const {
	return colgeovec.size();
}

int64 NvFlexHCollisionData::getStoredHash(const std::string &key) {
	if (!hasKey(key))return -2;
	return hashmap.at(key);
}

bool NvFlexHCollisionData::setStoredHash(const std::string &key, const int64 hash) {
	if (!hasKey(key))return false;
	hashmap[key] = hash;
	return true;
}

void NvFlexHCollisionData::mapall() {
	colgeovec.map();
	positionvec.map();
	rotationvec.map();
	prevpositionvec.map();
	prevrotationvec.map();
	flagvec.map();
}
void NvFlexHCollisionData::unmapall() {
	colgeovec.unmap();
	positionvec.unmap();
	rotationvec.unmap();
	prevpositionvec.unmap();
	prevrotationvec.unmap();
	flagvec.unmap();
}

void NvFlexHCollisionData::setCollisionData(NvFlexSolver * solv) {
	NvFlexSetShapes(solv, colgeovec.buffer, positionvec.buffer, rotationvec.buffer, prevpositionvec.buffer, prevrotationvec.buffer, flagvec.buffer, flagvec.size());
}

void NvFlexHCollisionData::resizeall(int newsize) {
	colgeovec.resize(newsize);
	positionvec.resize(newsize);
	rotationvec.resize(newsize);
	prevpositionvec.resize(newsize);
	prevrotationvec.resize(newsize);
	flagvec.resize(newsize);
}

NvFlexHCollisionData::NvFlexHCollisionData(NvFlexLibrary *lib):colgeovec(lib), positionvec(lib), rotationvec(lib), prevpositionvec(lib), prevrotationvec(lib), flagvec(lib) {
	colgeovec.resize(0);
	positionvec.resize(0);
	rotationvec.resize(0);
	prevpositionvec.resize(0);
	prevrotationvec.resize(0);
	flagvec.resize(0);
	unmapall();
}


NvFlexHCollisionData::~NvFlexHCollisionData() {
	for (auto it = meshmap.begin(); it != meshmap.end(); ++it) {
		delete it->second;
	}

	colgeovec.destroy(); //dont need to destroy them - destructor does that!
	positionvec.destroy();
	rotationvec.destroy();
	prevpositionvec.destroy();
	prevrotationvec.destroy();
	flagvec.destroy();
}
