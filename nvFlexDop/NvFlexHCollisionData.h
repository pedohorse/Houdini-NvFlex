#pragma once
#include <sys/SYS_Types.h>
#include <NvFlex.h>
#include <NvFlexExt.h>
#include <../core/maths.h>

#include <string>
#include <unordered_map>

#include "NvFlexHTriangleMesh.h"



template <class T>
class NvFlexHCollisionGeometryWrapper {
public:
	T* collgeo;
	Vec4* position;
	Quat* rotation;
	Vec4* prevposition;
	Quat* prevrotation;

	NvFlexHCollisionGeometryWrapper(T* cg, Vec4* p, Quat* r, Vec4* pp, Quat* pr) {
		collgeo = cg;
		position = p;
		rotation = r;
		prevposition = pp;
		prevrotation = pr;
	}

	NvFlexHCollisionGeometryWrapper() :collgeo(NULL), position(NULL), rotation(NULL), prevposition(NULL), prevrotation(NULL) {}
};

typedef NvFlexHCollisionGeometryWrapper<NvFlexSphereGeometry> NvfSphereGeo;
typedef NvFlexHCollisionGeometryWrapper<NvFlexHTriangleMesh> NvfTrimeshGeo;

class NvFlexHCollisionData {
public:
	NvFlexHCollisionData(NvFlexLibrary*lib);
	NvFlexHCollisionData(const NvFlexHCollisionData&) = delete;
	NvFlexHCollisionData& operator=(const NvFlexHCollisionData&) = delete;
	~NvFlexHCollisionData();


	bool hasKey(const std::string &key) const;
	int64 getStoredHash(const std::string &key);
	bool setStoredHash(const std::string &key, const int64 hash);
	//add-remove shit
	bool removeItem(const std::string &key);

	bool addSphere(const std::string &key);
	NvfSphereGeo getSphere(const std::string &key) const;

	bool addTriangleMesh(const std::string &key);
	NvfTrimeshGeo getTriangleMesh(const std::string &key) const;
	//
	int size() const;

	void mapall();
	void unmapall();

	void setCollisionData(NvFlexSolver* solv);

private:
	std::unordered_map<std::string, int> collmap; //offset into colgeovec
	std::unordered_map<NvFlexTriangleMeshId, NvFlexHTriangleMesh*> meshmap;
	std::unordered_map<std::string, int64> hashmap;

	void resizeall(int newsize);

private:
	NvFlexVector<NvFlexCollisionGeometry> colgeovec;
	NvFlexVector<Vec4> positionvec;
	NvFlexVector<Quat> rotationvec;
	NvFlexVector<Vec4> prevpositionvec;
	NvFlexVector<Quat> prevrotationvec;
	NvFlexVector<int>  flagvec;

};
