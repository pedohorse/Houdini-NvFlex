#pragma once
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

typedef long long int64;

class NvFlexHCollisionData
{
public:
	NvFlexHCollisionData(NvFlexLibrary*lib);
	NvFlexHCollisionData(const NvFlexHCollisionData&) = delete;
	NvFlexHCollisionData& operator=(const NvFlexHCollisionData&) = delete;
	~NvFlexHCollisionData();


	bool hasKey(std::string key);
	int64 getStoredHash(std::string key);
	bool setStoredHash(std::string key, int64 hash);
	//add-remove shit
	bool removeItem(std::string key);

	bool addSphere(std::string key);
	NvfSphereGeo getSphere(std::string key);

	bool addTriangleMesh(std::string key);
	NvfTrimeshGeo getTriangleMesh(std::string key);
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
