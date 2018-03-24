#pragma once
#include <NvFlex.h>
#include <NvFlexExt.h>
#include <../core/maths.h>


class NvFlexHTriangleMesh
{
public:
	NvFlexHTriangleMesh(NvFlexLibrary* lib);
	NvFlexHTriangleMesh(const NvFlexHTriangleMesh&) = delete;
	NvFlexHTriangleMesh& operator=(const NvFlexHTriangleMesh&) = delete;
	~NvFlexHTriangleMesh();

	NvFlexTriangleMeshId getId()const;
	void loadData(const Vec3* verts, const int* tris, int vertcount, int triscount);
	
	void mapall();
	void unmapall();

	void updateNvBuffers();

private:
	friend class NvFlexHTriangleMeshAutoMapper;

	NvFlexTriangleMeshId id;
	NvFlexVector<Vec3> vertvec;
	NvFlexVector<int> trivec;
	float lower[3];
	float upper[3];
};


class NvFlexHTriangleMeshAutoMapper {
public:
	NvFlexHTriangleMeshAutoMapper(NvFlexHTriangleMesh* m) :mesh(*m) { mesh.mapall(); }
	NvFlexHTriangleMeshAutoMapper(NvFlexHTriangleMesh& m) :mesh(m) { mesh.mapall(); }
	NvFlexHTriangleMeshAutoMapper(const NvFlexHTriangleMeshAutoMapper&) = delete;
	NvFlexHTriangleMeshAutoMapper& operator=(const NvFlexHTriangleMeshAutoMapper&) = delete;
	~NvFlexHTriangleMeshAutoMapper() { mesh.unmapall(); mesh.updateNvBuffers(); }

	inline void setVertexCount(int count) { mesh.vertvec.resize(count); }
	inline void setTrianglesCount(int count) { mesh.trivec.resize(count * 3); }
	inline Vec3* vertices()const { return mesh.vertvec.mappedPtr; }
	inline int* triangles()const { return mesh.trivec.mappedPtr; }
	inline float* lower()const { return mesh.lower; }
	inline float* upper()const { return mesh.upper; }


private:
	NvFlexHTriangleMesh& mesh;
};
