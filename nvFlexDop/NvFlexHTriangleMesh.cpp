#include "NvFlexHTriangleMesh.h"



NvFlexHTriangleMesh::NvFlexHTriangleMesh(NvFlexLibrary* lib):vertvec(lib),trivec(lib)
{
	id = NvFlexCreateTriangleMesh(lib);
	vertvec.resize(0);
	trivec.resize(0);
	vertvec.unmap();
	trivec.unmap();
}

NvFlexHTriangleMesh::~NvFlexHTriangleMesh()
{
	NvFlexDestroyTriangleMesh(vertvec.lib, id);
	vertvec.destroy(); //actually, NvFlexVector's destructor does that for us, so wtf? u'r lucky he checks if shit has already been destroyed and does nothing second time
	trivec.destroy(); 
}

NvFlexTriangleMeshId NvFlexHTriangleMesh::getId() const{
	return id;
}

void NvFlexHTriangleMesh::loadData(const Vec3* verts, const int* tris, int vertcount, int triscount) {
	mapall();

	vertvec.resize(vertcount);
	trivec.resize(triscount * 3);
	memcpy(vertvec.mappedPtr, verts, vertcount * sizeof(Vec3));
	memcpy(trivec.mappedPtr, tris, triscount * 3 * sizeof(int));

	unmapall();
}

void NvFlexHTriangleMesh::mapall() {
	vertvec.map();
	trivec.map();
}

void NvFlexHTriangleMesh::unmapall() {
	trivec.unmap();
	vertvec.unmap();
}

void NvFlexHTriangleMesh::updateNvBuffers() {
	NvFlexUpdateTriangleMesh(vertvec.lib, id, vertvec.buffer, trivec.buffer, vertvec.size(), trivec.size() / 3, lower, upper);
}