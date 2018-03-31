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


class SIM_NvFlexSolver; //fwd decl

class SIM_NvFlexData:public SIM_Data, public SIM_OptionsUser
{
	friend class SIM_NvFlexSolver;
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

		explicit NvFlexContainerWrapper(NvFlexLibrary*lib, int maxParticles, int MaxDiffuseParticles, int maxNeighbours = 96):_springIndices(lib),_springRestLengths(lib),_springStrenghts(lib), _triangleIndices(lib),_triangleNormals(lib){
			_slv = NvFlexCreateSolver(lib, maxParticles, MaxDiffuseParticles, maxNeighbours);
			if (_slv == NULL)throw std::runtime_error("NULL NVFLEX SOLVER!");
			_cont = NvFlexExtCreateContainer(lib, _slv, maxParticles);
			if (_cont == NULL)throw std::runtime_error("NULL NVFLEX CONTAINER!");
			_colld = new NvFlexHCollisionData(lib);
		}
		NvFlexContainerWrapper(NvFlexContainerWrapper&) = delete;
		~NvFlexContainerWrapper() {
			NvFlexExtDestroyContainer(_cont);
			NvFlexDestroySolver(_slv);
			delete _colld;
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
	};

	
	static NvFlexLibrary* nvFlexLibrary;

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
	int64 _lastGdpPId;

private:
	static const SIM_DopDescription* getDescriptionForFucktory();

	DECLARE_STANDARD_GETCASTTOTYPE()
	DECLARE_DATAFACTORY(SIM_NvFlexData, SIM_Data, "data for nvFlex sim", getDescriptionForFucktory());

};

