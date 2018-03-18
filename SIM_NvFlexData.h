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
			int* springIds;
			float* springRls;
			float* springSts;

			NvFlexHSpringData(int*sid, float*srl, float*sts):springIds(sid),springRls(srl),springSts(sts){}
		} NvFlexHSpringData;

		explicit NvFlexContainerWrapper(NvFlexLibrary*lib, int maxParticles, int MaxDiffuseParticles, int maxNeighbours = 96):springIndices(lib),springRestLengths(lib),springStrenghts(lib) {
			slv = NvFlexCreateSolver(lib, maxParticles, MaxDiffuseParticles, maxNeighbours);
			if (slv == NULL)throw std::runtime_error("NULL NVFLEX SOLVER!");
			cont = NvFlexExtCreateContainer(lib, slv, maxParticles);
			if (cont == NULL)throw std::runtime_error("NULL NVFLEX CONTAINER!");
			colld = new NvFlexHCollisionData(lib);
		}
		NvFlexContainerWrapper(NvFlexContainerWrapper&) = delete;
		~NvFlexContainerWrapper() {
			NvFlexExtDestroyContainer(cont);
			NvFlexDestroySolver(slv);
			delete colld;
		}

		NvFlexSolver* solver() { return slv; }
		NvFlexExtContainer * container() { return cont; }
		NvFlexHCollisionData* collisionData() { return colld; }

		//springs
		int getSpringDataSize()const { return springRestLengths.size(); }
		void resizeSpringData(int newSize) {
			/// be sure data is NOT MAPPED before here
			/// cuz all previous pointers will be invalidated
			springIndices.resize(newSize);
			springRestLengths.resize(newSize);
			springStrenghts.resize(newSize);
			springIndices.unmap();
			springRestLengths.unmap();
			springStrenghts.unmap();
		}
		NvFlexHSpringData mapSpringData(){
			springIndices.map();
			springRestLengths.map();
			springStrenghts.map();
			return NvFlexHSpringData(springIndices.mappedPtr, springRestLengths.mappedPtr, springStrenghts.mappedPtr);
		}
		void unmapSpringData() {
			springIndices.unmap();
			springRestLengths.unmap();
			springStrenghts.unmap();
		}
		void pushSpringsToDevice() {
			NvFlexSetSprings(slv, springIndices.buffer, springRestLengths.buffer, springStrenghts.buffer, springRestLengths.size());
		}
	private:
		NvFlexHCollisionData* colld;
		NvFlexSolver* slv;
		NvFlexExtContainer* cont;

		//springs
		NvFlexVector<int> springIndices;
		NvFlexVector<float> springRestLengths;
		NvFlexVector<float> springStrenghts;
	};

	
	static NvFlexLibrary* nvFlexLibrary;

	std::shared_ptr<NvFlexContainerWrapper> nvdata;

	GETSET_DATA_FUNCS_I("maxpts", MaxPtsCount);

protected:
	explicit SIM_NvFlexData(const SIM_DataFactory*fack);
	virtual ~SIM_NvFlexData();

	void initializeSubclass();
	void makeEqualSubclass(const SIM_Data* source);

private: //for a friend
	std::shared_ptr<int> _indices;
	int64 _lastGdpPId;

private:
	static const SIM_DopDescription* getDescriptionForFucktory();

	DECLARE_STANDARD_GETCASTTOTYPE()
	DECLARE_DATAFACTORY(SIM_NvFlexData, SIM_Data, "data for nvFlex sim", getDescriptionForFucktory());

};

