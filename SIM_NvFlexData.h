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
		explicit NvFlexContainerWrapper(NvFlexLibrary*lib, int maxParticles, int MaxDiffuseParticles, int maxNeighbours = 96) {
			slv = NvFlexCreateSolver(lib, maxParticles, MaxDiffuseParticles, maxNeighbours);
			cont = NvFlexExtCreateContainer(lib, slv, maxParticles);
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
	private:
		NvFlexHCollisionData* colld;
		NvFlexSolver* slv;
		NvFlexExtContainer* cont;
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

