#include "SIM_NvFlexData.h"
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Default.h>

NvFlexLibrary* SIM_NvFlexData::nvFlexLibrary = NULL;

static void CreateFluidParticleGrid(NvFlexExtParticleData& ptd, int* indices, Vec3 lower, int dimx, int dimy, int dimz, float radius, Vec3 velocity, float invMass, int phase, float jitter = 0.005f);

static int ONCE = 0;

void SIM_NvFlexData::initializeSubclass() {
	SIM_Data::initializeSubclass();
	_lastGdpPId = -1;

	int ptsmaxcount = getMaxPtsCount();
	nvdata.reset(new NvFlexContainerWrapper(SIM_NvFlexData::nvFlexLibrary, ptsmaxcount, 0));
	_indices.reset(new int[ptsmaxcount]);

	std::cout << "nvflex data initialized" << std::endl;

	//debug test
	/*float sizex = 1.76f;
	float sizey = 3.20f;
	float sizez = 3.50f;

	float restDistance = 0.055f;

	int x = int(sizex / restDistance);
	int y = int(sizey / restDistance);
	int z = int(sizez / restDistance);

	NvFlexExtParticleData ptd = NvFlexExtMapParticleData(nvdata->container());

	NvFlexExtAllocParticles(nvdata->container(), x*y*z, _indices.get());
	CreateFluidParticleGrid(ptd, _indices.get(), Vec3(0, restDistance, 0), x, y, z, restDistance, Vec3(0.0f), 1, eNvFlexPhaseSelfCollide | eNvFlexPhaseFluid);
	*/

	//NvFlexExtUnmapParticleData(nvdata->container());
}

void SIM_NvFlexData::makeEqualSubclass(const SIM_Data* source) {
	SIM_Data::makeEqualSubclass(source);

	const SIM_NvFlexData* src = SIM_DATA_CASTCONST(source, SIM_NvFlexData);
	if (src == NULL) {
		// some info?
		return;
	}
	nvdata = src->nvdata;
	_indices = src->_indices;
	_lastGdpPId = src->_lastGdpPId;
}

const SIM_DopDescription* SIM_NvFlexData::getDescriptionForFucktory() {
	static PRM_Name maxpts_name("maxpts", "Maximum Particles Count");

	static PRM_Default maxpts_default(1000000);

	static PRM_Template prms[]{
		PRM_Template(PRM_INT_E, 1, &maxpts_name, &maxpts_default),
		PRM_Template()
	};

	static SIM_DopDescription desc(true, "nvflexData", "NvFlex Data", "NvFlexData", classname(), prms);
	return &desc;
}

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


SIM_NvFlexData::SIM_NvFlexData(const SIM_DataFactory*fack):SIM_Data(fack),SIM_OptionsUser(this), _indices(nullptr_t(), std::default_delete<int[]>()), _lastGdpPId(-1){
	if (nvFlexLibrary == NULL) {
		nvFlexLibrary = NvFlexInit(110, &nvFlexErrorCallbackPrint);
	}
}


SIM_NvFlexData::~SIM_NvFlexData(){}



// debug helpers
static void CreateFluidParticleGrid(NvFlexExtParticleData& ptd, int* indices,  Vec3 lower, int dimx, int dimy, int dimz, float radius, Vec3 velocity, float invMass, int phase, float jitter)
{
	for (int x = 0; x < dimx; ++x)
	{
		for (int y = 0; y < dimy; ++y)
		{
			for (int z = 0; z < dimz; ++z)
			{
				
				Vec3 position = lower + Vec3(float(x), float(y), float(z))*radius + RandomUnitVector()*jitter;
				int ind = indices[z + dimz*(y + dimy*x)];
				ptd.particles[ind * 4 + 0] = position.x;
				ptd.particles[ind * 4 + 1] = position.y;
				ptd.particles[ind * 4 + 2] = position.z;
				ptd.particles[ind * 4 + 3] = invMass;

				ptd.velocities[ind * 3 + 0] = velocity.x;
				ptd.velocities[ind * 3 + 1] = velocity.y;
				ptd.velocities[ind * 3 + 2] = velocity.z;
				
				ptd.phases[ind] = phase;
			}
		}
	}
}