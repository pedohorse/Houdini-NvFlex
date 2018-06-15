#include "SIM_NvFlexData.h"
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Default.h>

#include <NvFlexDevice.h>


static void log(unsigned short level, const char* fmt, ...) {
	if (level > 999)return;
	va_list args;
	va_start(args, fmt);
	std::vfprintf(stderr, fmt, args);
	std::fflush(stderr);
	va_end(args);
}


static void CreateFluidParticleGrid(NvFlexExtParticleData& ptd, int* indices, Vec3 lower, int dimx, int dimy, int dimz, float radius, Vec3 velocity, float invMass, int phase, float jitter = 0.005f);

static uint cudaContextAcquiredCount = 0;

bool acquireCudaContext() {
	if (SIM_NvFlexData::nvFlexLibrary == NULL)return false;
	NvFlexAcquireContext(SIM_NvFlexData::nvFlexLibrary);
	++cudaContextAcquiredCount;
	return true;
}

bool releaseCudaContext() {
	if (SIM_NvFlexData::nvFlexLibrary==NULL || cudaContextAcquiredCount==0)return false;
	NvFlexRestoreContext(SIM_NvFlexData::nvFlexLibrary);
	--cudaContextAcquiredCount;
	return true;
}

void SIM_NvFlexData::initializeSubclass() {
	SIM_Data::initializeSubclass();
	_lastGdpPId = -1;
	_lastGdpTId = -1;

	

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

void SIM_NvFlexData::setParametersSubclass(const SIM_Options & parms) {
	// we dont care what option was set for now, we have only one
	SIM_Data::setParametersSubclass(parms);
	
	int ptsmaxcount = getMaxPtsCount();
	if (_prevMaxPts == ptsmaxcount)return;

	try {
		acquireCudaContext();
		nvdata.reset(new NvFlexContainerWrapper(SIM_NvFlexData::nvFlexLibrary, ptsmaxcount, 0));
		releaseCudaContext();
		_indices.reset(new int[ptsmaxcount]);
	}
	catch (...) {
		log(1, "nvflex data initialization failed!\n");
		_valid = false;
		nvdata.reset();
		_indices.reset();
		return;
	}
	_prevMaxPts = ptsmaxcount;
	log(5, "nvflex data initialized with %d\n", ptsmaxcount);
}

void SIM_NvFlexData::makeEqualSubclass(const SIM_Data* source) {
	SIM_Data::makeEqualSubclass(source);
	log(6, "do makeEqual");
	const SIM_NvFlexData* src = SIM_DATA_CASTCONST(source, SIM_NvFlexData);
	if (src == NULL) {
		// some info?
		log(6, "makeEqual src==Null\n");
		return;
	}
	nvdata = src->nvdata;
	_indices = src->_indices;
	_lastGdpPId = src->_lastGdpPId;
	_lastGdpTId = src->_lastGdpTId;
	_prevMaxPts = src->_prevMaxPts;
	_valid = _valid && src->_valid;
	if (!_valid) {
		log(6, "makeEqual data was invalid\n");;
		nvdata.reset();
		_indices.reset();
	}
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

//cuda-aware deleter
void delete_NvFlexContainerWrapper(SIM_NvFlexData::NvFlexContainerWrapper *wrp) {
	acquireCudaContext();
	delete wrp;
	releaseCudaContext();
}


SIM_NvFlexData::SIM_NvFlexData(const SIM_DataFactory*fack):SIM_Data(fack),SIM_OptionsUser(this), _indices(nullptr_t(), std::default_delete<int[]>()), nvdata(nullptr_t(), delete_NvFlexContainerWrapper), _lastGdpPId(-1), _lastGdpTId(-1), _lastGdpStrId(-1), _prevMaxPts(-1), _valid(false){
	if (nvFlexLibrary != NULL)_valid = true;
	log(5, "flex data constructed.\n");
}


SIM_NvFlexData::~SIM_NvFlexData(){
	log(5, "flex data destructed.\n");
}


//wrapper
bool NvFlexHLibraryHolder::cudaContextCreated = false;
NvFlexLibrary* NvFlexHLibraryHolder::nvFlexLibrary = NULL;
GA_Size NvFlexHLibraryHolder::_instanceCount = 0;



NvFlexHLibraryHolder::NvFlexHLibraryHolder() {
	++_instanceCount;
	if (nvFlexLibrary == NULL) {
		if (!cudaContextCreated) {
			if (!NvFlexDeviceCreateCudaContext(NvFlexDeviceGetSuggestedOrdinal())) {
				log(0, "Failed to initialize Cuda Context\n");
				throw std::runtime_error("Failed to initialize Cuda Context");
			}
			cudaContextCreated = true;
		}
		NvFlexInitDesc desc;
		desc.deviceIndex = 0; // ignored, device index is set by the context
		desc.enableExtensions = false;
		desc.renderDevice = NULL;
		desc.renderContext = NULL;
		desc.computeType = NvFlexComputeType::eNvFlexCUDA;
		bool flexFailed = false;
		try {
			nvFlexLibrary = NvFlexInit(110, &nvFlexErrorCallbackPrint, &desc);
		}
		catch (...) { flexFailed = true; }
		if (nvFlexLibrary == NULL)flexFailed = true;

		if (flexFailed) {
			log(0, "Failed to initialize Flex library\n");
			throw std::runtime_error("Failed to initialize Flex library");
		}

		log(5, "flex library initialized\n");
	}
	log(5, "libhld: instancecount: %d\n",  _instanceCount);
}
NvFlexHLibraryHolder::~NvFlexHLibraryHolder() {
	--_instanceCount;
	log(5, "libhld: instancecount: %d\n", _instanceCount);
	if (_instanceCount == 0) {
		//no, lets assume that we need to have proper context by this time for it to be properly released
		// and lets assume that NvFlexShutdown restores previous context
		while (releaseCudaContext()) {};//release all cuda contexts
		NvFlexAcquireContext(nvFlexLibrary);
		NvFlexShutdown(nvFlexLibrary);
		nvFlexLibrary = NULL;
		cudaContextAcquiredCount = 0;
		NvFlexDeviceDestroyCudaContext();
		cudaContextCreated = false;
		log(5, "flex library destroyed\n");
	}
}



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