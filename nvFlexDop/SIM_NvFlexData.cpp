#include <PRM/PRM_Template.h>
#include <PRM/PRM_Default.h>
#include <NvFlexDevice.h>

#include "utils.h"

#include "SIM_NvFlexData.h"

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
	_lastGdpVId = -1;
	_lastGdpTId = -1;
	_lastGdpStrId = -1;

	

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
	// IMOIRTANT: if you add at least one more parameter to data - the below code must only be executed in case of ptsmaxcount parameter change
	
	int ptsmaxcount = getMaxPtsCount();
	if (_prevMaxPts == ptsmaxcount)return;

	try {
		acquireCudaContext();
		nvdata.reset(new NvFlexContainerWrapper(SIM_NvFlexData::nvFlexLibrary, ptsmaxcount, 0));
		releaseCudaContext();
		_indices.reset(new int[ptsmaxcount]);
	}
	catch (...) {
		messageLog(1, "nvflex data initialization failed!\n");
		_valid = false;
		nvdata.reset();
		_indices.reset();
		return;
	}
	_prevMaxPts = ptsmaxcount;
	messageLog(5, "nvflex data initialized with %d\n", ptsmaxcount);
}

void SIM_NvFlexData::makeEqualSubclass(const SIM_Data* source) {
	SIM_Data::makeEqualSubclass(source);
	//messageLog(6, "do makeEqual\n");
	const SIM_NvFlexData* src = SIM_DATA_CASTCONST(source, SIM_NvFlexData);
	if (src == NULL) {
		// some info?
		messageLog(6, "makeEqual src==Null\n");
		return;
	}
	nvdata = src->nvdata;
	_indices = src->_indices;
	_lastGdpPId = src->_lastGdpPId;
	_lastGdpVId = src->_lastGdpVId;
	_lastGdpTId = src->_lastGdpTId;
	_lastGdpStrId = src->_lastGdpStrId;
	_prevMaxPts = src->_prevMaxPts;
	_valid = _valid && src->_valid;
	if (!_valid) {
		messageLog(6, "makeEqual data was invalid\n");;
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
	const char * err = "NvF ERROR";
	const char * wrn = "NvF WARNING";
	const char * dbg = "NvF DEBUG";
	const char * all = "NvF ALL";
	const char * pre = all;
	short errlvl = 5;

	switch (type) {
	case eNvFlexLogError:
		pre = err; 
		errlvl = 1;
		break;
	case eNvFlexLogWarning:
		pre = wrn;
		errlvl = 2;
		break;
	case eNvFlexLogDebug:
		pre = dbg;
		errlvl = 3;
		break;
	case eNvFlexLogAll:
		pre = all;
		errlvl = 4;
		break;
	}
	messageLog(errlvl, "%s", pre);
	if (msg != NULL)messageLog(errlvl, ": %s", msg);
	if (file != NULL)messageLog(errlvl, " :: %s", file);
	messageLog(errlvl, " : line %d\n", line);

}

//cuda-aware deleter
void delete_NvFlexContainerWrapper(SIM_NvFlexData::NvFlexContainerWrapper *wrp) {
	acquireCudaContext();
	delete wrp;
	releaseCudaContext();
}


SIM_NvFlexData::SIM_NvFlexData(const SIM_DataFactory*fack):SIM_Data(fack),SIM_OptionsUser(this), _indices(nullptr, [](int*p){delete[] p;}), nvdata(nullptr, delete_NvFlexContainerWrapper), _lastGdpPId(-1), _lastGdpVId(-1), _lastGdpTId(-1), _lastGdpStrId(-1), _prevMaxPts(-1), _valid(false) {
	if (nvFlexLibrary != NULL)_valid = true;
	messageLog(5, "flex data constructed.\n");
}


SIM_NvFlexData::~SIM_NvFlexData() {
	messageLog(5, "flex data destructed.\n");
}


//wrapper
bool NvFlexHLibraryHolder::cudaContextCreated = false;
bool NvFlexHLibraryHolder::_deviceContextAvailable = true;
NvFlexLibrary* NvFlexHLibraryHolder::nvFlexLibrary = NULL;
GA_Size NvFlexHLibraryHolder::_instanceCount = 0;



NvFlexHLibraryHolder::NvFlexHLibraryHolder() {
	++_instanceCount;
	if (nvFlexLibrary == NULL) {
		if (!cudaContextCreated && _deviceContextAvailable) {
			try {
				int cdevice = NvFlexDeviceGetSuggestedOrdinal();
				if (cdevice == -1) {
					messageLog(1, "FlexDevice: No Cuda device found ! \n");
					throw std::runtime_error("Failed to initialize Cuda Context");
				}
				if (!NvFlexDeviceCreateCudaContext(cdevice)) {
					messageLog(1, "FlexDevice: Failed to initialize Cuda Context\n");
					throw std::runtime_error("Failed to initialize Cuda Context");
				}
				cudaContextCreated = true;
				_deviceContextAvailable = true;
			}
			catch (...) {
				messageLog(1, "FlexDevice: falling back to default method...\n");
				_deviceContextAvailable = false;
			}
		}
		NvFlexInitDesc desc;
		desc.deviceIndex = 0; // ignored, device index is set by the context. TODO: make an env variable for fallback device
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
			messageLog(0, "Failed to initialize Flex library\n");
			throw std::runtime_error("Failed to initialize Flex library");
		}

		messageLog(5, "flex library initialized\n");
	}
	messageLog(5, "libhld: instancecount: %d\n",  _instanceCount);
}
NvFlexHLibraryHolder::~NvFlexHLibraryHolder() {
	--_instanceCount;
	messageLog(5, "libhld: instancecount: %d\n", _instanceCount);
	if (_instanceCount == 0) {
		//no, lets assume that we need to have proper context by this time for it to be properly released
		// and lets assume that NvFlexShutdown restores previous context
		while (releaseCudaContext()) {};//release all cuda contexts
		NvFlexAcquireContext(nvFlexLibrary);
		NvFlexShutdown(nvFlexLibrary);
		nvFlexLibrary = NULL;
		cudaContextAcquiredCount = 0;
		if (_deviceContextAvailable) NvFlexDeviceDestroyCudaContext();
		cudaContextCreated = false;
		messageLog(5, "flex library destroyed\n");
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
