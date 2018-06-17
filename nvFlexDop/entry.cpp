#include <UT/UT_DSOVersion.h>

#include "SIM_NvFlexData.h"
#include "SIM_NvFlexSolver.h"
#include <NvFlexDevice.h>
#include "utils.h"


void initializeSIM(void*) {
	try { //some useless error handling
		if (NvFlexDeviceGetSuggestedOrdinal() == -1)throw std::runtime_error("CUDA device not found!");
		IMPLEMENT_DATAFACTORY(SIM_NvFlexData);
		IMPLEMENT_DATAFACTORY(SIM_NvFlexSolver);
	}
	catch (std::runtime_error &e) {
		messageLog(0, "OMEGA ERROR: %s !\nnvFlex is not loaded!\n",e.what());
	}
	catch (...) {
		messageLog(0, "UNKNOWN OMEGA ERROR ! nvFlex is not loaded!\n");
	}
}