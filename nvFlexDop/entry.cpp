#include <UT/UT_DSOVersion.h>

#include "SIM_NvFlexData.h"
#include "SIM_NvFlexSolver.h"
#include <NvFlexDevice.h>
#include <stdlib.h>
#include <climits>
#include "utils.h"


void initializeSIM(void*) {
	//init logging:
	const char* envar = std::getenv("NVFLEX_VERBOSITY_LEVEL");
	if (envar != NULL) {
		char** end;
		long lerrlvl = strtol(envar, end, 10);
		short errlvl = 0;
		if (lerrlvl > SHRT_MAX)errlvl = SHRT_MAX;
		else if (lerrlvl < 0)errlvl = 0;
		else errlvl = (short)lerrlvl;
		setMessageLogLevel(errlvl);
	}
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