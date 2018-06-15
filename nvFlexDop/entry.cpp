#include <UT/UT_DSOVersion.h>

#include "SIM_NvFlexData.h"
#include "SIM_NvFlexSolver.h"


void initializeSIM(void*) {
	try { //some useless error handling
		IMPLEMENT_DATAFACTORY(SIM_NvFlexData);
		IMPLEMENT_DATAFACTORY(SIM_NvFlexSolver);
	}
	catch (std::runtime_error &e) {
		std::cout << "OMEGA ERROR: "<< e.what() <<" !\nnvFlex is not loaded!\n" << std::endl;
	}
	catch (...) {
		std::cout << "UNKNOWN OMEGA ERROR ! nvFlex is not loaded!\n" << std::endl;
	}
}