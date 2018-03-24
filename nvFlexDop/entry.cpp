#include <UT/UT_DSOVersion.h>

#include "SIM_NvFlexData.h"
#include "SIM_NvFlexSolver.h"


void initializeSIM(void*) {
	IMPLEMENT_DATAFACTORY(SIM_NvFlexData);
	IMPLEMENT_DATAFACTORY(SIM_NvFlexSolver);
}