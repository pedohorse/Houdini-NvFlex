#pragma once
#include <SIM/SIM_Solver.h>
#include <SIM/SIM_DataUtils.h>
#include <SIM/SIM_DopDescription.h>

#include <NvFlex.h>
#include <NvFlexExt.h>

class SIM_NvFlexSolver:public SIM_Solver,public SIM_OptionsUser
{
public:
	

	SIM_Result solveObjectsSubclass(SIM_Engine& engine, SIM_ObjectArray& objs, SIM_ObjectArray& newobjs, SIM_ObjectArray& feedbackobjs, const SIM_Time& timestep);

	GET_DATA_FUNC_F("radius", Radius);
	GETSET_DATA_FUNCS_I("iterations", Iterations);
	GETSET_DATA_FUNCS_I("substeps", Substeps);
	GETSET_DATA_FUNCS_F("maxSpeed", MaxSpeed);
	GETSET_DATA_FUNCS_F("maxAcceleration", MaxAcceleration);

	GETSET_DATA_FUNCS_F("fluidRestDistanceMult", FluidRestDistanceMult);

	GETSET_DATA_FUNCS_I("planesCount", PlanesCount);

	GETSET_DATA_FUNCS_F("adhesion", Adhesion);
	GETSET_DATA_FUNCS_F("cohesion", Cohesion);
	GETSET_DATA_FUNCS_F("surfaceTension", SurfaceTension);
	GETSET_DATA_FUNCS_F("viscosity", Viscosity);
	GETSET_DATA_FUNCS_F("relaxationFactor", RelaxationFactor);
	GETSET_DATA_FUNCS_F("solidPressure", SolidPressure);
	GETSET_DATA_FUNCS_F("vorticityConfinement", VorticityConfinement);
	GETSET_DATA_FUNCS_F("buoyancy", Buoyancy);

	GETSET_DATA_FUNCS_F("dynamicfriction", DynamicFriction);
	GETSET_DATA_FUNCS_F("staticfriction", StaticFriction);
	GETSET_DATA_FUNCS_F("particleFriction", ParticleFriction);
	GETSET_DATA_FUNCS_F("drag", Drag);
	GETSET_DATA_FUNCS_F("lift", Lift);

	GETSET_DATA_FUNCS_F("shapeCollisionMargin", ShapeCollisionMargin);
	GETSET_DATA_FUNCS_F("particleCollisionMargin", ParticleCollisionMargin);
	GETSET_DATA_FUNCS_F("collisionDistance", CollisionDistance);

	GETSET_DATA_FUNCS_V3("wind", Wind);

protected:
	explicit SIM_NvFlexSolver(const SIM_DataFactory*fack);
	virtual ~SIM_NvFlexSolver();

	void initializeSubclass();
	void updateSolverParams();
	void makeEqualSubclass(const SIM_Data* source);


	NvFlexParams nvparams;


private:
	static const SIM_DopDescription* getDescriptionForFucktory();

	DECLARE_STANDARD_GETCASTTOTYPE();
	DECLARE_DATAFACTORY(SIM_NvFlexSolver, SIM_Solver, "solver for nvflex sim", getDescriptionForFucktory());
};

class NvFlexHContextAutoGetter {
public:
	explicit NvFlexHContextAutoGetter(NvFlexLibrary*lib):_lib(lib) { NvFlexAcquireContext(lib); }
	~NvFlexHContextAutoGetter() { NvFlexRestoreContext(_lib); }
private:
	NvFlexLibrary* _lib;
};