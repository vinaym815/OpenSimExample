// Using it primarily for debugging purposes


#include <OpenSim/OpenSim.h>
#include<iostream>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

const double initialTime = 0.0;
const double finalTime = 0.25;
const double desiredAccuracy = 1.0e-5;

int main()
{
    Model osimModel("Arm26_Optimize.osim");
	osimModel.setUseVisualizer(true);
    
    /////////////////////////////////////
    /////// Adding new controller //////
    //////////////////////////////////// 

    const int segs = 2;
    OpenSim::Set<Actuator> actuators = osimModel.getActuators();
    const int numActuators = actuators.getSize(); 
    const int numVars = 2*numActuators*segs;

    double xT[segs] = {0.1, 0.14};
    double yVal[segs] = {0.3, 0.7};

    // Array of activations targets
    SimTK::Vector vecVars(numVars);

    for(int i=0; i<numActuators; i++){
        for(int j=0; j<segs; j++){
            vecVars[2*i*segs+j] = xT[j];
            vecVars[(2*i+1)*segs+j] = yVal[j];
        }
    }

    PrescribedController *muscleController = new PrescribedController();
    muscleController->setActuators(osimModel.updActuators());
    muscleController->setName("MuscleController");

    for(int i=0; i<numActuators; i++){
        muscleController->prescribeControlForActuator( actuators[i].getName(), 
                            new PiecewiseConstantFunction(segs, &vecVars[2*i*segs], 
                            &vecVars[(2*i+1)*segs], "ActivationSignal"));
    }

    osimModel.addController(muscleController);
    osimModel.finalizeConnections();

    //OpenSim::PrescribedController *handleController = dynamic_cast<PrescribedController*>(&osimModel.updControllerSet()[0]);
    //FunctionSet &funcSet = handleController->upd_ControlFunctions();
    //for(int i=0; i<numActuators; i++){
    //    double time = 0;
    //    PiecewiseConstantFunction *func = dynamic_cast<PiecewiseConstantFunction*>(&funcSet[i]);
    //    for(int j=0; j<segs ; j++){
    //        time += vecVars[2*i*segs+j];
    //        func->setX(j, time);
    //        func->setY(j, vecVars[(2*i+1)*segs+j]);
    //    }
    //}

    /////////////////////////////////////////
    ///// Setting up Initial State  /////////
    /////////////////////////////////////////

    // Initialize the system and get the state representing the state system
    State& si = osimModel.initSystem();

    // Initialize the starting shoulder angle.
    const CoordinateSet& coords = osimModel.getCoordinateSet();
    coords.get("r_shoulder_elev").setValue(si, -1.57079633);

    // Set the initial muscle activations and make all tendons rigid.
    const Set<Muscle> &muscleSet = osimModel.getMuscles();
    for(int i=0; i<muscleSet.getSize(); ++i) {
        muscleSet[i].setActivation(si, 0.01);
        muscleSet[i].setIgnoreTendonCompliance(si, true);
    }
    // Make sure the muscles states are in equilibrium
    osimModel.equilibrateMuscles(si);


    ///////////////////////////////////////////
    ///// Runnning Simulation ////////////////
    ///////////////////////////////////////////

    Manager manager(osimModel);
    manager.setIntegratorAccuracy(desiredAccuracy);

    // Integrate from initial time to final time.
    si.setTime(initialTime);
    osimModel.getMultibodySystem().realize(si, Stage::Acceleration);
    
    manager.initialize(si);
    si = manager.integrate(finalTime);
    auto statesTable = manager.getStatesTable();
    STOFileAdapter_<double>::write(statesTable, "testing.sto");

    std::cout << "Everything Ran Fine" << std::endl;
}