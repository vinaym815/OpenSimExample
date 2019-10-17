#include <OpenSim/OpenSim.h>
#include<iostream>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

const double initialTime = 0.0;
const double finalTime = 0.25;
const double desired_accuracy = 1.0e-5;

int main()
{
    Model osimModel("Arm26_Optimize.osim");
	osimModel.setUseVisualizer(true);
    
    /////////////////////////////////////
    /////// Adding new controller //////
    //////////////////////////////////// 

    PrescribedController *muscleController = new PrescribedController();
    muscleController->setActuators(osimModel.updActuators());
    muscleController->setName("MuscleController");
    OpenSim::Set<Actuator> actuators = osimModel.getActuators();
    const int numActuators = actuators.getSize(); 

    for(int i=0; i<numActuators; i++){
        muscleController->prescribeControlForActuator( actuators[i].getName(), new PiecewiseConstantFunction());
    }

    osimModel.addController(muscleController);
    osimModel.finalizeConnections();

    //////////////////////////////////////////
    //// Setting up Controller Parameters ///
    /////////////////////////////////////////

    const int segs = 2;
    SimTK::Matrix coeffs(numActuators, segs);

    double xT[segs] = {0.1, 0.2};
    double yVal[segs] = {0.3, 0.7};

    for(int i=0; i<numActuators; i++){
        for(int j=0; j<segs; j++){
            coeffs[i][j] = yVal[j];
        }
    }

    FunctionSet &funcSet = muscleController->upd_ControlFunctions();
    for(int i=0; i<numActuators; i++){
        PiecewiseConstantFunction *func = dynamic_cast<PiecewiseConstantFunction*>(&funcSet[i]);
        std::cout << i << std::endl;
        for(int j=0; j<segs ; j++){
            func->addPoint(xT[j], coeffs[i][j]);
        }
    }

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
    manager.setIntegratorAccuracy(desired_accuracy);

    // Integrate from initial time to final time.
    si.setTime(initialTime);
    osimModel.getMultibodySystem().realize(si, Stage::Acceleration);
    
    manager.initialize(si);
    si = manager.integrate(finalTime);
    auto statesTable = manager.getStatesTable();
    STOFileAdapter_<double>::write(statesTable, "Arm26_optimized_states.sto");

    std::cout << "Everything Ran Fine" << std::endl;
}
