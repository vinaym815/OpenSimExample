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

    const int segs = 2;
    OpenSim::Set<Actuator> actuators = osimModel.getActuators();
    const int numActuators = actuators.getSize(); 
    const int numCoeffs = numActuators*segs;

    SimTK::Vector coeffs(2*numCoeffs);
    double xT[segs] = {0.1, 0.2};
    double yVal[segs] = {0.3, 0.7};

    // Matrix that contains the activation
    double **activMatrix = new double*[(size_t)numActuators];
    for(int i=0; i<numActuators; i++){
        activMatrix[i] = new double[segs];
    }

    // Matrix that contains the time for different activations levels
    double **DtMatrix = new double*[(size_t)numActuators];
    for(int i=0; i<numActuators; i++){
        DtMatrix[i] = new double[segs];
    }

    for(int i=0; i<numActuators; i++){
        for(int j=0; j<segs; j++){
            DtMatrix[i][j] = xT[j];
            activMatrix[i][j] = yVal[j];
            coeffs[i*segs+j] = yVal[j];
        }
    }

    PrescribedController *muscleController = new PrescribedController();
    muscleController->setActuators(osimModel.updActuators());
    muscleController->setName("MuscleController");

    for(int i=0; i<numActuators; i++){
        muscleController->prescribeControlForActuator( actuators[i].getName(), 
                            new PiecewiseLinearFunction(segs, xT, 
                            activMatrix[i], "ActivationSignal"));
    }


    osimModel.addController(muscleController);
    osimModel.finalizeConnections();

    //OpenSim::PrescribedController *handleController = dynamic_cast<PrescribedController*>(&osimModel.updControllerSet()[0]);

    //std::cout << handleController->getName() << std::endl;
    //FunctionSet &funcSet = handleController->upd_ControlFunctions();
    //for(int i=0; i<numActuators; i++){
    //    PiecewiseConstantFunction *func = dynamic_cast<PiecewiseConstantFunction*>(&funcSet[i]);
    //    for(int j=0; j<segs ; j++){
    //        func->setY(j, coeffs[i*segs+j]/2);
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
    manager.setIntegratorAccuracy(desired_accuracy);

    // Integrate from initial time to final time.
    si.setTime(initialTime);
    osimModel.getMultibodySystem().realize(si, Stage::Acceleration);
    
    manager.initialize(si);
    si = manager.integrate(finalTime);
    auto statesTable = manager.getStatesTable();
    STOFileAdapter_<double>::write(statesTable, "testing.sto");

    std::cout << "Everything Ran Fine" << std::endl;
}