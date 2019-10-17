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
	osimModel.setUseVisualizer(false);

    // Adding the controller to the model
    PrescribedController *muscleController = new PrescribedController();
    muscleController->setActuators(osimModel.updActuators());
    muscleController->setName("MuscleController");
    osimModel.addController(muscleController);

    // finalizing the connections
    osimModel.finalizeConnections();

    const int segs = 2;
    double xT[segs] = {0.1, 0.2};
    double yVal[segs] = {0.3, 0.7};

    PrescribedController *vinay = dynamic_cast<PrescribedController*> (&osimModel.updControllerSet()[0]);
    auto actuators = vinay->getActuatorSet();
    int numActuators = actuators.getSize(); 
    for(int i=0; i<numActuators; i++){
        vinay->prescribeControlForActuator( actuators[i].getName(), new PiecewiseConstantFunction(segs, xT, yVal, "yay"));
    }

    PrescribedController *vinay1 = dynamic_cast<PrescribedController*> (&osimModel.updControllerSet()[0]);
    auto &raman = vinay1->upd_ControlFunctions();
    for(int i=0; i<numActuators; i++){
        std::cout << raman[i].getName() << std::endl;
        auto sasanka = dynamic_cast<PiecewiseConstantFunction*>(&raman[i]);
        sasanka->setName("Blah Blah Blah");
        sasanka->setX(0,500);
    }
    std::cout << "Set New Values" << std::endl;

    PrescribedController *vinay2 = dynamic_cast<PrescribedController*> (&osimModel.updControllerSet()[0]);
    auto raman2 = vinay2->upd_ControlFunctions();
    for(int i=0; i<numActuators; i++){
        auto sasanka2 = dynamic_cast<PiecewiseConstantFunction*>(&raman2[i]);
        std::cout << sasanka2->getName() << std::endl;
        std::cout << sasanka2->getX(0) << std::endl;
    }
    //FunctionSet funcSet = vinay->upd_ControlFunctions();
    //PiecewiseConstantFunction *func = dynamic_cast<PiecewiseConstantFunction*>(&funcSet[0]);
    //std::cout << func->getX(0) << std::endl;
    //func->setX(0, 0.999);
    //std::cout << func->getX(0) << std::endl;

    //PrescribedController *vinay1 = dynamic_cast<PrescribedController*> (&osimModel.updControllerSet()[0]);
    //FunctionSet funcSet1 = vinay1->upd_ControlFunctions();
    //PiecewiseConstantFunction *func1 = dynamic_cast<PiecewiseConstantFunction*>(&funcSet1[0]);
    //std::cout << func1->getX(0) << std::endl;

    std::cout << "Everything went well" << std::endl;

}
