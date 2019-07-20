#include <OpenSim/OpenSim.h>
using namespace SimTK;
using namespace OpenSim;

int main()
{
    Model model("vinay1.osim");

    //model.setUseVisualizer(true);

    //BodySet& ram = model.updBodySet();
    //OpenSim::Body* vin2 = &ram.get("humerus");

    //vin2->setMass (5.5);
    //std::cout << vin2->getMass() << std::endl;

    //OpenSim::Body* vin3 = dynamic_cast<OpenSim::Body*>(&model.updComponent("/bodyset/humerus"));
    //std::cout << vin3->getMass() << std::endl;

    model.setUseVisualizer(true);

    model.printSubcomponentInfo();
    State& state = model.initSystem();
    OpenSim::Joint* shoulder = dynamic_cast<OpenSim::Joint*>(&model.updComponent("/jointset/shoulder"));
    shoulder->getCoordinate().setLocked(state, true);
    model.equilibrateMuscles(state);

    // Simulate.
    Manager manager(model);
    manager.initialize(state);
    state = manager.integrate(10);

    std::cout << "Surprisingly everything went fine" << std::endl;
    std::cin.get();

    return 0;
};
