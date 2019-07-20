/* Build an OpenSim model of a device for assisting our hopping mechanism. The
device consists of two bodies ("cuffA" and "cuffB") connected by a PathActuator
("cableAtoB") that can wrap around the hopper's patella (similar to the vastus
muscle in the hopper). The actuator receives its control signal from a
PropMyoController. Each cuff is attached to the "child" frame of a WeldJoint;
the "parent" frames of these joints will be connected to PhysicalFrames on the
hopper or testbed.

Several lines of code need to be added to this file; see exampleHopperDevice.cpp
and the "TODO" comments below for instructions. */

#include <OpenSim/OpenSim.h>
#include "defineDeviceAndController.h"

static const double OPTIMAL_FORCE{ 4000. };
static const double GAIN{ 1.0 };

namespace OpenSim {

// [Step 2, Task C]
Device* buildDevice() {
    using SimTK::Vec3;
    using SimTK::Inertia;

    // Create the device.
    auto device = new Device();
    device->setName("device");

    // The device's mass is distributed between two identical cuffs that attach
    // to the hopper via WeldJoints (to be added below).
    double deviceMass = 2.0;
    auto cuffA = new Body("cuffA", deviceMass/2., Vec3(0), Inertia(0.5));
    auto cuffB = new Body("cuffB", deviceMass/2., Vec3(0), Inertia(0.5));
    //TODO: Repeat for cuffB.
	device->addComponent(cuffA);
    device->addComponent(cuffB);

    // Attach a sphere to each cuff for visualization.
    auto sphere = new Sphere(0.01);
    sphere->setName("sphere");
    sphere->setColor(SimTK::Red);
    cuffA->attachGeometry(sphere);
    cuffB->attachGeometry(sphere->clone());


	// Create a WeldJoint to anchor cuffA to the hopper.
    auto anchorA = new WeldJoint();
    anchorA->setName("anchorA");
    //TODO: Connect the "child_frame" (a PhysicalFrame) Socket of anchorA to
    //      cuffA. Note that only the child frame is connected now; the parent
    //      frame will be connected in exampleHopperDevice.cpp.
	anchorA->connectSocket_child_frame(*cuffA);

    //TODO: Add anchorA to the device.
	device->addComponent(anchorA);

    //TODO: Create a WeldJoint to anchor cuffB to the hopper. Connect the
    //      "child_frame" Socket of anchorB to cuffB and add anchorB to the
    //      device.
	auto anchorB = new WeldJoint();
	anchorB->setName("anchorB");
	anchorB->connectSocket_child_frame(*cuffB);
	device->addComponent(anchorB);

    // Attach a PathActuator between the two cuffs.
    auto pathActuator = new PathActuator();
    pathActuator->setName("cableAtoB");
    pathActuator->set_optimal_force(OPTIMAL_FORCE);
    pathActuator->addNewPathPoint("pointA", *cuffA, Vec3(0));
    pathActuator->addNewPathPoint("pointB", *cuffB, Vec3(0));
    device->addComponent(pathActuator);

    // Create a PropMyoController.
    auto controller = new PropMyoController();
    controller->setName("controller");
    controller->set_gain(GAIN);

    //TODO: Connect the controller's "actuator" Socket to pathActuator.
	controller->connectSocket_actuator(*pathActuator);

    //TODO: Add the controller to the device.
	device->addComponent(controller);

    return device;
}

} // end of namespace OpenSim
