#include <OpenSim/OpenSim.h>

int main() {
    OpenSim::Model model;
    model.setName("bicep_curl");

    // Create two point masses, of 1 kg
    OpenSim::Body* humerus = new OpenSim::Body("humerus", 1, SimTK::Vec3(0), SimTK::Inertia(0));

    // Add display geometry.
    OpenSim::Ellipsoid bodyGeometry(0.1, 0.5, 0.1);
    bodyGeometry.setColor(SimTK::Gray);
    // Attach an ellipsoid to a frame located at the center of each body.
    OpenSim::PhysicalOffsetFrame* humerusCenter = new OpenSim::PhysicalOffsetFrame(
        "humerusCenter", *humerus, SimTK::Transform(SimTK::Vec3(0, 0.5, 0)));
    humerus->addComponent(humerusCenter);
    humerusCenter->attachGeometry(bodyGeometry.clone());

    OpenSim::Coordinate* knee_angle_r = new OpenSim::Coordinate("knee_angle_r",
    OpenSim::Coordinate::MotionType::Rotational, 0, -2.0943951, 0.17453293);

    OpenSim::Array<std::string> coordNames = OpenSim::Array<std::string>();
    coordNames.append("knee_angle_r");

    OpenSim::TransformAxis *axis1 = new OpenSim::TransformAxis(coordNames, SimTK::Vec3(0,0,1));
    axis1->setName("rotation1");
    //OpenSim::LinearFunction *func1 = new OpenSim::LinearFunction(1, 0);
    //axis1->setFunction(func1);
    axis1->print("out.xml");

    OpenSim::SpatialTransform *Vinay = new OpenSim::SpatialTransform();
    Vinay->set_rotation1(*axis1);

    OpenSim::CustomJoint* shoulder = new OpenSim::CustomJoint("shoulder", model.getGround(), *humerus, *Vinay);

    //OpenSim::CustomJoint* shoulder = new OpenSim::CustomJoint("shoulder", model.getGround(), SimTK::Vec3(0), SimTK::Vec3(0), 
//                                                              *humerus, SimTK::Vec3(0), SimTK::Vec3(0),*Vinay);

    shoulder->print("shoudler.xml");
    model.addBody(humerus);
    model.addJoint(shoulder);

    //model.finalizeConnections();
    //model.print("customjoint.osim");

    //model.setUseVisualizer(true);
    //SimTK::State& state = model.initSystem();

    //OpenSim::Manager manager(model);
    //manager.setIntegratorAccuracy(1e-4);
    //manager.initialize(state);
    //manager.integrate(1e-4);

    std::cout << "Everything ran fine. \nPress enter to exit." << std::endl;
    std::cout << std::cin.get();
    return 0;
};
