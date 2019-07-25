#include <OpenSim/OpenSim.h>

int main() {
    OpenSim::Model model;
    model.setName("CustomJoinTut");

    // Create two point masses, of 1 kg
    OpenSim::Body* body1 = new OpenSim::Body("body1", 1, SimTK::Vec3(0), SimTK::Inertia(0));

    // Add display geometry.
    OpenSim::Ellipsoid bodyGeometry(0.1, 0.5, 0.1);
    bodyGeometry.setColor(SimTK::Gray);

    // Attach an ellipsoid to a frame located at the center of each body.
    OpenSim::PhysicalOffsetFrame* body1Center = new OpenSim::PhysicalOffsetFrame(
        "body1Center", *body1, SimTK::Transform(SimTK::Vec3(0, 0.5, 0)));
    body1->addComponent(body1Center);
    body1Center->attachGeometry(bodyGeometry.clone());
    model.addBody(body1);

    // Coordinate for actuating the custom joint 
    OpenSim::Coordinate* q1 = new OpenSim::Coordinate();
    q1->setName("q1");
    q1->setDefaultValue(0);
    q1->setDefaultSpeedValue(0);
    double Range[2] = {-2.0943951, 0.17453293};
    q1->setRange(Range);

    // Array of coordinate names required by TransformAxis 
    OpenSim::Array<std::string> coordNames = OpenSim::Array<std::string>();
    coordNames.append(q1->getName());

    // TransformAxis for Spatial Transform
    OpenSim::TransformAxis *axis1 = new OpenSim::TransformAxis(coordNames, SimTK::Vec3(1,0,0));
    OpenSim::LinearFunction *func1 = new OpenSim::LinearFunction(1, 0);
    axis1->setFunction(func1);

    // Spatial Transform for the Custom Joint
    OpenSim::SpatialTransform *SpTrans = new OpenSim::SpatialTransform();
    SpTrans->set_rotation1(*axis1);

    // Custom joint between ground and body 1
    OpenSim::CustomJoint* customJoint = new OpenSim::CustomJoint("customJoint", model.getGround(), *body1, *SpTrans);

    // Giving the ownership of coordinate q1 to Custom Joint
    customJoint->append_coordinates(*q1);

    // Adding custom joint to the model
    model.addJoint(customJoint);

    // Finalizing the models and printing it
    model.finalizeConnections();
    model.print("customjoint.osim");

    //model.setUseVisualizer(true);
    //SimTK::State& state = model.initSystem();
    //OpenSim::Manager manager(model);
    //manager.setIntegratorAccuracy(1e-4);
    //manager.initialize(state);
    //manager.integrate(1e-4);

    std::cout << "Everything ran fine. \nPress enter to exit." << std::endl;
    std::cout << std::cin.get() << std::endl;
    return 0;
};
