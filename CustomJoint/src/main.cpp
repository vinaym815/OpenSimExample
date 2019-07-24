#include <OpenSim/OpenSim.h>

int main() {
    OpenSim::Model model;
    model.setName("bicep_curl");

    //// Create two point masses, of 1 kg
    //OpenSim::Body* humerus = new OpenSim::Body("humerus", 1, SimTK::Vec3(0), SimTK::Inertia(0));

    //// Add display geometry.
    //OpenSim::Ellipsoid bodyGeometry(0.1, 0.5, 0.1);
    //bodyGeometry.setColor(SimTK::Gray);
    //// Attach an ellipsoid to a frame located at the center of each body.
    //OpenSim::PhysicalOffsetFrame* humerusCenter = new OpenSim::PhysicalOffsetFrame(
    //    "humerusCenter", *humerus, SimTK::Transform(SimTK::Vec3(0, 0.5, 0)));
    //humerus->addComponent(humerusCenter);
    //humerusCenter->attachGeometry(bodyGeometry.clone());

    //model.addBody(humerus);


    OpenSim::Coordinate* knee_angle_r = new OpenSim::Coordinate();
    knee_angle_r->setDefaultValue(0);
    knee_angle_r->setDefaultSpeedValue(0);
    double Range[2] = {-2.0943951, 0.17453293};
    knee_angle_r->setRange(Range);


    //std::cout << knee_angle_r->getName() << std::endl;
    //std::cout << knee_angle_r->getRangeMax() << std::endl;
    ////model.addComponent(knee_angle_r);

    //OpenSim::Constant zero(0);
    //OpenSim::Array<std::string> emptyArray = OpenSim::Array<std::string>();

    //OpenSim::Array<std::string> coordNames = OpenSim::Array<std::string>();
    //coordNames.append("knee_angle_r");

    //OpenSim::TransformAxis *axis1 = new OpenSim::TransformAxis(coordNames, SimTK::Vec3(1,0,0));
    //axis1->setName("rotation1");
    //OpenSim::LinearFunction *func1 = new OpenSim::LinearFunction(1, 0);
    //axis1->setFunction(func1);

    //OpenSim::TransformAxis *axis2 = new OpenSim::TransformAxis(emptyArray, SimTK::Vec3(0,1,0));
    //axis2->setName("rotation2");
    //axis2->setFunction(zero);

    //OpenSim::TransformAxis *axis3 = new OpenSim::TransformAxis(emptyArray, SimTK::Vec3(0,0,1));
    //axis3->setName("rotation3");
    //axis3->setFunction(zero);

    //OpenSim::TransformAxis *translationX = new OpenSim::TransformAxis(emptyArray, SimTK::Vec3(1,0,0));
    //translationX->setName("translationX");
    //translationX->setFunction(zero);

    //OpenSim::TransformAxis *translationY = new OpenSim::TransformAxis(emptyArray, SimTK::Vec3(0,1,0));
    //translationY->setName("translationY");
    //translationY->setFunction(zero);

    //OpenSim::TransformAxis *translationZ = new OpenSim::TransformAxis(emptyArray, SimTK::Vec3(0,0,1));
    //translationZ->setName("translationZ");
    //translationZ->setFunction(zero);

    //OpenSim::SpatialTransform *Vinay = new OpenSim::SpatialTransform();
    //Vinay->set_rotation1(*axis1);
    //Vinay->set_rotation2(*axis2);
    //Vinay->set_rotation3(*axis3);
    //Vinay->set_translation1(*translationX);
    //Vinay->set_translation2(*translationY);
    //Vinay->set_translation3(*translationZ);

    //OpenSim::CustomJoint* shoulder = new OpenSim::CustomJoint("shoulder", model.getGround(), *humerus, *Vinay);
    //model.addJoint(shoulder);

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
