#include <OpenSim/OpenSim.h>
using namespace SimTK;
using namespace OpenSim;

int main() {
    Model model;
    model.setName("bicep_curl");

    // Create two point masses, of 1 kg
    OpenSim::Body* humerus = new OpenSim::Body("humerus", 1, Vec3(0), Inertia(0));
    OpenSim::Body* radius  = new OpenSim::Body("radius",  1, Vec3(0), Inertia(0));

    // Add display geometry.
    Ellipsoid bodyGeometry(0.1, 0.5, 0.1);
    bodyGeometry.setColor(Gray);
    // Attach an ellipsoid to a frame located at the center of each body.
    PhysicalOffsetFrame* humerusCenter = new PhysicalOffsetFrame(
        "humerusCenter", *humerus, Transform(Vec3(0, 0.5, 0)));
    humerus->addComponent(humerusCenter);
    humerusCenter->attachGeometry(bodyGeometry.clone());
    PhysicalOffsetFrame* radiusCenter = new PhysicalOffsetFrame(
        "radiusCenter", *radius, Transform(Vec3(0, 0.5, 0)));
    radius->addComponent(radiusCenter);
    radiusCenter->attachGeometry(bodyGeometry.clone());

    // Connect the bodies with pin joints. Assume each body is 1 m long.
    PinJoint* shoulder = new PinJoint("shoulder",
            // Parent body, location in parent, orientation in parent.
            model.getGround(), Vec3(0), Vec3(0),
            // Child body, location in child, orientation in child.
            *humerus, Vec3(0, 1, 0), Vec3(0));
    PinJoint* elbow = new PinJoint("elbow",
            *humerus, Vec3(0), Vec3(0), *radius, Vec3(0, 1, 0), Vec3(0));
    elbow->updCoordinate().setDefaultValue(0.5*Pi);
	shoulder->updCoordinate().setDefaultValue(0);

	// Adding muscle actuator
    Millard2012EquilibriumMuscle* biceps = new
        Millard2012EquilibriumMuscle("biceps", 200, 0.6, 0.55, 0);
    biceps->addNewPathPoint("origin",    *humerus, Vec3(0, 0.8, 0));
    biceps->addNewPathPoint("insertion", *radius,  Vec3(0, 0.7, 0));
	
    // Add a controller that specifies the excitation of the muscle.
    PrescribedController* brain = new PrescribedController();
    brain->addActuator(*biceps);
    // Muscle excitation is 0.3 for the first 0.5 seconds, then increases to 1.
    brain->prescribeControlForActuator("biceps",
            new StepFunction(0.5, 3, 0.3, 1));

    // Add components to the model.
    model.addBody(humerus);    model.addBody(radius);
    model.addJoint(shoulder);  model.addJoint(elbow);
    model.addForce(biceps);
    model.addController(brain);

	// Adding reporters to the console
	//elbow->printOutputInfo();
	// elbow->getParentFrame().printOutputInfo();
	// model.getComponent("/jointset/elbow").printOutputInfo();

    ConsoleReporter* reporter = new ConsoleReporter();
    reporter->set_report_time_interval(1.0);
    reporter->addToReport(elbow->getCoordinate().getOutput("value"), "elbow_angle");

	//model.printSubcomponentInfo();
	//model.printOutputInfo();
	ConsoleReporterVec3* reporterVec3 = new ConsoleReporterVec3();
	reporterVec3->setName("Vec3Reporter");
    reporterVec3->set_report_time_interval(1.0);
    //reporterVec3->addToReport(humerus->getOutput("angular_velocity"), "humerus_position");
	reporterVec3->addToReport(model.getOutput("com_position"), "COMPosition");

    model.addComponent(reporter);
    model.addComponent(reporterVec3);

	model.finalizeConnections();
	model.print("vinay1.osim");

    model.setUseVisualizer(true);
    State& state = model.initSystem();

	// The get method gives a const reference to the coordinate while the updCoordinate
	//  give a writable reference to the joint coordinate. SetValue is a const function which
	// sets the value of the coordinate on to the state therefore it doesn't matter if we use the
	// getCoordinate() or updCoordinate() for pin joint
    // Fix the shoulder at its default angle and begin with the elbow flexed.
    //elbow->getCoordinate().setValue(state, 0.5 * Pi);
    //elbow->updCoordinate().setValue(state, 0.5*Pi);

    shoulder->getCoordinate().setLocked(state, true);

    model.equilibrateMuscles(state);

	// Its perfoms the same computations as performed by the manager.integrate
    // Simulate.
    simulate(model, state, 10.0);

    return 0;
};
