#include "OpenSim/OpenSim.h"
#include "OpenSim/Common/STOFileAdapter.h"
#include <iostream>
#include <thread>

class armModel{
    private:
        OpenSim::Model osimModel;
    public:

    armModel(const char* modelName){
        osimModel.setName(modelName);
        osimModel.setUseVisualizer(true);

        // Create two point masses, of 1 kg
        OpenSim::Body* humerus = new OpenSim::Body("humerus", 1, SimTK::Vec3(0), SimTK::Inertia(0));
        OpenSim::Body* radius  = new OpenSim::Body("radius",  1, SimTK::Vec3(0), SimTK::Inertia(0));

        OpenSim::WrapCylinder* wrapCylinder1 = new OpenSim::WrapCylinder();
        wrapCylinder1->set_length(0.1);
        wrapCylinder1->set_radius(0.1);
        OpenSim::PhysicalOffsetFrame* wrapCylinderFrame1 = new OpenSim::PhysicalOffsetFrame("wrapCylinderFrame", *humerus, 
                                                                                    SimTK::Transform(SimTK::Vec3(0.14,0.3,0.0)));
        wrapCylinderFrame1->addWrapObject(wrapCylinder1);
        humerus->addComponent(wrapCylinderFrame1);

        // Add display geometry.
        OpenSim::Ellipsoid bodyGeometry(0.1, 0.5, 0.1);
        bodyGeometry.setColor(SimTK::Gray);

        // Attach an ellipsoid to a frame located at the center of each body.
        OpenSim::PhysicalOffsetFrame* humerusCenter = new OpenSim::PhysicalOffsetFrame("humerusCenter", *humerus, SimTK::Transform(SimTK::Vec3(0, 0.5, 0)));
        humerus->addComponent(humerusCenter);
        humerusCenter->attachGeometry(bodyGeometry.clone());
        OpenSim::PhysicalOffsetFrame* radiusCenter = new OpenSim::PhysicalOffsetFrame("radiusCenter", *radius, SimTK::Transform(SimTK::Vec3(0, 0.5, 0)));
        radius->addComponent(radiusCenter);
        radiusCenter->attachGeometry(bodyGeometry.clone());

        // Connect the bodies with pin joints. Assume each body is 1 m long.
        OpenSim::PinJoint* shoulder = new OpenSim::PinJoint("shoulder",
                // Parent body, location in parent, orientation in parent.
                osimModel.getGround(), SimTK::Vec3(0), SimTK::Vec3(0),
                // Child body, location in child, orientation in child.
                *humerus, SimTK::Vec3(0, 1, 0), SimTK::Vec3(0));
        OpenSim::PinJoint* elbow = new OpenSim::PinJoint("elbow",
                *humerus, SimTK::Vec3(0), SimTK::Vec3(0), *radius, SimTK::Vec3(0, 1, 0), SimTK::Vec3(0));
        elbow->updCoordinate().setDefaultValue(0.5*SimTK::Pi);
    	shoulder->updCoordinate().setDefaultValue(0);

    	//// Adding muscle actuator
        OpenSim::Millard2012EquilibriumMuscle* biceps = new OpenSim::Millard2012EquilibriumMuscle("biceps", 200, 0.6, 0.55, 0);
        biceps->addNewPathPoint("origin",    *humerus, SimTK::Vec3(0, 0.8, 0));
        biceps->addNewPathPoint("insertion", *radius,  SimTK::Vec3(0, 0.7, 0));
        biceps->updGeometryPath().addPathWrap(*wrapCylinder1);
    
        // Add a controller that specifies the excitation of the muscle.
        OpenSim::PrescribedController* brain = new OpenSim::PrescribedController();
        brain->addActuator(*biceps);
        double time[6] = {0,1,2,3,4};
        double exts[6] = {0.01, 0.01, 0.01, 0.01};
        OpenSim::PiecewiseConstantFunction* extFun = new OpenSim::PiecewiseConstantFunction(6,time,exts,"excitationFunc");

        brain->prescribeControlForActuator("biceps", extFun);

        // Add components to the model.
        osimModel.addBody(humerus);    osimModel.addBody(radius);
        osimModel.addJoint(shoulder);  osimModel.addJoint(elbow);
        osimModel.addForce(biceps);
        osimModel.addController(brain);
        osimModel.finalizeConnections();

    };

    void simulate(){
        SimTK::State si = osimModel.initSystem();
        si.setTime(0.0);

        OpenSim::Coordinate *shoulder = dynamic_cast<OpenSim::Coordinate *>(&osimModel.updComponent("/jointset/shoulder/shoulder_coord_0"));
        shoulder->setLocked(si,SimTK::convertDegreesToRadians(90));
        OpenSim::Manager manager(osimModel);
        manager.initialize(si);
        manager.integrate(6);
        auto stateTable = manager.getStatesTable();
        OpenSim::STOFileAdapter_<double>::write( stateTable, "states.sto");
    };
};

void DoWork(){
    armModel model("armModel");
    model.simulate();
}

int main()
{
    std::thread t1(DoWork); 
    t1.join();
    std::cout << "Everything went fine" << std::endl;
    return 0;
};