#include <iostream>
#include "OpenSim.h"

int main(){
    OpenSim::ModelVisualizer::addDirToGeometrySearchPaths("../../Geometry/");
    OpenSim::Model osimModel("gait2354_simbody.osim");
    osimModel.setUseVisualizer(true);

    OpenSim::Array<std::string> emptyStringArray("");
    OpenSim::Constant *constPelvixTransZ = new OpenSim::Constant(0);

    OpenSim::JointSet &jointSet = osimModel.updJointSet();
    OpenSim::CustomJoint *ground_pelvis = dynamic_cast<OpenSim::CustomJoint*>(&jointSet[0]);
    OpenSim::SpatialTransform &gpST = ground_pelvis->updSpatialTransform();
    gpST.upd_translation3().setFunction(constPelvixTransZ);
    gpST.upd_translation3().setCoordinateNames(emptyStringArray);
    OpenSim::CoordinateSet &coordSet = osimModel.updCoordinateSet();
    std::cout << coordSet.getSize() << std::endl;
    coordSet.remove(5);         // removing the pelvis_tz coordinate

    std::cout << osimModel.getCoordinateSet().getSize() << std::endl;
    std::cout << ground_pelvis->getCoordinate(5).getName() << std::endl;

    osimModel.finalizeConnections();
    osimModel.print("modifiedModel.osim");

    SimTK::State si = osimModel.initSystem();
    OpenSim::Manager manager(osimModel);
    manager.initialize(si);
    manager.integrate(4);

    std::cout << "Everything ran fine" << std::endl;
}