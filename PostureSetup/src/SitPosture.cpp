// This script is used to change the default model posture
// We have serialized the model and not the state as serialization of state is complex
// We are also adding the chair Half plane

#include <iostream>
#include "OpenSim/OpenSim.h"

class TerminateSimulation: public SimTK::TriggeredEventHandler {
public:
  TerminateSimulation(const OpenSim::Model &m) : TriggeredEventHandler(SimTK::Stage::Dynamics), coordSet(m.getCoordinateSet()){
    getTriggerInfo().setRequiredLocalizationTimeWindow(1e-4);
    //std::cout << getTriggerInfo().getRequiredLocalizationTimeWindow() << std::endl;
  }

  SimTK::Real getValue(const SimTK::State &s) const {
    for(int i=3; i<coordSet.getSize(); ++i){
      auto& coord = coordSet[i];
      double currValue = coord.getValue(s);
      bool withinRange = (currValue >= coord.getRangeMin()) && (currValue <= coord.getRangeMax());
      if (!withinRange){
        //std::cout << SimTK::convertRadiansToDegrees(currValue) 
        //          << "\t" << SimTK::convertRadiansToDegrees(coord.getRangeMin()) 
        //          << "\t" << SimTK::convertRadiansToDegrees(coord.getRangeMax())
        //          << "\t" << bool(currValue >= coord.getRangeMin())
        //          << "\t" << bool(currValue <= coord.getRangeMax())
        //          << "\t" << withinRange << std::endl;
        return 0;

      }
    }
    return -1;           // Continue the simulation
  }

  void handleEvent(SimTK::State &s, SimTK::Real accuracy, bool& terminate) const {
    // Terminating the simulation
    terminate = true;
  }

private:
  const OpenSim::CoordinateSet &coordSet;
};

int main(int argc, const char* argv[]){

  // Checking up the number of input arguments
  if (argc!=2){
    std::cout << "Inappropriate input arguments" << std::endl;
    std::cout << "CorrectFormat: ./main duration" << std::endl;
    return 1;
  }

  // Chair Height
  const double chairHeight = 0.45;

  // Ground Contact Parameters (Copied from Human0914.osim)
  const double stiffness = 2.0e6, dissipation = 1, static_friction = 0.9;
  const double dynamic_friction=0.8, viscous_friction=0.6, transitionVelocity=0.1;

  // Coordinate Limit Parameters
  // Removed Knee limit force and added rectus femoris from Human0914.osim to obtain Human0916.osim. Original joint stiffness was 2.
  double jointStiffness = 20, damping = 0.2, dq = 0.01;

  // Corrdinate values for Initial posture
  double jointValue[9]={SimTK::convertDegreesToRadians(-10.0), 0, 0.52, SimTK::convertDegreesToRadians(105.0), SimTK::convertDegreesToRadians(-105.0), 
                                   SimTK::convertDegreesToRadians(15.0), SimTK::convertDegreesToRadians(105.0), SimTK::convertDegreesToRadians(-105.0), 
                                   SimTK::convertDegreesToRadians(15.0)};

  // Lower limit of coordinates
  double jointValueMin[9]={SimTK::convertDegreesToRadians(-90), -2, -1, SimTK::convertDegreesToRadians(-50), SimTK::convertDegreesToRadians(-120), 
                                   SimTK::convertDegreesToRadians(-60), SimTK::convertDegreesToRadians(-50), SimTK::convertDegreesToRadians(-120), 
                                   SimTK::convertDegreesToRadians(-60)};

  // Upper limit of coordinates
  double jointValueMax[9]={SimTK::convertDegreesToRadians(90), 2, 2, SimTK::convertDegreesToRadians(155), SimTK::convertDegreesToRadians(10), 
                                 SimTK::convertDegreesToRadians(40), SimTK::convertDegreesToRadians(155), SimTK::convertDegreesToRadians(10), 
                                 SimTK::convertDegreesToRadians(40)};

  // Adding the geometric dir to search path
  OpenSim::ModelVisualizer::addDirToGeometrySearchPaths("../../Geometry");

  // Simulation running time
  double finalTime = std::stof(argv[1]);

  // Loading the human model
  OpenSim::Model osimModel = OpenSim::Model("Human0916.osim");
  osimModel.setUseVisualizer(true);

  // Ground Frame
  OpenSim::Ground &ground = osimModel.updGround();
  // Hip Frame
  OpenSim::Body *pelvis = dynamic_cast<OpenSim::Body *>(&osimModel.updComponent("/bodyset/pelvis"));

  // Chair Contact Half Space
  OpenSim::ContactHalfSpace *chairContactHalfSpace = new OpenSim::ContactHalfSpace(SimTK::Vec3(0, chairHeight, 0), 
                                                  SimTK::Vec3(0,0, -0.5*SimTK::Pi), ground, "chairPlane");

  OpenSim::ContactSphere *hipContactSphere = new OpenSim::ContactSphere(0.05, SimTK::Vec3(-0.1, -0.075, 0), *pelvis, "hipContactSphere");
  osimModel.addContactGeometry(chairContactHalfSpace);
  osimModel.addContactGeometry(hipContactSphere);

  OpenSim::HuntCrossleyForce::ContactParameters *contactParams = new OpenSim::HuntCrossleyForce::ContactParameters(stiffness, dissipation, 
                                                                                      static_friction, dynamic_friction, viscous_friction);
  
  contactParams->addGeometry("chairPlane");
  contactParams->addGeometry("hipContactSphere");

  OpenSim::HuntCrossleyForce *chairForce = new OpenSim::HuntCrossleyForce(contactParams);
  chairForce->setTransitionVelocity(transitionVelocity);
  chairForce->setName("chairForce");

  // Adding the force to the model
  osimModel.addForce(chairForce);

  // Adding the force reporter
  OpenSim::ForceReporter *forceReporter = new OpenSim::ForceReporter();
  osimModel.addAnalysis(forceReporter);

  // Setting up the default posture and coordinate limits
  auto& coodSet = osimModel.updCoordinateSet();
  for(int i=0; i<coodSet.getSize(); ++i){
    auto& coord = coodSet[i];
    coord.setDefaultValue(jointValue[i]);
    coord.setRangeMin(jointValueMin[i]);
    coord.setRangeMax(jointValueMax[i]);
    // Adding the coordinate limits to the human controlled joints 
    if(i>2){
      OpenSim::CoordinateLimitForce *forceLimiter = new OpenSim::CoordinateLimitForce(coord.getName(), SimTK::convertRadiansToDegrees(coord.getRangeMax()), 
                                                    jointStiffness, SimTK::convertRadiansToDegrees(coord.getRangeMin()), jointStiffness, damping, dq);
      osimModel.addForce(forceLimiter);
    }
  }

  // Finalizing the connections
  osimModel.finalizeConnections();

  osimModel.buildSystem();
  // Adding the terminal condition to the system
  TerminateSimulation *terminate = new TerminateSimulation(osimModel);
  osimModel.updMultibodySystem().addEventHandler(terminate);
  SimTK::State si = osimModel.initializeState();

  // Equilibrating the muscles
  osimModel.equilibrateMuscles(si);

  //// Locking joints to compute the equilibrium pelvis height of the model
  //for(int i=0; i<coodSet.getSize(); ++i){
  //  auto& coord = coodSet[i];
  //  if (i==0 || i==1){
  //    coord.setLocked(si, true);
  //  }
  //}

  // Printing the component Info
  //osimModel.printSubcomponentInfo();
  //osimModel.printOutputInfo();

  // Visualization of the posture
  OpenSim::Manager manager(osimModel);
  manager.initialize(si);
  si = manager.integrate(finalTime);

  // Computing the COP
  SimTK::MultibodySystem &mbdSys = osimModel.updMultibodySystem();
  mbdSys.realize(si, SimTK::Stage::Dynamics);
  const SimTK::SpatialVec &groundWrench = mbdSys.getRigidBodyForces(si, SimTK::Stage::Dynamics)[ground.getMobilizedBodyIndex()];

  std::cout << "COP: " << groundWrench[0][2]/groundWrench[1][1] << std::endl;
  std::cout << "COM: " << osimModel.calcMassCenterPosition(si)[0] << std::endl;

  // printing the final foot forces
  const OpenSim::TimeSeriesTable &forceTimeSeries = forceReporter->getForcesTable();
  const SimTK::MatrixView &forceMatrix = forceTimeSeries.getMatrix();
  int lastRow = forceMatrix.nrow()-1;
  std::cout << "Right Foot Force : " << forceMatrix[lastRow][17] << std::endl;
  std::cout << "Left Foot Force : " << forceMatrix[lastRow][35] << std::endl;
  std::cout << "Pelvis Force : " << forceMatrix[lastRow][57] << std::endl;
  //forceReporter->getForceStorage().print("forces.mot");

  //// Setting the default posture to the equilibrium one
  //for(int i=0; i<coodSet.getSize(); ++i){
  //  auto& coord = coodSet[i];
  //  coord.setDefaultValue(coord.getValue(si));
  //}
  //osimModel.print("modifiedHuman0916.osim");

  // Ending statement
  std::cout << "Simulation completed successfully !!" << std::endl;
  return 0;
}