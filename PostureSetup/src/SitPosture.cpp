// This script is used to change the default model posture
// We have serialized the model and not the state as serialization of state is complex
// We are also adding the chair Half plane

#include <OpenSim/OpenSim.h>
#include <iostream>

// Chair Height
const double chairHeight = 0.430;

// Contact Parameters (Copied from Human0914.osim)
const double stiffness = 2.0e6, dissipation = 1, static_friction = 0.9, dynamic_friction=0.8, viscous_friction=0.6, transition_velocity = 0.1;

int main(int argc, const char* argv[])
{
  // Simulation running time
  double finalTime = std::atof(argv[1]);

  // Loading the model
  OpenSim::Model osimModel = OpenSim::Model("Human0914.osim");
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
  chairForce->setTransitionVelocity(transition_velocity);
  chairForce->setName("chairForce");

  // Adding the force to the model
  osimModel.addForce(chairForce);

  //// Adding force reporter
  //OpenSim::ForceReporter *forceReporter = new OpenSim::ForceReporter(&osimModel);
  //osimModel.addAnalysis(forceReporter);

  // Finalizing the connections
  osimModel.finalizeConnections();

  // Setting the Initial posture
  double jointValue[9]={SimTK::convertDegreesToRadians(0.0), 0, 0.655, SimTK::convertDegreesToRadians(90), SimTK::convertDegreesToRadians(-90), 
                    SimTK::convertDegreesToRadians(0.0), SimTK::convertDegreesToRadians(90), SimTK::convertDegreesToRadians(-90), 
                    SimTK::convertDegreesToRadians(0.0)};

  auto& coodSet = osimModel.updCoordinateSet();

  for(int i=0; i<coodSet.getSize(); ++i){
    auto& coord = coodSet[i];
    coord.setDefaultValue(jointValue[i]);
    //std::cout << coord.getName() << ": " << SimTK::convertRadiansToDegrees(coord.getDefaultValue()) << std::endl;
  }

  // Initializing the model
  SimTK::State si = osimModel.initSystem();
  
  // Locking joints to compute the equilibrium pelvis height of the model
  for(int i=0; i<coodSet.getSize(); ++i){
    auto& coord = coodSet[i];
    if (!((i==0)||(i==1)||(i==2))){
      coord.setLocked(si, true);
    }
  }

  // Equilibrating the muscles
  osimModel.equilibrateMuscles(si);

  // Printing the component Info
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
  std::cout << "ComX: " << osimModel.calcMassCenterPosition(si)[0]<< std::endl;

  //// COP Trajectory
  //const OpenSim::TimeSeriesTable &forceTimeSeries = forceReporter->getForcesTable();
  //const std::vector<double> &timeVector = forceTimeSeries.getIndependentColumn();
  //const SimTK::MatrixView &forceMatrix = forceTimeSeries.getMatrix();
  //std::ofstream ofile;
  //ofile.open("output.csv", std::ios::out);
  //for(int i=0; i<forceMatrix.nrow(); i++){
  //  double netForceGround = forceMatrix[i][15] + forceMatrix[i][33] + forceMatrix[i][55];
  //  double netTorqueGround = forceMatrix[i][19] + forceMatrix[i][37] + forceMatrix[i][59];
  //  double COP = netTorqueGround/netForceGround;
  //  ofile << timeVector[i] << ", " <<  COP << "\n";
  //  std::cout << COP << std::endl;
  //}
  //ofile.close();

  //// Getting the equilibrium height and setting it to default
  //auto &pelvisYCoordinate = coodSet[2];
  //double equilibriumHeight = pelvisYCoordinate.getValue(si);
  //pelvisYCoordinate.setDefaultValue(equilibriumHeight);
  //osimModel.print("modifiedHuman0914.osim");

  std::cout << "Everything ran fine" << std::endl;
  return 0;
}