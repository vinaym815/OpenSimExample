/* -------------------------------------------------------------------------- *
 *  Example of an OpenSim Optimization routine.  
 *  In this code the horizontal velocity of the hand at the final time step is  
 *  is maximized. The variables for the problem are the muscle activation targets
 *  i.e. both the activation levels and their respective time intervals.
 */

//==============================================================================
#include <OpenSim/OpenSim.h>
#include "OpenSim/Common/STOFileAdapter.h"

// Global variables used to define integration time window, optimizer step count,
// the best solution. 
int stepCount = 0;
const double initialTime = 0.0;
const double finalTime = 0.25;
const double desiredAccuracy = 1.0e-5;
double bestSoFar = SimTK::Infinity;

// The number of segments muscle activation function is made up of 
const int segs = 2;

// Initial gusses of muscle activation levels and the corresponsing time intervals
const double dT0[segs] = {0.1, 0.1};
const double aL0[segs] = {0.3, 0.3};

class ExampleOptimizationSystem : public SimTK::OptimizerSystem {
public:
    /* Constructor class. Parameters passed are accessed in the objectiveFunc() class. */
    ExampleOptimizationSystem(int numParameters, SimTK::State& s, OpenSim::Model& aModel):
        OptimizerSystem(numParameters),
        si(s),
        osimModel(aModel),
        numVars(numParameters)
    {}

    int objectiveFunc(const SimTK::Vector &newControls,
        bool new_coefficients, SimTK::Real& f) const override {

        // make a copy of the initial states
        SimTK::State s = si;

        // Updating the values of the control function
        OpenSim::PrescribedController *muscleController = dynamic_cast<OpenSim::PrescribedController*>(&osimModel.updControllerSet()[0]);
        OpenSim::FunctionSet &funcSet = muscleController->upd_ControlFunctions();

        for(int i=0; i<funcSet.getSize(); i++){
            OpenSim::PiecewiseConstantFunction *func = dynamic_cast<OpenSim::PiecewiseConstantFunction*>(&funcSet[i]);
            double time = 0;
            for(int j=0; j<segs ; j++){
                time += newControls[2*i*segs+j];
                func->setX(j, time);
                func->setY(j, newControls[(2*i+1)*segs+j]);
            }
        }

        // Integrate from initial time to the final time
        OpenSim::Manager manager(osimModel);
        manager.setIntegratorAccuracy(desiredAccuracy);
        s.setTime(initialTime);
        osimModel.getMultibodySystem().realize(s, SimTK::Stage::Acceleration);
        manager.initialize(s);
        s = manager.integrate(finalTime);

        /* Calculate the scalar quantity we want to minimize or maximize.
        *  In this case, we are maximizing forward velocity of the
        *  forearm/hand mass center, so to maximize, compute velocity
        *  and multiply it by -1.
        */
        const auto& hand = osimModel.getBodySet().get("r_ulna_radius_hand");
        osimModel.getMultibodySystem().realize(s, SimTK::Stage::Velocity);
        SimTK::Vec3 massCenter = hand.getMassCenter();
        SimTK::Vec3 velocity = hand.findStationVelocityInGround(s, massCenter);
        f = -velocity[0];
        stepCount++;

        // Use an if statement to only store and print the results of an
        //  optimization step if it is better than a previous result.
        if(f < bestSoFar) {
            bestSoFar = f;
            std::cout << "\nobjective evaluation #: " << stepCount <<  " bestSoFar = " << f << std::endl;
            //for (int i=0; i<funcSet.getSize(); i++){
            //    for (int j=0; j<segs; j++){
            //        std::cout << newControls[2*i*segs+j] << ", ";
            //    }
            //    std::cout << std::endl;
            //    for (int j=0; j<segs; j++){
            //        std::cout << newControls[(2*i+1)*segs+j] << ", ";
            //    }
            //    std::cout << std::endl << std::endl;
            //}
        }

      return 0;

   }

private:
    SimTK::State& si;
    OpenSim::Model& osimModel;
    int numVars;
};

//______________________________________________________________________________
/**
 * Define an optimization problem that finds a set of muscle controls to maximize
 * the forward velocity of the forearm/hand segment mass center.
 */
int main()
{
    try {
        // Use Millard2012Equilibrium muscles with rigid tendons for better
        // performance.
        OpenSim::Object::renameType("Thelen2003Muscle", "Millard2012EquilibriumMuscle");

        // Create a new OpenSim model. This model is similar to the arm26 model,
        // but without wrapping surfaces for better performance.
        OpenSim::Model osimModel("Arm26_Optimize.osim");
		osimModel.setUseVisualizer(true);

        /////////////////////////////////////
        /////// Adding new controller //////
        //////////////////////////////////// 

        OpenSim::Set<OpenSim::Actuator> actuators = osimModel.getActuators();
        const int numActuators = actuators.getSize(); 

        // Creating an array and Vector of time intervals and activations 
        int numVars = 2*numActuators*segs;

        // Vector of variables that will be optimized 
        // Consists of times and activation muscle by muscle, i.e.:
        // Variables = (M1t1 M1t2 M1t3 ....  M1tSegs  M1a1 M1a2 ...... M1aSeg M2t1......)
        SimTK::Vector vecVars(numVars);
        for(int i=0; i<(int)numActuators; i++){
            for(int j=0; j<segs; j++){
                vecVars[2*i*segs+j] = dT0[j];
                vecVars[(2*i+1)*segs+j] = aL0[j];
            }
        }

        OpenSim::PrescribedController *muscleController = new OpenSim::PrescribedController();
        muscleController->setActuators(osimModel.updActuators());

        // Adding the muscle activation target functions
        for(int i=0; i<numActuators; i++){
            muscleController->prescribeControlForActuator( actuators[i].getName(), 
                                new OpenSim::PiecewiseConstantFunction(segs, &vecVars[2*i*segs], 
                                &vecVars[(2*i+1)*segs], "ActivationSignal"));
        }

        osimModel.addController(muscleController);
        osimModel.finalizeConnections();

        /////////////////////////////////////////
        ///// Setting up Initial State  /////////
        /////////////////////////////////////////

        // Initialize the system and get the state representing the state system
        SimTK::State& si = osimModel.initSystem();
        
        auto &modelVisualizer = osimModel.updVisualizer();
        const std::string geometryDir = "./geometry";
        modelVisualizer.addDirToGeometrySearchPaths(geometryDir);

        // Initialize the starting shoulder angle.
        const OpenSim::CoordinateSet& coords = osimModel.getCoordinateSet();
        coords.get("r_shoulder_elev").setValue(si, -1.57079633);

        // Set the initial muscle activations and make all tendons rigid.
        const OpenSim::Set<OpenSim::Muscle> &muscleSet = osimModel.getMuscles();
        for(int i=0; i<muscleSet.getSize(); ++i) {
            muscleSet[i].setActivation(si, 0.01);
            muscleSet[i].setIgnoreTendonCompliance(si, true);
        }

        // Make sure the muscles states are in equilibrium
        osimModel.equilibrateMuscles(si);

        ///////////////////////////
        ///// Optimization ////////
        ///////////////////////////
        // Initialize the optimizer system we've defined.
        ExampleOptimizationSystem sys(numVars, si, osimModel);
        SimTK::Real f = SimTK::NaN;

        // Defining the bounds for optimization varibles 
        SimTK::Vector lower_bounds(numVars);
        SimTK::Vector upper_bounds(numVars);

        // Standard deviations for the CMAES algorithm
        SimTK::Vector initStepSize(numVars); 

        for(int i=0; i<(int)numActuators; i++){
            for(int j=0; j<segs; j++){
                lower_bounds[2*i*segs+j] = 0.01;
                lower_bounds[(2*i+1)*segs+j] = 0.01;
                upper_bounds[2*i*segs+j] = 0.12;
                upper_bounds[(2*i+1)*segs+j] = 0.99;
                initStepSize[2*i*segs+j] = 0.04;
                initStepSize[(2*i+1)*segs+j] = 0.1;
            }
        }
        sys.setParameterLimits( lower_bounds, upper_bounds );

        // Create an optimizer. Pass in our OptimizerSystem
        // and the name of the optimization algorithm.
        // Docs: https://simbody.github.io/simbody-3.6-doxygen/api/classSimTK_1_1Optimizer.html

    
        SimTK::Optimizer opt(sys, SimTK::CMAES);
        opt.setAdvancedVectorOption("init_stepsize", initStepSize);
        //opt.setAdvancedIntOption("popsize", 200);

        //SimTK::Optimizer opt(sys, SimTK::InteriorPoint);
        //SimTK::Optimizer opt(sys, SimTK::LBFGS);
        //SimTK::Optimizer opt(sys, SimTK::LBFGSB);

        // Specify settings for the optimizer
        opt.setConvergenceTolerance(0.0001);
        //opt.setMaxIterations(4);

        opt.setDiagnosticsLevel(3);

        // Optimize it!
        f = opt.optimize(vecVars);

        std::cout << "Finished Optimization" << std::endl;

        /////////////////////////////////////
        //// Simulating best results ////////
        /////////////////////////////////////

        std::cout << "Press Enter to run the optimized simulation"  << std::endl;
        std::cout << std::cin.get();
		osimModel.setUseVisualizer(true);

        // Updating the activations targets to the one corresponsing to the best result
        OpenSim::FunctionSet &funcSet = muscleController->upd_ControlFunctions();
        for(int i=0; i<funcSet.getSize(); i++){
            OpenSim::PiecewiseConstantFunction *func = dynamic_cast<OpenSim::PiecewiseConstantFunction*>(&funcSet[i]);
            double time = 0;
            for(int j=0; j<segs ; j++){
                time += vecVars[2*i*segs+j];
                func->setX(j, time);
                func->setY(j, vecVars[(2*i+1)*segs+j]);
            }
        }

        // Running the simulation
        OpenSim::Manager manager(osimModel);
        manager.setIntegratorAccuracy(desiredAccuracy);
        si.setTime(initialTime);
        manager.initialize(si);
        si = manager.integrate(finalTime);

        std::cout << "\nMaximum hand velocity = " << -f << "m/s" << std::endl;
        std::cout << "OpenSim example completed successfully." << std::endl;

        // Dump out optimization results to a text file for testing
        std::ofstream ofile;
        ofile.open("Arm26_optimization_result.txt");
        for(int i=0; i<numActuators; i++){
             for (int j=0; j<segs; j++){
                 ofile << vecVars[2*i*segs+j] << " ";
             }
             ofile << std::endl;
             for (int j=0; j<segs; j++){
                 ofile << vecVars[(2*i+1)*segs+j] << " ";
             }
             ofile << std::endl << std::endl;
        }
        ofile << -f << std::endl;
        ofile.close();
        auto statesTable = manager.getStatesTable();
        OpenSim::STOFileAdapter_<double>::write(statesTable,
                                       "Arm26_optimized_states.sto");
    }
    catch (const std::exception& ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }

    // End of main() routine.
    return 0;
}
