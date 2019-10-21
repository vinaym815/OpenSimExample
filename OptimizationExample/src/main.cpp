/* -------------------------------------------------------------------------- *
 *  Example of an OpenSim program that optimizes the performance of a model.
 *  The main() loads the arm26 model and maximizes the forward velocity of
 *  the hand during a muscle-driven forward simulation by finding the set
 *  of (constant) controls.
 */

//==============================================================================
//==============================================================================
#include <OpenSim/OpenSim.h>
#include "OpenSim/Common/STOFileAdapter.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

// Global variables to define integration time window, optimizer step count,
// the best solution.
int stepCount = 0;
const int segs = 2;
const double initialTime = 0.0;
const double finalTime = 0.25;
const double xT[segs] = {0.125, 0.2};
const double yVal[segs] = {0.3, 0.7};
const double desired_accuracy = 1.0e-5;
double bestSoFar = Infinity;

class ExampleOptimizationSystem : public OptimizerSystem {
public:
    /* Constructor class. Parameters passed are accessed in the objectiveFunc() class. */
    ExampleOptimizationSystem(int numParameters, State& s, Model& aModel):
        OptimizerSystem(numParameters),
        si(s),
        osimModel(aModel),
        numCoeffs(numParameters)
    {}

    int objectiveFunc(const Vector &newControls,
        bool new_coefficients, Real& f) const override {

        // make a copy of the initial states
        State s = si;

        // Update the control values
        PrescribedController *muscleController = dynamic_cast<PrescribedController*>(&osimModel.updControllerSet()[0]);
        FunctionSet &funcSet = muscleController->upd_ControlFunctions();
        for(int i=0; i<funcSet.getSize(); i++){
            PiecewiseConstantFunction *func = dynamic_cast<PiecewiseConstantFunction*>(&funcSet[i]);
            for(int j=0; j<segs ; j++){
                func->setX(j, newControls[2*i+j]);
                func->setY(j, newControls[(2*i+1)*segs+j]);
            }
        }

        // Integrate from initial time to final time
        Manager manager(osimModel);
        manager.setIntegratorAccuracy(desired_accuracy);
        s.setTime(initialTime);
        osimModel.getMultibodySystem().realize(s, Stage::Acceleration);
        manager.initialize(s);
        s = manager.integrate(finalTime);

        /* Calculate the scalar quantity we want to minimize or maximize.
        *  In this case, we are maximizing forward velocity of the
        *  forearm/hand mass center, so to maximize, compute velocity
        *  and multiply it by -1.
        */
        const auto& hand = osimModel.getBodySet().get("r_ulna_radius_hand");
        osimModel.getMultibodySystem().realize(s, Stage::Velocity);
        Vec3 massCenter = hand.getMassCenter();
        Vec3 velocity = hand.findStationVelocityInGround(s, massCenter);
        f = -velocity[0];
        stepCount++;

        // Use an if statement to only store and print the results of an
        //  optimization step if it is better than a previous result.
        if(f < bestSoFar) {
            bestSoFar = f;
            cout << "\nobjective evaluation #: " << stepCount <<  " bestSoFar = " << f << std::endl;
            for (int i=0; i<numCoeffs; i++)
            {
                std::cout << newControls[i] << ", ";
            }
            std::cout<< std::endl;
        }

      return 0;

   }

private:
    State& si;
    Model& osimModel;
    int numCoeffs;
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
        Object::renameType("Thelen2003Muscle", "Millard2012EquilibriumMuscle");

        // Create a new OpenSim model. This model is similar to the arm26 model,
        // but without wrapping surfaces for better performance.
        Model osimModel("Arm26_Optimize.osim");
		osimModel.setUseVisualizer(true);

        /////////////////////////////////////
        /////// Adding new controller //////
        //////////////////////////////////// 

        OpenSim::Set<Actuator> actuators = osimModel.getActuators();
        const int numActuators = actuators.getSize(); 

        // Creating an array and Vector of time intervals and activations 
        int numCoeffs = 2*numActuators*segs;

        // Coeffs structure (M1t1 M1t2 M1t3 ....  M1tSegs  M1a1 M1a2 ...... M1aSeg M2t1......)
        SimTK::Vector coeffs(numCoeffs);

        double **DtMatrix = new double*[(size_t)numActuators];
        for(int i=0; i<(int)numActuators; i++){
            DtMatrix[i] = new double[segs];
        }
        double **activMatrix = new double*[(size_t)numActuators];
        for(int i=0; i<(int)numActuators; i++){
            activMatrix[i] = new double[segs];
        }

        for(int i=0; i<(int)numActuators; i++){
            for(int j=0; j<segs; j++){
                DtMatrix[i][j] = xT[j];
                activMatrix[i][j] = yVal[j];
                coeffs[2*i*segs+j] = xT[j];
                coeffs[(2*i+1)*segs+j] = yVal[j];
            }
        }

        PrescribedController *muscleController = new PrescribedController();
        muscleController->setActuators(osimModel.updActuators());
        muscleController->setName("MuscleController");

        for(int i=0; i<numActuators; i++){
            muscleController->prescribeControlForActuator( actuators[i].getName(), 
                                new PiecewiseConstantFunction(segs, DtMatrix[i], 
                                activMatrix[i], "ActivationSignal"));
        }

        osimModel.addController(muscleController);
        osimModel.finalizeConnections();


        /////////////////////////////////////////
        ///// Setting up Initial State  /////////
        /////////////////////////////////////////

        // Initialize the system and get the state representing the state system
        State& si = osimModel.initSystem();

        // Initialize the starting shoulder angle.
        const CoordinateSet& coords = osimModel.getCoordinateSet();
        coords.get("r_shoulder_elev").setValue(si, -1.57079633);

        // Set the initial muscle activations and make all tendons rigid.
        const Set<Muscle> &muscleSet = osimModel.getMuscles();
        for(int i=0; i<muscleSet.getSize(); ++i) {
            muscleSet[i].setActivation(si, 0.01);
            muscleSet[i].setIgnoreTendonCompliance(si, true);
        }

        // Make sure the muscles states are in equilibrium
        osimModel.equilibrateMuscles(si);
        std::cout << "Everything Ran Fine" << std::endl;

        ///////////////////////////
        ///// Optimization ////////
        ///////////////////////////
        // Initialize the optimizer system we've defined.
        ExampleOptimizationSystem sys(numCoeffs, si, osimModel);
        Real f = NaN;

        /* Defining the bounds for the coefficients */
        SimTK::Vector lower_bounds(numCoeffs);
        SimTK::Vector upper_bounds(numCoeffs);
        for(int i=0; i<(int)numActuators; i++){
            for(int j=0; j<segs; j++){
                lower_bounds[2*i*segs+j] = 0.01;
                lower_bounds[(2*i+1)*segs+j] = 0.01;
                upper_bounds[2*i*segs+j] = 0.125;
                upper_bounds[(2*i+1)*segs+j] = 0.99;
            }
        }

        sys.setParameterLimits( lower_bounds, upper_bounds );

        // Create an optimizer. Pass in our OptimizerSystem
        // and the name of the optimization algorithm.
        Optimizer opt(sys, SimTK::LBFGSB);
        //Optimizer opt(sys, SimTK::CMAES);

        // Specify settings for the optimizer
        opt.setConvergenceTolerance(0.001);
        opt.useNumericalGradient(true, desired_accuracy);
        opt.setMaxIterations(4);
        opt.setLimitedMemoryHistory(500);

        // Optimize it!
        f = opt.optimize(coeffs);

        /////////////////////////////////////
        //// Simulating best results ////////
        /////////////////////////////////////

        std::cout << "Press Enter To Run"  << std::endl;
        std::cout << std::cin.get();
        FunctionSet &funcSet = muscleController->upd_ControlFunctions();
        for(int i=0; i<funcSet.getSize(); i++){
            PiecewiseConstantFunction *func = dynamic_cast<PiecewiseConstantFunction*>(&funcSet[i]);
            for(int j=0; j<segs ; j++){
                func->setY(j, coeffs[i*segs+j]);
            }
        }

       Manager manager(osimModel);
       manager.setIntegratorAccuracy(desired_accuracy);
       si.setTime(initialTime);
       manager.initialize(si);
       si = manager.integrate(finalTime);

       cout << "\nMaximum hand velocity = " << -f << "m/s" << endl;
       cout << "OpenSim example completed successfully." << endl;

       // Dump out optimization results to a text file for testing
       ofstream ofile;
       ofile.open("Arm26_optimization_result");
       for(int i=0; i<numActuators; i++){
            for (int j=0; j<segs; j++){
                ofile << coeffs[i*segs+j] << " ";
            }
            ofile << endl;
       }
       ofile << -f <<endl;
       ofile.close();
       auto statesTable = manager.getStatesTable();
       STOFileAdapter_<double>::write(statesTable,
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
