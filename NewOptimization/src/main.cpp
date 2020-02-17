/* -------------------------------------------------------------------------- *
 *  Used to synthesize High Jump from sitting posture
 *  The variables for the problem are the muscle excitations levels
 *  There is an assumption of symmetry. Symmetry is achieved by providing same values
 *  to the left and right leg muscle excitation functions. 
 *  Debugging: 1) Try different initial values (CMAES might be stuck in initialization)
 *             2) Upd and Get functions provide reference and const reference respectively.
 *                Make sure you are using them correctly.
 */
//==============================================================================
#include "OpenSim/Common/STOFileAdapter.h"
#include "OpenSim/OpenSim.h"
#include <mutex>
#include <math.h>

// # of weights and hyperparameters. They are used to check the validity of input files
const int numHyperParams = 4;

// Number of threads used by the optimizer
const int numThreads = 4;                   // For debugging purposes

const double reportInterval = 0.05;         // Reporting Interval of table reporters(secs) 
const double desiredAccuracy = 1.0e-4;      // Desired Integration Accuracy
const double convTolerance = 1e-4;          // Convergance Tolerance for optimization
const int maxIter = 10000;                  // Maximum number of iterations
double bestSoFar = SimTK::Infinity;         // Initial value of the cost function

// Number of actuators the model has and the number of segments their excitatio profile is made of;
int numActuators, segs;
double simulationDuration;

// Visualization Flag and iteration counter
bool visualize;
int stepCount = 0;

// Lock used for accessing a unique index to the human model and associated functions
std::mutex queueLock;

// Function used to read files with numerical values
void fileReader(std::string fileName, SimTK::Vector &vecOutput){
    // Vector containing the initial guesses
    std::fstream fin;
    fin.open(fileName, std::ios::in);
    std::string line, value;
    int i=0;
    while(std::getline(fin,line)){
        if(line[0] != '#'){
            std::stringstream s(line);
            while(std::getline(s,value,',')){
                vecOutput[i] = stod(value);
                ++i;
            }
        }
    }
    fin.close();
    // Checking if the file provided appropriate amount of data
    if(i != vecOutput.size()){
        std::cout << fileName <<  " do not have correct data" << std::endl;
    }
}


// Custom Optimization Class.
class CustomOptimizationSystem : public SimTK::OptimizerSystem {

private:
    // Number of variables to be optimized
    int numVars;
    const std::string &modelName;
public:
    // The constructor creates arrays of pointers to the human models and associated functions
    CustomOptimizationSystem(const std::string &modelName, const int numVars): OptimizerSystem(numVars), numVars(numVars), 
    modelName(modelName){}

    int objectiveFunc(const SimTK::Vector &newControls,
        bool new_coefficients, SimTK::Real& f) const override {

        std::unique_lock<std::mutex> locker(queueLock);
        // Creating the model 
        OpenSim::Model osimModel(modelName);
        osimModel.setUseVisualizer(visualize);
        locker.unlock();
        
        // Open Loop controller for muscles
        OpenSim::PrescribedController *muscleController = new OpenSim::PrescribedController();

        // Array of muscles within the model
        OpenSim::Set<OpenSim::Actuator> &actuators = osimModel.updActuators();

        // Passing the muscles to the controller
        muscleController->setActuators(actuators);

        // Updating the values of excitation patterns 
        double dT = simulationDuration/segs;

        // Setting up excitation patterns. The +1 is for initial state
        for(int i=0; i<numActuators; i++){
            double *vecTime = new double[segs+1];
            double *vecExt = new double[segs+1];

            // The initial time and muscle excitation levels
            // The default muscle activation levels are set within the model
            vecTime[0] = 0;
            vecExt[0] = 0.01;

            double time = 0;
            int k = i;
            if (i>=numActuators/2){
                k = i-numActuators/2;
            }
            for(int j=0; j<segs ; j++){               // +1 is offset to avoid changing initial value
                time += dT;
                vecTime[j+1] = time;
                vecExt[j+1]  = newControls[k*segs+j];
            }

            // Creating the excitation function
            OpenSim::PiecewiseLinearFunction* func = new OpenSim::PiecewiseLinearFunction(segs+1, 
                                                            vecTime, vecExt, "ExcitationSignal");
            // Adding the excitation function to the actuator
            muscleController->prescribeControlForActuator( actuators[i].getName(), func);
        }

        // Adding new components to the model
        osimModel.addController(muscleController);
        osimModel.finalizeConnections();

        // Initializing the system 
        SimTK::State si = osimModel.initSystem();
        si.setTime(0.0);
        osimModel.equilibrateMuscles(si);

        // Performing the forward simulation
        OpenSim::Manager manager(osimModel);
        //manager.setIntegratorAccuracy(desiredAccuracy);
        manager.initialize(si);

        si = manager.integrate(simulationDuration);

        f = -osimModel.calcMassCenterPosition(si)[1];

        // Engaging Lock
        locker.lock();
        if(f < bestSoFar) {
            bestSoFar = f;
            std::cout << stepCount << ": " << -f << " meters" << std::endl;
        }

        auto statesTable = manager.getStatesTable();
        OpenSim::STOFileAdapter_<double>::write(statesTable, std::to_string(stepCount)+".sto");
        // Updating Step Size
        ++stepCount;

        // Releasing the theread lock
        locker.unlock();
        return 0;
    }

};

/*
 * Defining the optimization problem that computes a set of muscle excitation pattern
 * for high jump 
 */
int main(int argc, const char *argv[])
{
    try {
        if (argc != 5){
            std::cout << "Inappropriate number of function arguments" << std::endl;
            std::cout << "Correct Format: funcName.exe modelName.osim initialGuessFile.txt hyperParametersFile.txt visualize(true/false)" 
                      << std::endl;
            exit(1);
        }

        // Name of the model
        const std::string modelName = argv[1];

        // File containing the initial guess for the excitation pattern
        const std::string initialGuessFile = argv[2]; 

        // File containing the hyperparameters
        const std::string hyperParamFile = argv[3];

        // Visualize Flag
        if ((std::string)argv[4] == "true"){
            visualize = true;
        }
        else {
            visualize = false;
        }

        // Vector containing the hyper parameters
        SimTK::Vector hyperParamsVec(numHyperParams);
        fileReader(hyperParamFile, hyperParamsVec);

        // Simulation Duration
        simulationDuration = hyperParamsVec[0];

        // Number of segments in the excitation profile
        segs = int(hyperParamsVec[1]);

        // Adding the geometry directory to the search path
        OpenSim::ModelVisualizer::addDirToGeometrySearchPaths("../../Geometry");

        // Gettings the number of actuators
        OpenSim::Model osimModel(modelName);
        numActuators = osimModel.getActuators().getSize();

        // Number of variables
        int numVars = segs*numActuators/2;                    

        // Vector containing the initial guesses
        SimTK::Vector vecVars(numVars);
        fileReader(initialGuessFile, vecVars);

        //// Standard deviations for the CMAES algorithm
        SimTK::Vector initStepSize(numVars);
        //// Filling standard deviations array
        for(int i=0; i<numActuators/2; i++){
            for(int j=0; j<segs; j++){
                initStepSize[i*segs+j] = hyperParamsVec[2];            // Standard deviation of muscle excitations
            }
        }

        // Initialize the optimizer system we've defined.
        CustomOptimizationSystem sys(modelName, numVars);
        SimTK::Real f = SimTK::NaN;

        // Docs: https://simbody.github.io/simbody-3.6-doxygen/api/classSimTK_1_1Optimizer.html
        SimTK::Optimizer opt(sys, SimTK::CMAES);
        opt.setAdvancedVectorOption("init_stepsize", initStepSize);
        opt.setAdvancedIntOption("popsize", hyperParamsVec[3]);
        //opt.setAdvancedIntOption("seed", 42);
        opt.setAdvancedRealOption("maxTimeFractionForEigendecomposition", 1);
        opt.setDiagnosticsLevel(2);

        // Setting up the multithreading options
        opt.setAdvancedStrOption("parallel", "multithreading");
        opt.setAdvancedIntOption("nthreads", numThreads);

        // Settings up optimizer settings
        opt.setConvergenceTolerance(convTolerance);
        opt.setMaxIterations(maxIter);
        //opt.setDiagnosticsLevel(3);

        // Running the optimizer
        f = opt.optimize(vecVars);

        std::cout << "Finished Optimization" << std::endl << std::endl;
        std::cout << "Minimized Cost Function Value: " << -f << std::endl;
    }
    catch (const std::exception& ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }

    // End of main() routine.
    return 0;
}