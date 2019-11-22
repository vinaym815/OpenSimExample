/* -------------------------------------------------------------------------- *
 *  Example of an OpenSim Optimization routine.  
 *  In this code we try to emulate the results of SCONE high jump exmaple 
 *  and match the results
 *  The variables for the problem are the muscle excitations 
 *  i.e. both the excitation levels and their respective time intervals.
 */

//==============================================================================
#include "OpenSim/OpenSim.h"
#include "OpenSim/Common/STOFileAdapter.h"
#include <queue>
#include <mutex>

// Global variables used to define integration time window, optimizer step count,
// the best solution. 
int stepCount = 0;
const double initialTime = 0.0;         // Time at the start of simulation
const double maxFinalTime = 1.5;        // Maximum Simulation duration

// Standard deviations for time steps and muscle excitations for CMAES
const double stdDevDt = 0.04;
const double stdDevExt = 0.1;

// Upper and lower limits of time steps and excitations 
const double minDt = 0.01;
const double minExt = 0.01;
const double maxExt = 0.99;

const double desiredAccuracy = 1.0e-5;      // Desired Integration Accuracy
const double convTolerance = 1e-4;          // Convergance Tolerance for optimization
const int maxIter = 1;                // Maximum number of iterations
double bestSoFar = SimTK::Infinity;         // Value of the best simulation

// Number of threads used by the optimizer
const int numThreads = std::thread::hardware_concurrency();                   

// Number of actuators and segments inside excitatio profile;
int numActuators, segs;

// Whether to resume optimization or not. Can be reset in the main function
bool visualize;

// Lock for accessing queue and saving file
std::mutex queueLock;

// Queue of jobs;
std::queue<int> Queue;

///////////////////////////////////////
/////// Optimization Class ///////////
///////////////////////////////////////

// The constructor creates arrays of pointers
// The threads during objective evalution use these pointers
// Queue is used to ensure that each thread uses different set of pointers
class CustomOptimizationSystem : public SimTK::OptimizerSystem {

private:
    OpenSim::Model **arrayModels;
    SimTK::State **arrayInitialState;

    // Table reporters and Function arrays are created to avoid 
    // dynamic casting during objective evalutaiton
    OpenSim::TableReporterVec3 **arrayComReporters;
    OpenSim::TableReporter **arrayPelvisReporters;
    OpenSim::PiecewiseConstantFunction ***arrayExtFuncs;

    // These three arrays are only used for memory cleanup
    OpenSim::PrescribedController **arrayMuscleCtrls;
    double ***arrayVecTimes;
    double ***arrayVecExts;

public:
    /* Constructor class. Parameters passed are accessed in the objectiveFunc() class. */
    CustomOptimizationSystem(const std::string &modelName, const int numVars): OptimizerSystem(numVars){ 

        // Array of model pointers
        arrayModels = new OpenSim::Model *[numThreads];

        // Array of initial state pointers
        arrayInitialState = new SimTK::State *[numThreads];

        // Array of com reporter pointers
        arrayComReporters = new OpenSim::TableReporterVec3 *[numThreads];

        // Array of pelvis tilt reporter pointers
        arrayPelvisReporters = new OpenSim::TableReporter*[numThreads];

        // Array of excitation functions to the different muscles
        arrayExtFuncs = new OpenSim::PiecewiseConstantFunction **[numThreads];

        // Memory clean up arrays
        arrayMuscleCtrls = new OpenSim::PrescribedController *[numThreads];
        arrayVecTimes = new double **[numThreads];
        arrayVecExts = new double **[numThreads];

        // Filling up the arrays
        for(int i=0; i<numThreads; ++i){

            // Creating the model
            OpenSim::Model *osimModel = new OpenSim::Model(modelName);
            osimModel->setUseVisualizer(visualize);

            // Array to hold the pointers the excitation functions
            OpenSim::PiecewiseConstantFunction **extFuncs= 
                                new OpenSim::PiecewiseConstantFunction* [(size_t)numActuators];
            
            // Creating a controller for the model
            OpenSim::PrescribedController *muscleController = new OpenSim::PrescribedController();

            // Array of Actuators
            OpenSim::Set<OpenSim::Actuator> &actuators = osimModel->updActuators();

            // Adding actuators to the controller
            muscleController->setActuators(actuators);

            double ** vecPtTimes = new double *[(size_t)numActuators];
            double ** vecPtExts = new double *[(size_t)numActuators];
            // Setting up excitation patterns
            for(int i=0; i<numActuators; i++){
                double *vecTime = new double[segs];
                double *vecExt = new double[segs];
                OpenSim::PiecewiseConstantFunction* func = new OpenSim::PiecewiseConstantFunction(segs, 
                                                                vecTime, vecExt, "ExcitationSignal");
                muscleController->prescribeControlForActuator( actuators[i].getName(), func);
                extFuncs[i] = func;
                vecPtTimes[i] = vecTime;
                vecPtExts[i] = vecExt;
            }

            // Creating a center of mass reporter 
            OpenSim::TableReporterVec3 *comReporter = new OpenSim::TableReporterVec3();
            comReporter->setName("comReporter");
            comReporter->addToReport(osimModel->getOutput("com_position"), "COMPosition");
            comReporter->set_report_time_interval(0.05);

            // Creating a Pelvis Tilt reporter 
            OpenSim::TableReporter *pelvisReporter = new OpenSim::TableReporter();
            pelvisReporter->setName("pelvisReporter");
            pelvisReporter->addToReport(osimModel->getComponent("/jointset/ground_pelvis/pelvis_tilt").getOutput("value"),
                                            "pevlisTilt");
            pelvisReporter->set_report_time_interval(0.05);


            // Adding new components to the model
            osimModel->addController(muscleController);
            osimModel->addComponent(comReporter);
            osimModel->addComponent(pelvisReporter);
            osimModel->finalizeConnections();

            // Initializing the system 
            SimTK::State *si = new SimTK::State(osimModel->initSystem());

            // Filling up the Pointer arrays
            arrayModels[i] = osimModel;
            arrayInitialState[i] = si;
            arrayComReporters[i] = comReporter;
            arrayPelvisReporters[i] = pelvisReporter;
            arrayExtFuncs[i] = extFuncs;
            arrayMuscleCtrls[i] = muscleController;
            arrayVecTimes[i] = vecPtTimes;
            arrayVecExts[i] = vecPtExts;
            }
        }

    int objectiveFunc(const SimTK::Vector &newControls,
        bool new_coefficients, SimTK::Real& f) const override {
        
        // Getting a unique index for the thread
        std::unique_lock<std::mutex> locker(queueLock);
        int threadInd = Queue.front();
        Queue.pop();
        locker.unlock();
        
        // Setting pointer to different objects for the unique index
        OpenSim::Model *osimModel = arrayModels[threadInd];
        SimTK::State *si = arrayInitialState[threadInd];
        OpenSim::TableReporterVec3 *comReporter = arrayComReporters[threadInd];
        OpenSim::TableReporter *pelvisReporter = arrayPelvisReporters[threadInd];
        OpenSim::PiecewiseConstantFunction **extFuns = arrayExtFuncs[threadInd];
        
        // Making a copy of the initial state
        SimTK::State s = *si;

        // Updating the values of excitation patterns
        double finalTime = 0;           // Simulation runtime
        for(int i=0; i<numActuators; i++){
            OpenSim::PiecewiseConstantFunction *func = extFuns[i];
            double time = 0;
            for(int j=0; j<segs ; j++){
                time += newControls[2*i*segs+j];
                func->setX(j, time);
                func->setY(j, newControls[(2*i+1)*segs+j]);
            }
            if (time>finalTime){
                finalTime = time;
            }
        }

        // Integrating from initial time to the final time
        OpenSim::Manager manager(*osimModel);
        manager.setIntegratorAccuracy(desiredAccuracy);
        s.setTime(initialTime);
        manager.initialize(s);
        manager.integrate(finalTime);

        // Calculating the scalar quantity we want to maximize.
        auto comTrajectory = comReporter->getTable().getDependentColumnAtIndex(0);
        double maxHeight = 0;
        for(int i=0; i<comTrajectory.nrow(); ++i){
            if (comTrajectory[i][1] > maxHeight){
                maxHeight = comTrajectory[i][1];
            }
        }
        comReporter->clearTable();

        auto pelvisTiltData = pelvisReporter->getTable().getMatrix();
        double penalty = 0;
        for(int i=0; i<pelvisTiltData.nrow(); ++i){
            double pelvisTilt = pelvisTiltData[i][0];
            if((pelvisTilt > SimTK::Pi/6) || (pelvisTilt < -SimTK::Pi/36) ){
                penalty += 0.01;
            }
        }

        pelvisReporter->clearTable();
        ++stepCount;

        // Lock to make sure only one function is changing the best value
        //std::lock_guard<std::mutex> locker(queueLock);
        locker.lock();

        // Computing the cost function
        f = -maxHeight+penalty;

        if(f < bestSoFar) {
            bestSoFar = f;
            std::cout << "\nobjective evaluation #: " << stepCount <<  " bestSoFar = " << f << std::endl;

            // Dumping the optimized results
            std::ofstream ofile;
            ofile.open("Jump_optimization_result.txt", std::ios::out);
            for(int i=0; i<numActuators; ++i){
                 for (int j=0; j<segs; ++j){
                     ofile << newControls[2*i*segs+j] << ",";
                     }
                 for (int j=0; j<segs; ++j){
                     ofile << newControls[(2*i+1)*segs+j];
                     if(j<segs-1){
                         ofile << ",";
                     }
                 }
                 ofile << "\n";
            }
            ofile.close();
        }

        // Adding the unique index back to queue for use by the other threads
        Queue.push(threadInd);
        locker.unlock();
        return 0;
    }
    ~CustomOptimizationSystem(){
        // If we delete the model do its newly added component also get deleted ?
        for(int i=0; i<numThreads; ++i){
            delete arrayModels[i];
            delete arrayInitialState[i];
            //delete arrayMuscleCtrls[i];
            //delete arrayComReporters[i];
            //delete arrayPelvisReporters[i];

            for (int j=0; j<numActuators; ++j){
                //delete arrayExtFuncs[i][j];
                delete[] arrayVecTimes[i][j];
                delete[] arrayVecExts[i][j];
            }
            delete[] arrayExtFuncs[i];
            delete[] arrayVecTimes[i];
            delete[] arrayVecExts[i];
        }
        delete[] arrayModels;
        delete[] arrayInitialState;
        delete[] arrayMuscleCtrls;
        delete[] arrayComReporters;
        delete[] arrayPelvisReporters;
        delete[] arrayExtFuncs;
        delete[] arrayVecTimes;
        delete[] arrayVecExts;
    }
};

/**
 * Defines an optimization problem that computes a set of muscle excitation pattern
 * which maximizes the jump height which avoiding too much pelvis tilt.
 */
int main(int argc, const char *argv[])
{
    try {
        if (argc != 5){
            std::cout << "Inappropriate number of function arguments" << std::endl;
            std::cout << "Correct Format: funcName modelName initialGuessFile visualize(true/false) segments" 
                      << std::endl;
            exit(1);
        }

        // Name of the model
        const std::string modelName = argv[1];

        // File containing the initial guess of excitation pattern
        const std::string initialGuess = argv[2]; 

        // Visualize Flag
        if ((std::string)argv[3] == "true"){
            visualize = true;
        }
        else {
            visualize = false;
        }

        segs = std::stoi(argv[4]);

        // Gettings the number of actuators
        OpenSim::Model osimModel(modelName);
        numActuators = osimModel.getActuators().getSize();
        int numVars = 2*segs*numActuators;

        ///////////////////////////////////
        //// Loading the Initial values//// 
        ///////////////////////////////////
        SimTK::Vector vecVars(numVars);
        {
            std::fstream fin;
            fin.open(initialGuess, std::ios::in);

            std::string temp, value;
            int i = 0, j=0;
            while(fin >> temp){
                std::stringstream s(temp);
                j = 0;
                while (std::getline(s, value, ',')){
                    vecVars[2*i*segs+j] = stod(value);
                    ++j;
                }
                ++i;
            }
            // Checking if the file provided appropriate amount of data
            if((i != numActuators)||(j/2 != segs)){
                std::cout << "The resume file doesn't has correct amount of data" << std::endl;
            }
        }

        /////////////////////////////
        //// Optimization Setup /////
        /////////////////////////////

        // Initialize the optimizer system we've defined.
        CustomOptimizationSystem sys(modelName, numVars);
        SimTK::Real f = SimTK::NaN;

        // Defining the bounds for optimization varibles 
        SimTK::Vector lower_bounds(numVars);
        SimTK::Vector upper_bounds(numVars);

        //// Standard deviations for the CMAES algorithm
        SimTK::Vector initStepSize(numVars); 

        const double maxDt = (maxFinalTime/segs)-0.001;
        //// Filling up the bounds and standard deviations
        for(int i=0; i<(int)numActuators; i++){
            for(int j=0; j<segs; j++){
                lower_bounds[2*i*segs+j] = minDt;                // Lower bound of time step
                lower_bounds[(2*i+1)*segs+j] = minExt;            // Lower bound of muscle excitation
                upper_bounds[2*i*segs+j] = maxDt;                // Upper bound of time step
                upper_bounds[(2*i+1)*segs+j] = maxExt;            // Upper bound of muscle excitation
                initStepSize[2*i*segs+j] = stdDevDt;                // Standard deviation of time steps
                initStepSize[(2*i+1)*segs+j] = stdDevExt;             // Standard deviation of muscle excitations
            }
        }

        // Setting up the upper and lower bounds
        sys.setParameterLimits( lower_bounds, upper_bounds );

        // Create an optimizer. Pass in our OptimizerSystem
        // and the name of the optimization algorithm.
        // Docs: https://simbody.github.io/simbody-3.6-doxygen/api/classSimTK_1_1Optimizer.html
        SimTK::Optimizer opt(sys, SimTK::CMAES);
        opt.setAdvancedVectorOption("init_stepsize", initStepSize);
        //opt.setAdvancedIntOption("popsize", 50);

        // Creating Queue of jobs
        for(int i=0; i<numThreads; ++i){
            Queue.push(i);
        }

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
