/* -------------------------------------------------------------------------- *
 *  Used to synthesize Sit to Stand motion
 *  The variables for the problem are the muscle excitations levels
 *  There is an assumption of symmetry. Symmetry is achieved by providing same values
 *  to the left and right leg muscle excitation functions. 
 *  Debugging: 1) Try different initial values (CMAES might be stuck in initialization)
 *             2) Upd and Get functions provide reference and const reference respectively.
 *                Make sure you are using them correctly.
 */
//==============================================================================
#include "OpenSim/OpenSim.h"
#include <queue>
#include <mutex>
#include <math.h>

// # of weights and hyperparameters. They are used to check the validity of input files
const int numWeights = 6;
const int numHyperParams = 4+numWeights;
double weights[numWeights];

// Number of threads used by the optimizer
//const int numThreads = std::thread::hardware_concurrency();
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

// Queue of jobs;
std::queue<int> Queue;

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
        throw "Error Reading File";
    }
}

/* This class can be used to terminate the simulation if an coordinate exceeds its range
*/
//class TerminateSimulation: public SimTK::TriggeredEventHandler {
//public:
//  TerminateSimulation(const OpenSim::Model &m) : TriggeredEventHandler(SimTK::Stage::Dynamics), coordSet(m.getCoordinateSet()){
//    getTriggerInfo().setRequiredLocalizationTimeWindow(1e-4);
//  }
//
//  SimTK::Real getValue(const SimTK::State &s) const {
//    auto& coord = coordSet[0];
//    double currValue = coord.getValue(s);
//    bool withinRange = (currValue >= coord.getRangeMin()) && (currValue <= coord.getRangeMax());
//    if (!withinRange){
//        // Stopping the simulation
//        return 1;
//    }
//    return -1;           // Continue the simulation
//  }
//
//  void handleEvent(SimTK::State &s, SimTK::Real accuracy, bool& terminate) const {
//    // Terminating the simulation
//    terminate = true;
//  }
//
//private:
//  const OpenSim::CoordinateSet &coordSet;
//};

/* Custom Optimization Class.
*/
class CustomOptimizationSystem : public SimTK::OptimizerSystem {

private:
    OpenSim::Model **arrayModels;
    SimTK::State **arrayInitialState;

    // Table reporters and Function arrays are created to avoid 
    // dynamic casting during objective functions evalutations
    OpenSim::TableReporter **arrayActivReporters;
    //OpenSim::TableReporter **arrayCoordReporters;
    OpenSim::ForceReporter **arrayForceReporters;
    OpenSim::PiecewiseLinearFunction ***arrayExtFuncs;

    // These three arrays are used for memory cleanup. 
    double ***arrayVecTimes;
    double ***arrayVecExts;

    // Number of variables to be optimized
    int numVars;

public:
    // The constructor creates arrays of pointers to the human models and associated functions
    CustomOptimizationSystem(const std::string &modelName, const int numVars): OptimizerSystem(numVars), numVars(numVars){ 

        // Array of model pointers
        arrayModels = new OpenSim::Model *[numThreads];

        // Array of initial state pointers
        arrayInitialState = new SimTK::State *[numThreads];

        //// Array of com reporter pointers
        arrayActivReporters = new OpenSim::TableReporter *[numThreads];

        //// Array of coordinate reporters pointers
        //arrayCoordReporters = new OpenSim::TableReporter *[numThreads];

        // Array of force Reporters.
        arrayForceReporters = new OpenSim::ForceReporter *[numThreads];

        // Array of excitation functions to the different muscles
        arrayExtFuncs = new OpenSim::PiecewiseLinearFunction **[numThreads];

        // Memory clean up arrays. Required as PieceWiseConstant function doesn't
        // claim ownership of the x and y value arrays
        arrayVecTimes = new double **[numThreads];
        arrayVecExts = new double **[numThreads];

        // Filling up the arrays
        for(int i=0; i<numThreads; ++i){

            // Creating the model 
            OpenSim::Model *osimModel = new OpenSim::Model(modelName);
            osimModel->setUseVisualizer(visualize);

            //// Turning on the visualization only for the first window
            //if (i==0 && visualize){
            //    osimModel->setUseVisualizer(visualize);
            //}
           
            //// Muscle activation reporter
            OpenSim::TableReporter *activReporter = new OpenSim::TableReporter();
            activReporter->set_report_time_interval(reportInterval);

            //// Coordinate Values reporter. Used for computing path integral of pelvis posture
            //OpenSim::TableReporter *coordReporter = new OpenSim::TableReporter();
            //coordReporter->set_report_time_interval(reportInterval);
            //OpenSim::CoordinateSet &coordSet = osimModel->updCoordinateSet();
            //for(int i=0; i<3; i++){
            //    coordReporter->addToReport(coordSet[i].getOutput("value"), coordSet[i].getName());
            //}

            // Force Reporter for computing feet force
            OpenSim::ForceReporter *forceReporter = new OpenSim::ForceReporter(osimModel);
            
            // Open Loop controller for muscles
            OpenSim::PrescribedController *muscleController = new OpenSim::PrescribedController();

            // Array of muscles within the model
            OpenSim::Set<OpenSim::Actuator> &actuators = osimModel->updActuators();

            // Passing the muscles to the controller
            muscleController->setActuators(actuators);

            // Array to hold the pointers the excitation functions
            OpenSim::PiecewiseLinearFunction **extFuncs= 
                                new OpenSim::PiecewiseLinearFunction* [numActuators];

            // Memory cleanup arrays
            double ** vecPtTimes = new double *[numActuators];
            double ** vecPtExts = new double *[numActuators];

            // Setting up excitation patterns. The +1 is for initial state
            for(int i=0; i<numActuators; i++){
                double *vecTime = new double[segs+1];
                double *vecExt = new double[segs+1];

                // The initial time and muscle excitation levels
                // The default muscle activation levels are set within the model
                vecTime[0] = 0;
                vecExt[0] = 0.05;

                // Creating the excitation function
                OpenSim::PiecewiseLinearFunction* func = new OpenSim::PiecewiseLinearFunction(segs+1, 
                                                                vecTime, vecExt, "ExcitationSignal");
                // Adding the excitation function to the actuator
                muscleController->prescribeControlForActuator( actuators[i].getName(), func);

                // Adding the muscle activation to the reporter
                activReporter->addToReport(actuators[i].getOutput("activation"), actuators[i].getName());
                extFuncs[i] = func;
                vecPtTimes[i] = vecTime;
                vecPtExts[i] = vecExt;
            }

            // Adding new components to the model
            osimModel->addController(muscleController);
            osimModel->addComponent(activReporter);
            //osimModel->addComponent(coordReporter);
            osimModel->addAnalysis(forceReporter);
            osimModel->finalizeConnections();
            osimModel->buildSystem();

            //// Adding the termination event handler
            //TerminateSimulation *terminate = new TerminateSimulation(*osimModel);
            //osimModel->updMultibodySystem().addEventHandler(terminate);

            // Initializing the system 
            SimTK::State *si = new SimTK::State(osimModel->initializeState());
            si->setTime(0.0);
            osimModel->equilibrateMuscles(*si);

            // Filling up the Pointer arrays
            arrayModels[i] = osimModel;
            arrayInitialState[i] = si;
            arrayActivReporters[i] = activReporter;
            //arrayCoordReporters[i] = coordReporter;
            arrayForceReporters[i] = forceReporter;
            arrayExtFuncs[i] = extFuncs;
            arrayVecTimes[i] = vecPtTimes;
            arrayVecExts[i] = vecPtExts;
            }
        }

    int objectiveFunc(const SimTK::Vector &newControls,
        bool new_coefficients, SimTK::Real& f) const override {
        
        // Getting a unique index for accessing the human model 
        std::unique_lock<std::mutex> locker(queueLock);
        int threadInd = Queue.front();
        Queue.pop();
        locker.unlock();
        
        // Gettings pointers to the human model using the unique index
        OpenSim::Model *osimModel = arrayModels[threadInd];
        SimTK::State *si = arrayInitialState[threadInd];
        OpenSim::TableReporter *activReporter = arrayActivReporters[threadInd];
        //OpenSim::TableReporter *coordReporter = arrayCoordReporters[threadInd];
        OpenSim::ForceReporter *forceReporter = arrayForceReporters[threadInd];
        OpenSim::PiecewiseLinearFunction **extFuns = arrayExtFuncs[threadInd];
        
        // Updating the values of excitation patterns 
        double dT = simulationDuration/segs;
        for(int i=0; i<numActuators; i++){
            OpenSim::PiecewiseLinearFunction *func = extFuns[i];
            double time = 0;
            int k = i;
            if (i>=numActuators/2){
                k = i-numActuators/2;
            }
            for(int j=0; j<segs ; j++){               // +1 is offset to avoid changing initial value
                time += dT;
                double excitation = newControls[k*segs+j];
                func->setX(j+1, time);
                func->setY(j+1, excitation);
            }
        }

        // Performing the forward simulation
        OpenSim::Manager manager(*osimModel);
        manager.setIntegratorAccuracy(desiredAccuracy);
        manager.initialize(*si);
        SimTK::State siFinal = manager.integrate(simulationDuration);

        //// Computing the Pelvis posture (theta,x,y) and velocity error at the final Time 
        SimTK::Vec3 pelvisTargetPos = SimTK::Vec3(0.0, 0.35, 0.935);

        //// Computing the path integral of posture cost
        //double costPelvisLinearPos = 0;
        //double costPelvisAngularPos = 0;
        //auto &pelvisMatrix = coordReporter->getTable().getMatrix();
        //for(int i=0; i<pelvisMatrix.nrow(); i++){
        //    costPelvisLinearPos += std::pow(pelvisMatrix[i][1] - pelvisTargetPos[1], 2) 
        //                        + std::pow(pelvisMatrix[i][2] - pelvisTargetPos[2], 2);
        //    costPelvisAngularPos += std::pow(SimTK::convertRadiansToDegrees(pelvisMatrix[i][0] - pelvisTargetPos[0]), 2);
        //}
        //costPelvisLinearPos = std::sqrt(costPelvisLinearPos/pelvisMatrix.nrow());
        //costPelvisAngularPos = std::sqrt(costPelvisAngularPos/pelvisMatrix.nrow());
        //coordReporter->clearTable();

        SimTK::Vec3 pelvisFinalPos(0);
        SimTK::Vec3 pelvisFinalVel(0);
        const OpenSim::CoordinateSet &coordSet = osimModel->getCoordinateSet();
        for(int i=0; i<3; i++){
            OpenSim::Coordinate &coord = coordSet[i];
            pelvisFinalPos[i] = coord.getValue(siFinal);
            pelvisFinalVel[i] = coord.getSpeedValue(siFinal);
        }
        double costPelvisAngularPos = fabs(SimTK::convertRadiansToDegrees(pelvisFinalPos[0]-pelvisTargetPos[0]));
        double costPelvisLinearPos = std::sqrt(std::pow(pelvisFinalPos[1]-pelvisTargetPos[1],2) +
                                        std::pow(pelvisFinalPos[2]-pelvisTargetPos[2],2));
        double costPelvisAngularVel = fabs(SimTK::convertRadiansToDegrees(pelvisFinalVel[0]));
        double costPelvisLinearVel = std::sqrt(std::pow(pelvisFinalVel[1], 2) + std::pow(pelvisFinalVel[2], 2)); 

        //// Computing cost associated with feet contact
        const OpenSim::TimeSeriesTable &forceTimeSeries = forceReporter->getForcesTable();
        const SimTK::MatrixView &forceMatrix = forceTimeSeries.getMatrix();
        double bodyWeight = osimModel->getTotalMass(siFinal)*osimModel->getGravity()[1];

        //// Difference between the feet force and the body weight at the final frame
        int lastRow = forceMatrix.nrow()-1;
        double costFeetForce = fabs(forceMatrix[lastRow][29]+forceMatrix[lastRow][47] - bodyWeight);

        // Average difference between the feet force and the body weight
        //double costFeetForce = 0;
        //for(int i=0; i<forceMatrix.nrow(); i++){
        //    costFeetForce += SimTK::square(forceMatrix[i][17]+forceMatrix[i][35] - bodyWeight);
        //}
        //costFeetForce = std::sqrt(costFeetForce/forceMatrix.nrow());

        //// Computing sum of squared muscle activations
        double costExtSignal = activReporter->getTable().getMatrix().normRMS();

        // Cost associated with the excitation values
        //double costExtSignal = 0;
        //for(int i=0; i<numActuators; i++){
        //    int k = i;
        //    if (i>numActuators/2){
        //        k = i - numActuators/2; 
        //    }
        //    for(int j=0; j<segs; j++){
        //        costExtSignal += std::pow(newControls[k*segs+j],2);
        //    }
        //}
        //costExtSignal = std::sqrt(costExtSignal/(segs*numActuators));

        //// Clearing the activation reporters 
        activReporter->clearTable();

        //// Computing the different weighted costs
        double C0 = weights[0]*costPelvisLinearPos;
        double C1 = weights[1]*costPelvisAngularPos;
        double C2 = weights[2]*costPelvisLinearVel;
        double C3 = weights[3]*costPelvisAngularVel;
        double C4 = weights[4]*costFeetForce;
        double C5 = weights[5]*costExtSignal;

        // Computing the total cost
        f = C0 + C1 + C2 + C3 + C4 + C5;

        // Engaging Lock
        locker.lock();

        // Printing the objective evaluation for first thread
        if(threadInd == 0){
            std::cout << "objective evaluation #: " << stepCount <<  " bestSoFar = " << f << std::endl;
            std::cout << "Weighted Costs:\t" << C0 << ",\t" << C1 << ",\t" << C2 << ",\t" << C3 
                    << ",\t" << C4 << ",\t" << C5 << "\n";
            std::cout << "Diff Costs:\t" << costPelvisLinearPos << ",\t" << costPelvisAngularPos << ",\t"
                    << costPelvisLinearVel << ",\t" << costPelvisAngularVel << ",\t" 
                    << costFeetForce << ",\t" << costExtSignal << std::endl;
        }
        if(f < bestSoFar) {
            bestSoFar = f;

            // Dumping the optimized results
            std::ofstream ofile;
            ofile.open("STS_symmetric_results.txt", std::ios::out);
            for(int i=0; i<numActuators/2; ++i){
                 for (int j=0; j<segs; ++j){
                     ofile << newControls[i*segs+j] << ",";
                     }
                 ofile << "\n";
            }
            ofile.close();

            // Writing the optimization logs
            std::ofstream logFile;
            logFile.open("log_symmetric.txt", std::ios::app);
            logFile << "objective evaluation #: " << stepCount <<  " bestSoFar = " << f << "\n";
            logFile << "Weighted Costs:\t" << C0 << ",\t" << C1 << ",\t" << C2 << ",\t" << C3 
                    << ",\t" << C4 << ",\t" << C5 << "\n";
            logFile << "Diff Costs:\t" << costPelvisLinearPos << ",\t" << costPelvisAngularPos << ",\t"
                    << costPelvisLinearVel << ",\t" << costPelvisAngularVel << ",\t" 
                    << costFeetForce << ",\t" << costExtSignal << "\n";
            logFile.close();
        }

        // Adding the unique index back to queue for use by the other threads
        Queue.push(threadInd);

        // Updating Step Size
        ++stepCount;

        // Releasing the theread lock
        locker.unlock();
        return 0;
    }

    ~CustomOptimizationSystem(){
        for(int i=0; i<numThreads; ++i){
            delete arrayModels[i];
            delete arrayInitialState[i];

            for (int j=0; j<numActuators; ++j){
                delete[] arrayVecTimes[i][j];
                delete[] arrayVecExts[i][j];
            }
            delete[] arrayExtFuncs[i];
            delete[] arrayVecTimes[i];
            delete[] arrayVecExts[i];
        }
        delete[] arrayModels;
        delete[] arrayInitialState;
        delete[] arrayActivReporters;
        //delete[] arrayCoordReporters;
        delete[] arrayExtFuncs;
        delete[] arrayVecTimes;
        delete[] arrayVecExts;
    }
};

/*
 * Defining the optimization problem that computes a set of muscle excitation pattern
 * for Sit-to-stand motion
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

        // Filling up the weights for the cost function
        for(int i=0; i<numWeights; i++){
            weights[i] = hyperParamsVec[numHyperParams-numWeights+i];
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

        // Creating Queue of jobs
        for(int i=0; i<numThreads; ++i){
            Queue.push(i);
        }

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