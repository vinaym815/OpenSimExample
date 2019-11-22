// Used to replay the simulation results
// Inputs: Model Name, Excitation File Name, Number of segments in the excitation file

#include "OpenSim/OpenSim.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

const double initialTime = 0.0;
const double desiredAccuracy = 1.0e-5;

class sys{
public:
    sys(const char* modelName, int& segs):segs(segs)
    {
        // Creating the model on the heap
        osimModel = new OpenSim::Model(modelName);
        osimModel->setUseVisualizer(true);

        /////////////////////////////////////
        /////// Adding new controller ///////
        ///////////////////////////////////// 
        OpenSim::Set<Actuator> actuators = osimModel->getActuators();
        numActuators = actuators.getSize(); 

        //PrescribedController *muscleController = new PrescribedController();
        muscleController = new PrescribedController();
        muscleController->setActuators(osimModel->updActuators());
        muscleController->setName("MuscleController");

        // Array of Excitation Function
        arrayExtFuns = new OpenSim::PiecewiseConstantFunction *[numActuators];

        // These arrays are only used for memory cleanup
        arrayVecTime = new double *[numActuators];
        arrayVecExt = new double *[numActuators];

        // Adding empty excitation functions to the controller
        for(int i=0; i<numActuators; i++){
            double *vecTime = new double[segs];
            double *vecExt = new double[segs];

            // Creating the excitation function
            OpenSim::PiecewiseConstantFunction *func = new OpenSim::PiecewiseConstantFunction(
                                                    segs, vecTime, vecExt, "ExtSignal");
            
            // Adding the excitation function to the controller
            muscleController->prescribeControlForActuator( actuators[i].getName(), func);

            // Filling up the arrays
            arrayExtFuns[i] = func;
            arrayVecTime[i] = vecTime;
            arrayVecExt[i] = vecExt;
        }

        // Adding the new controller to the model
        osimModel->addController(muscleController);
        osimModel->finalizeConnections();
        
        // Initializing the System
        si = new SimTK::State(osimModel->initSystem());
    }

    ~sys(){
        // Cleaning up the memory
        delete osimModel;
        delete si;

        for(int i=0; i<numActuators; ++i){
            delete[] arrayVecTime[i];
            delete[] arrayVecExt[i];
            //std::cout << arrayExtFuns[i]->getName() << std::endl;
            //delete arrayExtFuns[i];
        }
        delete[] arrayVecExt;
        delete[] arrayVecTime;
        delete[] arrayExtFuns;

        // muscle Controller can be accessed but its deletion leads to core dump
        // std::cout << muscleController->getName() << std::endl;
        // delete muscleController;
    }

    void simulate(const char* fileName){

        // Array that holds up the excitation patterns
        OpenSim::Array<double> arrayVars;

        // Reading excitation patterns from the file
        std::fstream fin;
        fin.open(fileName, std::ios::in);
        std::string temp, value;
        int fileNumActuators = 0, fileNumSegs = 0;
        while(fin >> temp){
            ++fileNumActuators;
            std::stringstream s(temp);
            fileNumSegs = 0;
            while (std::getline(s, value, ',')){
                ++fileNumSegs;
                arrayVars.append(stod(value));
            }
        }
        fin.close();

        // Checking that file appropriate amount of data
        if ((fileNumActuators != numActuators)||(fileNumSegs/2 != segs)){
            std::cout << "Inapproraite data in excitation file specified" << std::endl;
            exit(1);
        }

        // Updating the controller's excitation pattern
        double finalTime = 0;
        for(int i=0; i<numActuators; i++){
            OpenSim::PiecewiseConstantFunction *func = arrayExtFuns[i];
            double time = 0;
            for (int j=0; j<segs; ++j){
                time += arrayVars[2*i*segs+j];
                func->setX(j, time);
                func->setY(j, arrayVars[(2*i+1)*segs+j]);
                if (time>finalTime){
                    finalTime = time;
                }
            }
    
        }
        // Manager for simulation
        Manager manager(*osimModel);
        manager.setIntegratorAccuracy(desiredAccuracy);
        si->setTime(initialTime);
        manager.initialize(*si);

        // Running the simulation
        manager.integrate(finalTime);

        // Saving the simulation results
        auto statesTable = manager.getStatesTable();
        char newName[50];
        strcpy(newName, fileName);
        strcat(newName,".sto");
        STOFileAdapter_<double>::write(statesTable, newName);
    }

private:
    Model *osimModel;
    State *si;
    PrescribedController *muscleController;
    OpenSim::PiecewiseConstantFunction **arrayExtFuns;
    double **arrayVecTime;
    double **arrayVecExt;
    int segs, numActuators;
};

int main(int argc, char *argv[]){

    // Checking the number of inputs;
    if (!(argc==4)){
        std::cout << "Inappropriate Number of Arguments" << std::endl;
        std::cout << "Correct Format:\n>>funcName modelName excitationFile #ExcitationSegments" 
                << std::endl;
        exit(1);
    }

    // Name of the model
    const char *modelName = argv[1];

    // Name of the file containing excitation profile
    const char *fileName = argv[2];

    // Number of segments the excitation profile is made up of 
    int segs = stoi(argv[3]);

    // Creating the system
    sys Vinay(modelName, segs);

    // Simulating 
    Vinay.simulate(fileName);

    // Bye bye
    std::cout << "Simulation Completed !" << std::endl;

    return 0;
}