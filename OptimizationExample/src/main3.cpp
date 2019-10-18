#include <iostream>
#include <OpenSim/OpenSim.h>

int main()
{
    const int segs=2;
    const int numActuators = 6;
    double yVal[segs] = {0.3, 0.7};

    int numCoeffs = numActuators*segs;
    SimTK::Vector coeffs(numCoeffs);

    double **activMatrix = new double*[numActuators];
    for(int i=0; i<numActuators; i++){
        activMatrix[i] = new double[segs];
    }

    for(int i=0; i<numActuators; i++){
        for(int j=0; j<segs; j++){
            activMatrix[i][j] = yVal[j];
            coeffs[i*segs+j] = yVal[j];
        }
    }

    for(int i=0; i<numActuators; i++){
        for (int j =0; j<segs; j++){
            std::cout << activMatrix[i][j] << ", ";
            
        }
        std::cout << std::endl;
    }

    for(int i=0; i<coeffs.size(); i++){
        std::cout << coeffs[i] << ", ";
    }


    std::cout << std::endl;

    std::cout << "Everything went well" << std::endl;

}
