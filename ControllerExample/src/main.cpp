/* 
 *  Below is an extension example of an OpenSim application that provides its own 
 *  main() routine.  It applies a controller to the forward simulation of a tug-of-war 
 *  between two muscles pulling on a block.
 */

#include <OpenSim/OpenSim.h>
#include "OpenSim/Common/STOFileAdapter.h"

using namespace OpenSim;
using namespace SimTK;

/*
	The controller will try to make the model follow this position,
	velocity and acceleration in the z direction.
 */

double desiredModelZPosition( double t ) {
    return 0.15 * sin( Pi * t );
}

double desiredModelZVelocity( double t ) {
    return 0.15 * Pi*cos( Pi * t );
}

double desiredModelZAcceleration( double t ) {
    return -0.15 * Pi * Pi * sin( Pi * t );
}

/*
 * This controller will try to track a desired trajectory of the block in
 * the tug-of-war model.
 */
class TugOfWarController : public Controller 
{
OpenSim_DECLARE_CONCRETE_OBJECT(TugOfWarController, Controller);

// This section contains methods that can be called in this controller class.
public:
    /**
     * Constructor
     *
     * @param aModel Model to be controlled
     * @param aKp Position gain by which the position error will be multiplied
     */
    TugOfWarController(double aKp, double aKv) : Controller(), kp(aKp), kv(aKv)
    {}

    /*
     * This function is called at every time step for every actuator.
     * @param s Current state of the system
     * @param controls Controls being calculated
     */
    void computeControls(const SimTK::State& s, SimTK::Vector &controls) const override
    {
        // Get the current time in the simulation.
        double t = s.getTime();

        // Read the mass of the block.
        double blockMass = getModel().getBodySet().get( "block" ).getMass();

        // Get pointers to each of the muscles in the model.
        auto leftMuscle = dynamic_cast<const Muscle*>  ( &getActuatorSet().get(0) );
        auto rightMuscle = dynamic_cast<const Muscle*> ( &getActuatorSet().get(1) );

        // Compute the desired position, velocity and accelerations of the block in the tug-of-war
        // model.

        double zdes  = desiredModelZPosition(t);
        double zdesv  = desiredModelZVelocity(t);
        double zdesa = desiredModelZAcceleration(t);

        // Get the z translation coordinate in the model.
        const Coordinate& zCoord = _model->getCoordinateSet().
            get( "blockToGround_zTranslation" );

        // Get the current position of the block in the tug-of-war
        // model.
        double z  = zCoord.getValue(s);
        double zv  = zCoord.getSpeedValue(s);
        //////////////////////////////////////////////////////////////

        // Compute the correction to the desired acceleration arising
        // from the deviation of the block's current position from its
        // desired position (this deviation is the "position error").
        double pErrTerm = kp * ( zdes  - z  );
        double vErrTerm = kv * ( zdesv  - zv  );

        // Compute the total desired acceleration based on the initial
        // desired acceleration plus the position and velocity error term we
        // computed above.
        double desAcc = zdesa + pErrTerm + vErrTerm;

        // Compute the desired force on the block as the mass of the
        // block times the total desired acceleration of the block.
        double desFrc = desAcc * blockMass;

        // Get the maximum isometric force for the left muscle.
        double FoptL = leftMuscle->getMaxIsometricForce();

        // Get the maximum isometric force for the right muscle.
        double FoptR = rightMuscle->getMaxIsometricForce();

        // If desired force is in direction of one muscle's pull
        // direction, then set that muscle's control based on desired
        // force.  Otherwise, set the muscle's control to zero.

        double leftControl = leftMuscle->getMinControl(),
            rightControl = rightMuscle->getMinControl();

        if( desFrc < 0 ) {
            leftControl = std::abs( desFrc ) / FoptL;
        }
        else if( desFrc > 0 ) {
            rightControl = std::abs( desFrc ) / FoptR;
        }
        // Don't allow any control value to be greater than one.
        if( leftControl > leftMuscle->getMaxControl())
            leftControl = leftMuscle->getMaxControl();
        if( rightControl > rightMuscle->getMaxControl())
            rightControl = rightMuscle->getMaxControl();

        // Thelen muscle has only one control
        Vector muscleControl(1, leftControl);  // This is a vectory of one element
        // Add in the controls computed for this muscle to the set of all model controls
        leftMuscle->addInControls(muscleControl, controls);
        // Specify control for other actuator (muscle) controlled by this controller
        muscleControl[0] = rightControl;
        rightMuscle->addInControls(muscleControl, controls);
    }

private:

    /** Position and Velocity gains for this controller */
    double kp;
    double kv;

};


//______________________________________________________________________________
/**
 * Run a forward dynamics simulation with a controller attached to a model.
 * The model consists of a block attached by two muscles to two walls.  The
 * block can make contact with the ground.
 */
int main()
{
    try {
        // Create an OpenSim model from the model file provided.
        Model osimModel( "tugOfWar_model_ThelenOnly.osim" );

		// setting use visualizer to true for visualization
		osimModel.setUseVisualizer(true);

        // Define the initial and final simulation times.
        double initialTime = 0.0;
        double finalTime = 9.0;

        // Set gain for the controller.
        double kp = 1600.0; // position gain
        double kv = 80.0; // position gain

        // Print the control gains and block mass.
        std::cout << std::endl;
        std::cout << "kp = " << kp << std::endl;
        std::cout << "kv = " << kv << std::endl;

        // Create the controller.
        TugOfWarController *controller = new TugOfWarController(kp, kv);

        // Give the controller the Model's actuators so it knows
        // to control those actuators.
        controller->setActuators(osimModel.updActuators());

        // Add the controller to the Model.
        osimModel.addController(controller);

		// Adding a console reporter to the system
		ConsoleReporter* reporter = new ConsoleReporter();
		reporter->set_report_time_interval(1.0);
		reporter->addToReport(osimModel.getComponent("/jointset/blockToGround/blockToGround_zTranslation").getOutput("value"),"z_coordinate");
		osimModel.addComponent(reporter);

        // Initialize the system and get the state representing the
        // system.
        SimTK::State& si = osimModel.initSystem();

		//osimModel.printSubcomponentInfo();
        // Define non-zero (defaults are 0) states for the free joint.
        CoordinateSet& modelCoordinateSet = osimModel.updCoordinateSet();
        // Get the z translation coordinate.
        Coordinate& zCoord = modelCoordinateSet.get( "blockToGround_zTranslation" );
        // Set z translation speed value.
        zCoord.setSpeedValue( si, 0.15 * Pi );

		// This does the same thing as above
		//FreeJoint* Vinay = dynamic_cast<FreeJoint*>(&osimModel.updComponent("/jointset/blockToGround"));
		//Vinay->updCoordinate(FreeJoint::Coord::TranslationZ).setSpeedValue(si, 0.15*Pi);

        // Define the initial muscle states.
		int index1 = 0, index2 = 1;
		Set<Muscle>& Vinay = osimModel.updMuscles();
        ActivationFiberLengthMuscle* muscle1 = dynamic_cast<ActivationFiberLengthMuscle*>( &Vinay.get(index1));
        ActivationFiberLengthMuscle* muscle2 = dynamic_cast<ActivationFiberLengthMuscle*>( &Vinay.get(index2));

//		This is type punning and should be avoided
//        const Set<Muscle>& muscleSet = osimModel.getMuscles();
//        ActivationFiberLengthMuscle* muscle1 = dynamic_cast<ActivationFiberLengthMuscle*>( &muscleSet.get(0) );
//        ActivationFiberLengthMuscle* muscle2 = dynamic_cast<ActivationFiberLengthMuscle*>( &muscleSet.get(1) );
		// This if condition checks if the dynamic cast was successful or not

        if((muscle1 == NULL) || (muscle2 == NULL)){
            throw OpenSim::Exception("ControllerExample: muscle1 or muscle2 is not an ActivationFiberLengthMuscle and example cannot proceed.");
        }
        muscle1->setActivation(si, 0.01 ); // muscle1 activation
        muscle1->setFiberLength(si, 0.2 ); // muscle1 fiber length
        muscle2->setActivation(si, 0.01 ); // muscle2 activation
        muscle2->setFiberLength(si, 0.2 ); // muscle2 fiber length

		//osimModel.printSubcomponentInfo();
		//osimModel.printOutputInfo();

        // Create the manager for the simulation.
        Manager manager(osimModel);
        manager.setIntegratorAccuracy(1.0e-4);

        // Examine the model.
        //osimModel.printDetailedInfo( si, std::cout );

        // Print out the initial position and velocity states.
       // for( int i = 0; i < modelCoordinateSet.getSize(); i++ ) {
       //     std::cout << "Initial " << modelCoordinateSet[i].getName()
       //         << " = " << modelCoordinateSet[i].getValue( si )
       //         << ", and speed = "
       //         << modelCoordinateSet[i].getSpeedValue( si ) << std::endl;
       // }

        // Integrate from initial time to final time.
        si.setTime(initialTime);
        manager.initialize(si);
        std::cout << "\n\nIntegrating from " << initialTime
            << " to " << finalTime << std::endl;
        manager.integrate(finalTime);

        // Save the simulation results.
        auto controlsTable = osimModel.getControlsTable();
        STOFileAdapter_<double>::write(controlsTable, "tugOfWar_controls.sto");

        auto statesTable = manager.getStatesTable();
        STOFileAdapter_<double>::write(statesTable, "tugOfWar_states.sto");

    }
    catch (const std::exception &ex) {
        
        // In case of an exception, print it out to the screen.
        std::cout << ex.what() << std::endl;

        // Return 1 instead of 0 to indicate that something
        // undesirable happened.
        return 1;
    }

    // If this program executed up to this line, return 0 to
    // indicate that the intended lines of code were executed.
	std::cout << "The code ran fine" << std::endl;
	//std::cout << std::cin.get();
    return 0;
}
