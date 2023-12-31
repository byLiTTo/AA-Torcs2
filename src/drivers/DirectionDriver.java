/**
 * This class represents a direction-based driver that uses Q-learning to control the steering of the car.
 */
package drivers;

import mdp.QLearning;
import mdp.SteerControl;
import torcs.*;

import static torcs.Constants.SEPARATOR;

/**
 * The DirectionDriver class extends the Controller class and implements the necessary methods to control the car's
 * steering, gear, acceleration, and braking based on the Q-learning algorithm for steering control.
 */
public class DirectionDriver extends Controller {

    // QLearning to Steer Control Variables
    private QLearning steerControlSystem;
    private SteerControl.States currentSteerState;
    private SteerControl.Actions actionSteer;

    // Time, Laps and Statistics Variables
    private int tics;
    private int epochs;
    private int laps;
    private double previosDistanceFromStartLine;
    private double currentDistanceFromStartLine;
    private int completeLaps;
    private double distanceRaced;
    private SensorModel previousSensors;
    private SensorModel currentSensors;

    // Cache variables
    private int stuck;
    private double clutch;
    private boolean completeLap;
    private boolean offTrack;

    /**
     * Constructs an instance of the DirectionDriver class.
     */
    public DirectionDriver() {
        steerControlSystem = new QLearning(Constants.ControlSystems.STEERING_CONTROL_SYSTEM);
        currentSteerState = SteerControl.States.STATE_9;
        actionSteer = SteerControl.Actions.TURN_C;

        tics = 0;
        epochs = 0;
        laps = -1;
        completeLaps = 0;
        completeLaps = 0;
        distanceRaced = 0;

        stuck = 0;
        clutch = 0;
        completeLap = false;
        offTrack = false;
    }

    /**
     * Controls the car's actions based on the current sensor inputs.
     *
     * @param sensors the sensor data received from the car
     * @return an Action object specifying the car's actions
     */
    @Override
    public Action control(SensorModel sensors) {
        if (this.tics == 0) {
            this.previosDistanceFromStartLine = sensors.getDistanceFromStartLine();
            this.currentDistanceFromStartLine = this.previosDistanceFromStartLine;

            this.previousSensors = sensors;
            this.currentSensors = this.previousSensors;

            this.tics++;
        } else {
            this.previosDistanceFromStartLine = this.currentDistanceFromStartLine;
            this.currentDistanceFromStartLine = sensors.getDistanceFromStartLine();

            this.previousSensors = this.currentSensors;
            this.currentSensors = sensors;

            this.tics++;

            System.out.println("Tics: " + this.tics);
            System.out.println("Laps: " + this.laps + "/1");
            System.out.println("Epochs: " + this.epochs + "/" + Constants.MAX_EPOCHS);
            System.out.println("Complete Laps: " + this.completeLaps + "/" + Constants.MAX_EPOCHS);
            System.out.println();
        }

        // Update raced distance
        this.distanceRaced = this.currentSensors.getDistanceRaced();

        // Update complete laps
        if (this.previosDistanceFromStartLine > 1 && this.currentDistanceFromStartLine < 1) {
            this.laps++;

            // Car start back the goal, so ignore first update
            // If the car complete the number of laps, restart the race
            if (this.laps >= 1) {
                this.completeLap = true;

                Action action = new Action();
                action.restartRace = true;
                return action;


            }
        }

        // If the car is off track, restart the race
        if (Math.abs(this.currentSensors.getTrackPosition()) >= 1) {
            this.offTrack = true;

            Action action = new Action();
            action.restartRace = true;
            return action;
        }

        // check if car is currently stuck
        if (Math.abs(this.currentSensors.getAngleToTrackAxis()) > DrivingInstructor.stuckAngle) {
            this.stuck++;
        } else {
            this.stuck = 0;
        }

        // After car is stuck for a while apply recovering policy
        if (this.stuck > DrivingInstructor.stuckTime) {
            // Set gear and steering command assuming car is pointing in a direction out of track

            // To bring car parallel to track axis
            float steer = (float) (-this.currentSensors.getAngleToTrackAxis() / DrivingInstructor.steerLock);
            int gear = -1; // gear R

            // If car is pointing in the correct direction revert gear and steer
            if (this.currentSensors.getAngleToTrackAxis() * this.currentSensors.getTrackPosition() > 0) {
                gear = 1;
                steer = -steer;
            }

            this.clutch = DrivingInstructor.clutching(this.currentSensors, (float) this.clutch, getStage());

            // Build a CarControl variable and return it
            Action action = new Action();
            action.gear = gear;
            action.steering = steer;
            action.accelerate = 1.0;
            action.brake = 0;
            action.clutch = clutch;

            return action;
        }

        // If the car is not stuck
        Action action = new Action();

        // Calculate gear value
//        action.gear = DrivingInstructor.getGear(this.currentSensors);
        action.gear = 1;


        // Calculate steer value
        this.currentSteerState = SteerControl.evaluateSteerState(this.currentSensors);
        this.actionSteer = (SteerControl.Actions) this.steerControlSystem.nextOnlyBestAction(this.currentSteerState);
        double steer = SteerControl.steerAction2Double(this.actionSteer);
        action.steering = steer;

        // Calculate accel/brake
        float accel_and_brake = DrivingInstructor.getAccel(this.currentSensors);

        // Set accel and brake from the joint accel/brake command
        float accel, brake;
        if (accel_and_brake > 0) {
            accel = accel_and_brake;
            brake = 0;
        } else {
            accel = 0;
            // apply ABS to brake
            brake = DrivingInstructor.filterABS(this.currentSensors, -accel_and_brake);
        }
//        action.accelerate = accel;
//        action.brake = brake;
        if (sensors.getSpeed() < 50) {
            action.accelerate = 1;
        }

        // Calculate clutch
//        this.clutch = DrivingInstructor.clutching(this.currentSensors, (float) this.clutch, getStage());
//        action.clutch = this.clutch;

        return action;
    }

    /**
     * Resets the state of the driver.
     */
    @Override
    public void reset() {
        currentSteerState = SteerControl.States.STATE_9;
        actionSteer = SteerControl.Actions.TURN_C;

        if (this.completeLap) {
            this.completeLaps++;
            System.out.println("Complete lap!");
        }
        if (this.offTrack) {
            System.out.println("Out of track!");
        }

        String newResults = this.generateStatistics();
        this.steerControlSystem.saveStatistics(newResults);

        tics = 0;
        epochs++;
        laps = -1;
        completeLaps = 0;
        distanceRaced = 0;

        stuck = 0;
        clutch = 0;
        completeLap = false;
        offTrack = false;

        System.out.println();
        System.out.println("*** Restarting the race ***");
        System.out.println();
    }

    /**
     * Shuts down the driver.
     */
    @Override
    public void shutdown() {
        System.out.println();
        System.out.println("*** Finish the test ***");
        System.out.println();
    }

    /**
     * Generates statistics about the race.
     *
     * @return a string containing the race statistics
     */
    private String generateStatistics() {
        return getTrackName() + SEPARATOR
                + this.epochs + SEPARATOR
                + this.tics + SEPARATOR
                + (int) (this.distanceRaced) + SEPARATOR
                + this.completeLaps + SEPARATOR
                + Constants.MAX_EPOCHS;
    }
}