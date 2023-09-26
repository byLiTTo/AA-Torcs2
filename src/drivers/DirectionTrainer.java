package drivers;

import mdp.QLearning;
import mdp.SteerControl;
import torcs.*;

import static torcs.Constants.SEPARATOR;

/**
 * A driver that uses Q-learning to control the steering direction of the car.
 */
public class DirectionTrainer extends Controller {
    private final double LIMITER_SPEED = 30.0;
    private final double TRACK_LIMIT = 0.85;
    // QLearning to Steer Control Variables
    private QLearning steerControlSystem;
    private SteerControl.States previousSteerState;
    private SteerControl.States currentSteerState;
    private SteerControl.Actions actionSteer;
    private double steerReward;
    // Time, Laps and Statistics Variables
    private int tics;
    private int epochs;
    private int laps;
    private double previosDistanceFromStartLine;
    private double currentDistanceFromStartLine;
    private int completeLaps;
    private double distanceRaced;
    private double highSpeed;
    private SensorModel previousSensors;
    private SensorModel currentSensors;
    // Cache variables
    private int stuck;
    private double clutch;
    private boolean completeLap;
    private boolean offTrack;
    private boolean timeOut;

    /**
     * Initializes a new instance of the DirectionTrainer class.
     */
    public DirectionTrainer() {
        steerControlSystem = new QLearning(Constants.ControlSystems.STEERING_CONTROL_SYSTEM, Constants.MAX_EPOCHS, Constants.RANGE_EPOCHS_END);
        previousSteerState = SteerControl.States.STATE_9;
        currentSteerState = SteerControl.States.STATE_9;
        actionSteer = SteerControl.Actions.TURN_C;
        steerReward = 0;

        tics = 0;
        epochs = 0;
        laps = -1;
        completeLaps = 0;
        distanceRaced = 0;
        highSpeed = 0;

        stuck = 0;
        clutch = 0;
        completeLap = false;
        offTrack = false;
        timeOut = false;
    }

    /**
     * Controls the car based on the current sensor inputs.
     *
     * @param sensors the sensor inputs received from the car
     * @return the action to be performed by the car
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
        }

        // Check if time-out
        if (this.currentSensors.getLastLapTime() > 240.0) {
            this.timeOut = true;

            Action action = new Action();
            action.restartRace = true;
            return action;
        }

        // Update raced distance
        this.distanceRaced = this.currentSensors.getDistanceRaced();

        // Update high speed
        if (this.currentSensors.getSpeed() > this.highSpeed) {
            this.highSpeed = this.currentSensors.getSpeed();
        }

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
        if (Math.abs(this.currentSensors.getTrackPosition()) >= TRACK_LIMIT) {
            this.offTrack = true;

            this.steerControlSystem.lastUpdate(this.actionSteer, (-10.0));

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

            this.clutch = (double) DrivingInstructor.clutching(this.currentSensors, (float) this.clutch, getStage());

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
        double steer;
        if (this.tics % 5 == 0) {
            this.previousSteerState = this.currentSteerState;
            this.currentSteerState = SteerControl.evaluateSteerState(this.currentSensors);
            this.steerReward = SteerControl.calculateReward(this.previousSensors, this.currentSensors);
            this.actionSteer = (SteerControl.Actions) this.steerControlSystem.update(
                    this.previousSteerState,
                    this.currentSteerState,
                    this.actionSteer,
                    this.steerReward
            );
            steer = SteerControl.steerAction2Double(this.actionSteer);
            System.out.println();
            System.out.println("Epochs: " + this.epochs + "\t"
                    + "Tics: " + this.tics + "\t"
                    + "State: " + this.currentSteerState + "\t"
                    + "Action: " + this.actionSteer + "\t"
                    + "Reward: " + this.steerReward
            );
            System.out.println();
        } else {
            steer = SteerControl.steerAction2Double(this.actionSteer);
        }
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
        if (sensors.getSpeed() < LIMITER_SPEED) {
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
        previousSteerState = SteerControl.States.STATE_9;
        currentSteerState = SteerControl.States.STATE_9;
        actionSteer = SteerControl.Actions.TURN_C;
        steerReward = 0;

        if (this.timeOut) {
            System.out.println("Time out!!!");
        }

        if (this.completeLap) {
            this.completeLaps++;
            System.out.println("Complete lap!");
        }
        if (this.offTrack) {
            System.out.println("Out of track!");
        }

        String newResults = this.generateStatistics();
        this.steerControlSystem.saveQTableAndStatistics(newResults);
        this.steerControlSystem.updateParams();

        tics = 0;
        epochs++;
        laps = -1;
        distanceRaced = 0;
        highSpeed = 0;

        stuck = 0;
        clutch = 0;
        completeLap = false;
        offTrack = false;
        timeOut = false;

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
     * Generates a string containing the statistics of the driver.
     *
     * @return the generated statistics string
     */
    private String generateStatistics() {
        return getTrackName() + SEPARATOR + this.epochs + SEPARATOR + this.tics + SEPARATOR + (int) (this.distanceRaced) + SEPARATOR + (int) (this.highSpeed) + SEPARATOR + this.completeLaps + SEPARATOR + Constants.MAX_EPOCHS;
    }
}