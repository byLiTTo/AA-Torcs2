package mdp;

public class AccelControl {

    public enum Actions {
        FULL_THROTTLE,
        ACCELERATE,
        HANDBRAKE,
        KEEP_ROLLING,
    }

    /**
     * Enumeration of possible acceleration states.
     */
    public enum States {
        STRAIGHT_LINE,
        IN_CURVE_SHOULD_ACCEL,
        IN_CURVE_SHOULD_HAND_BRAKE,
        IN_CURVE_SHOULD_ALLOW_ROLL
    }
}
