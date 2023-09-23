package mdp;

public class GearControl {

    public enum Actions {
        ACTIVE_LIMITER,
        GEAR_UP,
        GEAR_DOWN,
        KEEP_GEAR
    }

    /**
     * The gear states.
     */
    public enum States {
        NEUTRAL_REVERSE,
        ENOUGH_RPM_UP,
        LOW_RPM,
        REVOLUTIONIZING_ENGINE
    }
}
