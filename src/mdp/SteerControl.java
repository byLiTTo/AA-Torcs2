package mdp;

import torcs.Constants;
import torcs.SensorModel;

public class SteerControl {

    private static final double CENTERED_RANGE = 0.0;
    private static final double TILTED_RANGE = 0.0;
    private static final double BORDER_RANGE = 0.0;

    public static States evaluateSteerState(SensorModel currentSensors) {
        double maxValue = -1;
        int index = -1;
        String state = "STATE_";

        double[] distances = currentSensors.getTrackEdgeSensors();
        for (int i = 0; i < distances.length; i++) {
            if (maxValue < distances[i]) {
                maxValue = distances[i];
                index = i;
                index = i;
            }
        }
        if (index != -1) {
            state = state + index;
//            double range = Math.abs(currentSensors.getTrackPosition() * 0.5);
//            if (range <= CENTERED_RANGE) {
//                state += "CENTERED";
//            } else if (range <= TILTED_RANGE) {
//                state += "TILTED";
//            } else {
//                state += "BORDER";
//            }
            System.out.println(state);
            return States.valueOf(state);
        }
        return States.STATE_9;
    }

    public static double steerAction2Double(Actions actionSteer) {
        String[] params = actionSteer.name().replaceAll("TURN_", "").split("_");
        switch (params[0]) {
            case "C":
                return 0.0;
            case "L": {
                String angleName = actionSteer.name().replaceAll("TURN_L_", "").replace("_", ".");
                double angleDegree = Double.parseDouble(angleName);
                return Constants.round((Constants.toRadians(angleDegree) / Constants.radian), 4);
            }
            case "R": {
                String angleName = actionSteer.name().replaceAll("TURN_R_", "").replace("_", ".");
                double angleDegree = Double.parseDouble(angleName);
                return -Constants.round((Constants.toRadians(angleDegree) / Constants.radian), 4);
            }
        }
        return 0.0;
    }

    public static double calculateReward(SensorModel previous, SensorModel current) {
        double reward = 0.0;

        if (previous.getDistanceRaced() + 5 < current.getDistanceRaced()) {
            reward += 100;
        }

        if (current.getTrackEdgeSensors()[0] >= 10) {
            reward += 100;
        }

        if (current.getTrackEdgeSensors()[18] >= 10) {
            reward += 100;
        }


        return reward;
    }

    public enum Actions {
        TURN_L_45, TURN_L_30, TURN_L_25, TURN_L_20, TURN_L_15, TURN_L_10, TURN_L_7_5, TURN_L_5, TURN_L_2_5,
        TURN_C,
        TURN_R_2_5, TURN_R_5, TURN_R_7_5, TURN_R_10, TURN_R_15, TURN_R_20, TURN_R_25, TURN_R_30, TURN_R_45
    }

    /**
     * The steer states.
     */
    public enum States {
        STATE_0,
        STATE_1,
        STATE_2,
        STATE_3,
        STATE_4,
        STATE_5,
        STATE_6,
        STATE_7,
        STATE_8,
        STATE_9,
        STATE_10,
        STATE_11,
        STATE_12,
        STATE_13,
        STATE_14,
        STATE_15,
        STATE_16,
        STATE_17,
        STATE_18,
    }
}
