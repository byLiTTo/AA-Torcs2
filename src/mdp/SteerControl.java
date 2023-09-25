package mdp;

import torcs.Constants;
import torcs.SensorModel;

public class SteerControl {

    public static States evaluateSteerState(SensorModel currentSensors) {
        double maxValue = -1;
        int index = -1;
        String state = "STATE_";

        double[] distances = currentSensors.getTrackEdgeSensors();
        for (int i = 0; i < distances.length; i++) {
            if (maxValue < distances[i]) {
                maxValue = distances[i];
                index = i;
            }
        }
        if (index != -1) {
            state = state + index;
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
                return Constants.round((Constants.toRadians(angleDegree)), 4);
            }
            case "R": {
                String angleName = actionSteer.name().replaceAll("TURN_R_", "").replace("_", ".");
                double angleDegree = Double.parseDouble(angleName);
                return -Constants.round((Constants.toRadians(angleDegree)), 4);
            }
        }
        return 0.0;
    }

    public static double calculateReward(SensorModel previous, SensorModel current) {
        double reward = 0.0;
////        double speedDiference = Math.abs(previous.getSpeed() - current.getSpeed());
////        if (speedDiference >= 0 && speedDiference <= 5) {
////            reward += 100.0;
////        }
//
//        // Reading of sensor at +5 degrees w.r.t. car axis
//        float rxSensor = (float) previous.getTrackEdgeSensors()[10];
//        // Reading of sensor parallel to car axis
//        float sensorSensor = (float) previous.getTrackEdgeSensors()[9];
//        // Reading of sensor at -5 degrees w.r.t. car axis
//        float sxSensor = (float) previous.getTrackEdgeSensors()[8];
//
//        // Reading of sensor at +5 degrees w.r.t. car axis
//        float rxSensorCurrent = (float) current.getTrackEdgeSensors()[10];
//        // Reading of sensor parallel to car axis
//        float sensorSensorCurrent = (float) current.getTrackEdgeSensors()[9];
//        // Reading of sensor at -5 degrees w.r.t. car axis
//        float sxSensorCurrent = (float) current.getTrackEdgeSensors()[8];
//
//        // Approaching a turn on the left
//        if (rxSensor < sxSensor) {
//            reward += (sensorSensorCurrent - sensorSensor) * 100.0;
//        }
//        // Approaching a turn on the right
//        else if (rxSensor > sxSensor) {
//            reward += (sensorSensorCurrent - sensorSensor) * 100.0;
//        }
//
//        if (Constants.round(sensorSensor, 0) == 200 && Constants.round(sensorSensorCurrent, 0) == 200) {
//            reward += 100.0;
//        }
//
        reward += 10 * (1 - Math.abs(current.getTrackPosition()));


        return reward;

    }

    public enum Actions {
        TURN_L_45, TURN_L_22_5, TURN_L_11_25, TURN_L_5_6, TURN_L_2_8, TURN_L_1_4, TURN_L_0_7,

        TURN_C,
        TURN_R_0_7, TURN_R_1_4, TURN_R_2_8, TURN_R_5_6, TURN_R_11_25, TURN_R_22_5, TURN_R_45
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
