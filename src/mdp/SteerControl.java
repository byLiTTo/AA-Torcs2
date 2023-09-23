package mdp;

import torcs.SensorModel;

public class SteerControl {

    public static States evaluateSteerState(SensorModel currentSensors) {
        double maxValue = -1;
        int index = -1;

        double[] distances = currentSensors.getTrackEdgeSensors();
        for (int i = 0; i < distances.length; i++) {
            if (maxValue < distances[i]) {
                maxValue = distances[i];
                index = i;
                index = i;
            }
        }

        if (index != -1) {
            System.out.println("Estado: " + States.values()[index].name());
            return States.values()[index];
        }
        return States.CENTER;
    }

    public static double steerAction2Double(Actions actionSteer) {
        String[] params = actionSteer.name().replaceAll("TURN_", "").split("_");
        switch (params[0]) {
            case "C":
                return 0.0;
            case "L": {
                String angleName = actionSteer.name().replaceAll("TURN_L_", "").replace("_", ".");
                return Double.parseDouble(angleName);
            }
            case "R": {
                String angleName = actionSteer.name().replaceAll("TURN_R_", "").replace("_", ".");
                return -(Double.parseDouble(angleName));
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
        TURN_L_1_0, TURN_L_0_9, TURN_L_0_8, TURN_L_0_7, TURN_L_0_6, TURN_L_0_5, TURN_L_0_4, TURN_L_0_3, TURN_L_0_2, TURN_L_0_1,
        TURN_C,
        TURN_R_0_1, TURN_R_0_2, TURN_R_0_3, TURN_R_0_4, TURN_R_0_5, TURN_R_0_6, TURN_R_0_7, TURN_R_0_8, TURN_R_0_9, TURN_R_1_0
    }

    /**
     * The steer states.
     */
    public enum States {
        LEFT_90,
        LEFT_80,
        LEFT_70,
        LEFT_60,
        LEFT_50,
        LEFT_40,
        LEFT_30,
        LEFT_20,
        LEFT_10,
        CENTER,
        RIGHT_10,
        RIGHT_20,
        RIGHT_30,
        RIGHT_40,
        RIGHT_50,
        RIGHT_60,
        RIGHT_70,
        RIGHT_80,
        RIGHT_90
    }
}
