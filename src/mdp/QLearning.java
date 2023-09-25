package mdp;

import torcs.Constants;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.*;

import static torcs.Constants.*;

/**
 * The QLearning class implements the Q-learning algorithm for reinforcement learning in the TORCS environment.
 * It includes methods for creating and loading the Q-table, updating the Q-values, selecting actions, and saving
 * statistics.
 */
public class QLearning {

    private final HashMap<String, HashMap<String, Double>> qTable;
    private List<Object> possibleActions = null;
    private Object lastState;
    private double epsilon;
    private double epsilonDecay;
    private double learningRate;
    private double learningRateDecay;
    private int epochs;
    private Random random;
    private ControlSystems system;
    private String qTablePath;
    private List<Object> stateVisited = null;

    /**
     * Constructs a QLearning object for the specified control system.
     *
     * @param system The control system (Steering, Acceleration, or Gear).
     */
    public QLearning(ControlSystems system) {
        this.qTable = new HashMap<>();
        this.epochs = 0;

        this.random = new Random(System.currentTimeMillis());

        this.stateVisited = new ArrayList<>();

        this.system = system;
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM:
                this.possibleActions = Arrays.asList(SteerControl.Actions.values());
                this.qTablePath = STEER_Q_TABLE_PATH;
                break;
            case ACCELERATION_CONTROL_SYSTEM:
                this.possibleActions = Arrays.asList(AccelControl.Actions.values());
                this.qTablePath = ACCEL_Q_TABLE_PATH;
                break;
            case GEAR_CONTROL_SYSTEM:
                this.possibleActions = Arrays.asList(GearControl.Actions.values());
                this.qTablePath = GEAR_Q_TABLE_PATH;
                break;
        }
        File f = new File(this.qTablePath);
        if (!f.exists())
            this.createQTable();
        else
            this.loadQTable();

    }

    /**
     * Constructs a QLearning object for the specified control system with a maximum number of epochs.
     *
     * @param system    The control system (Steering, Acceleration, or Gear).
     * @param maxEpochs The maximum number of epochs.
     */
    public QLearning(ControlSystems system, int maxEpochs, int rangeEpochs) {
        this.qTable = new HashMap<>();
        this.epsilon = INITIAL_EPSILON;
//        this.epsilonDecay = Math.pow((FINAL_EPSILON / INITIAL_EPSILON), (1.0 / RANGE_EPOCHS));
        this.epsilonDecay = INITIAL_EPSILON / (double) (RANGE_EPOCHS);
        this.learningRate = INITIAL_LEARNING_RATE;
        this.learningRateDecay = Math.pow((FINAL_LEARNING_RATE / INITIAL_LEARNING_RATE), (1.0 / MAX_EPOCHS));
//        this.learningRateDecay = INITIAL_LEARNING_RATE / (double) (MAX_EPOCHS);
        this.epochs = 0;

        this.stateVisited = new ArrayList<>();

        this.random = new Random(System.currentTimeMillis());
        this.system = system;
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM:
                this.possibleActions = Arrays.asList(SteerControl.Actions.values());
                this.qTablePath = STEER_Q_TABLE_PATH;
                break;
            case ACCELERATION_CONTROL_SYSTEM:
                this.possibleActions = Arrays.asList(AccelControl.Actions.values());
                this.qTablePath = ACCEL_Q_TABLE_PATH;
                break;
            case GEAR_CONTROL_SYSTEM:
                this.possibleActions = Arrays.asList(GearControl.Actions.values());
                this.qTablePath = GEAR_Q_TABLE_PATH;
                break;
        }
        File f = new File(this.qTablePath);
        if (!f.exists())
            this.createQTable();
        else
            this.loadQTable();
    }

    public static double inverseSigmoid(double x) {
        return 1 / (1 + Math.exp(x));
    }

    public static double getFactorAprendizaje_Exploracion(double inicio, double fin, int actual, double delay, double end) {
        double min_x = delay;
        double max_x = end;

        double n_min_x = -6;
        double n_max_x = 6;

        double x = ((actual - min_x) / (max_x - min_x)) * (n_max_x - n_min_x) + n_min_x;

        double y = inverseSigmoid(x);

        double min_y = 0;
        double max_y = 1;

        double n_min_y = fin;
        double n_max_y = inicio;

        double ny = ((y - min_y) / (max_y - min_y)) * (n_max_y - n_min_y) + n_min_y;

        return ny;
    }


    /**
     * Creates the Q-table for the specified control system.
     */
    private void createQTable() {
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM:
                for (SteerControl.States state : SteerControl.States.values()) {
                    HashMap<String, Double> row = new HashMap<>();
                    for (Object tmp : this.possibleActions) {
                        SteerControl.Actions action = (SteerControl.Actions) tmp;
                        row.put(action.name(), 0.0);
                    }
                    qTable.put(state.name(), row);
                }
                break;
            case ACCELERATION_CONTROL_SYSTEM:
                for (AccelControl.States state : AccelControl.States.values()) {
                    HashMap<String, Double> row = new HashMap<>();
                    for (Object tmp : this.possibleActions) {
                        AccelControl.Actions action = (AccelControl.Actions) tmp;
                        row.put(action.name(), 0.0);
                    }
                    qTable.put(state.name(), row);
                }
                break;
            case GEAR_CONTROL_SYSTEM:
                for (GearControl.States state : GearControl.States.values()) {
                    HashMap<String, Double> row = new HashMap<>();
                    for (Object tmp : this.possibleActions) {
                        GearControl.Actions action = (GearControl.Actions) tmp;
                        row.put(action.name(), 0.0);
                    }
                    qTable.put(state.name(), row);
                }
                break;
        }
    }

    /**
     * Loads the Q-table from a file for the specified control system.
     */
    private void loadQTable() {
        qTable.clear();
        try (Scanner file = new Scanner(new File(this.qTablePath))) {
            String[] rowLabels = file.nextLine().split(SEPARATOR);
            while (file.hasNextLine()) {
                String[] row = file.nextLine().split(SEPARATOR);
                String state = row[0];
                HashMap<String, Double> rowHash = new HashMap<>();
                for (int i = 1; i < row.length; i++) {
                    rowHash.put(rowLabels[i], Double.parseDouble(row[i]));
                }
                this.qTable.put(state, rowHash);
            }
        } catch (FileNotFoundException e) {
            System.out.println("ERROR!!! -> Could not load tablaQ from .csv file...");
            e.printStackTrace();
        }
    }

    /**
     * Saves the Q-table to a file.
     */
    public void saveTable() {
        try (PrintWriter file = new PrintWriter(this.qTablePath)) {
            file.write(" Q-TABLE ");
            file.write(SEPARATOR);
            switch (this.system) {
                case STEERING_CONTROL_SYSTEM:
                    for (Object tmp : this.possibleActions) {
                        SteerControl.Actions action = (SteerControl.Actions) tmp;
                        file.write(action.name());
                        file.write(SEPARATOR);
                    }
                    file.write("\n");
                    for (SteerControl.States state : SteerControl.States.values()) {
                        file.write(state.name());
                        file.write(SEPARATOR);
                        for (Object tmp : possibleActions) {
                            SteerControl.Actions action = (SteerControl.Actions) tmp;
                            String value = String.valueOf(this.qTable.get(state.name()).get(action.name()));
                            file.write(value);
                            file.write(SEPARATOR);
                        }
                        file.write("\n");
                    }
                    break;
                case ACCELERATION_CONTROL_SYSTEM:
                    for (Object tmp : this.possibleActions) {
                        AccelControl.Actions action = (AccelControl.Actions) tmp;
                        file.write(action.name());
                        file.write(SEPARATOR);
                    }
                    file.write("\n");
                    for (AccelControl.States state : AccelControl.States.values()) {
                        file.write(state.name());
                        file.write(SEPARATOR);
                        for (Object tmp : possibleActions) {
                            AccelControl.Actions action = (AccelControl.Actions) tmp;
                            String value = String.valueOf(this.qTable.get(state.name()).get(action.name()));
                            file.write(value);
                            file.write(SEPARATOR);
                        }
                        file.write("\n");
                    }
                    break;
                case GEAR_CONTROL_SYSTEM:
                    for (Object tmp : this.possibleActions) {
                        GearControl.Actions action = (GearControl.Actions) tmp;
                        file.write(action.name());
                        file.write(SEPARATOR);
                    }
                    file.write("\n");
                    for (GearControl.States state : GearControl.States.values()) {
                        file.write(state.name());
                        file.write(SEPARATOR);
                        for (Object tmp : possibleActions) {
                            GearControl.Actions action = (GearControl.Actions) tmp;
                            String value = String.valueOf(this.qTable.get(state.name()).get(action.name()));
                            file.write(value);
                            file.write(SEPARATOR);
                        }
                        file.write("\n");
                    }
                    break;
            }
        } catch (FileNotFoundException e) {
            System.out.println("ERROR!!! -> Could not save tableQ in .csv file...");
            e.printStackTrace();
        }
    }

    /**
     * Updates the Q-value for the specified state-action pair based on the reward received.
     *
     * @param lastState       The last state.
     * @param currentState    The current state.
     * @param actionPerformed The action performed.
     * @param reward          The reward received.
     * @return The next action to take.
     */
    public Object update(Object lastState, Object currentState, Object actionPerformed, double reward) {
        if (!this.stateVisited.contains(currentState)) {
            this.stateVisited.add(currentState);
        }
        this.lastState = lastState;
        if (lastState != null) {
            double newQValue = this.getQValue(lastState, actionPerformed) + this.learningRate * (reward + DISCOUNT_FACTOR
                    * this.getMaxQValue(lastState));
            this.setQValue(lastState, actionPerformed, (Constants.round(newQValue, 8)));
        }
        return nextAction(currentState);
    }

    /**
     * Updates the Q-value for the last state-action pair based on the reward received.
     *
     * @param lastAction The last action.
     * @param reward     The reward received.
     */
    public void lastUpdate(Object lastAction, double reward) {
        if (this.lastState != null) {
            double newQValue = (1 - this.learningRate) * this.getQValue(this.lastState, lastAction) + this.learningRate
                    * (reward + DISCOUNT_FACTOR * this.getMaxQValue(this.lastState));
            this.setQValue(this.lastState, lastAction, (Constants.round(newQValue, 8)));
        }
    }

    /**
     * Returns the Q-value for the specified state-action pair.
     *
     * @param stateO  The state.
     * @param actionO The action.
     * @return The Q-value.
     */
    private double getQValue(Object stateO, Object actionO) {
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM:
                SteerControl.States state = (SteerControl.States) stateO;
                SteerControl.Actions action = (SteerControl.Actions) actionO;
                return this.qTable.get(state.name()).get(action.name());
            case ACCELERATION_CONTROL_SYSTEM:
                AccelControl.States accelState = (AccelControl.States) stateO;
                AccelControl.Actions accelAction = (AccelControl.Actions) actionO;
                return this.qTable.get(accelState.name()).get(accelAction.name());
            case GEAR_CONTROL_SYSTEM:
                GearControl.States gearState = (GearControl.States) stateO;
                GearControl.Actions gearAction = (GearControl.Actions) actionO;
                return this.qTable.get(gearState.name()).get(gearAction.name());
        }
        return 0.0;
    }

    /**
     * Sets the Q
     * <p>
     * -value for the specified state-action pair.
     *
     * @param stateO  The state.
     * @param actionO The action.
     * @param value   The Q-value.
     */
    private void setQValue(Object stateO, Object actionO, double value) {
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM:
                SteerControl.States state = (SteerControl.States) stateO;
                SteerControl.Actions action = (SteerControl.Actions) actionO;
                HashMap<String, Double> row = this.qTable.get(state.name());
                row.replace(action.name(), value);
                this.qTable.replace(state.name(), row);
                break;
            case ACCELERATION_CONTROL_SYSTEM:
                AccelControl.States accelState = (AccelControl.States) stateO;
                AccelControl.Actions accelAction = (AccelControl.Actions) actionO;
                HashMap<String, Double> accelRow = this.qTable.get(accelState.name());
                accelRow.replace(accelAction.name(), value);
                this.qTable.replace(accelState.name(), accelRow);
                break;
            case GEAR_CONTROL_SYSTEM:
                GearControl.States gearState = (GearControl.States) stateO;
                GearControl.Actions gearAction = (GearControl.Actions) actionO;
                HashMap<String, Double> gearRow = this.qTable.get(gearState.name());
                gearRow.replace(gearAction.name(), value);
                this.qTable.replace(gearState.name(), gearRow);
                break;
        }
    }

    /**
     * Returns the maximum Q-value for the specified state.
     *
     * @param stateO The state.
     * @return The maximum Q-value.
     */
    private double getMaxQValue(Object stateO) {
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM:
                SteerControl.States state = (SteerControl.States) stateO;
                double maxValue = -Double.MAX_VALUE;
                HashMap<String, Double> values = qTable.get(state.name());
                ArrayList<SteerControl.Actions> candidates = new ArrayList<>();
                for (Object tmp : this.possibleActions) {
                    SteerControl.Actions action = (SteerControl.Actions) tmp;
                    double value = values.get(action.name());
                    if (maxValue < value) {
                        maxValue = value;
                        candidates.clear();
                        candidates.add(action);
                    } else if (maxValue == value) {
                        candidates.add(action);
                    }
                }
                int index = random.nextInt(candidates.size());
                SteerControl.Actions selected = candidates.get(index);
                return values.get(selected.name());
            case ACCELERATION_CONTROL_SYSTEM:
                AccelControl.States accelState = (AccelControl.States) stateO;
                double accelMaxValue = -Double.MAX_VALUE;
                HashMap<String, Double> accelValues = qTable.get(accelState.name());
                ArrayList<AccelControl.Actions> accelCandidates = new ArrayList<>();
                for (Object tmp : this.possibleActions) {
                    AccelControl.Actions action = (AccelControl.Actions) tmp;
                    double value = accelValues.get(action.name());
                    if (accelMaxValue < value) {
                        accelMaxValue = value;
                        accelCandidates.clear();
                        accelCandidates.add(action);
                    } else if (accelMaxValue == value) {
                        accelCandidates.add(action);
                    }
                }
                int accelIndex = random.nextInt(accelCandidates.size());
                AccelControl.Actions accelSelected = accelCandidates.get(accelIndex);
                return accelValues.get(accelSelected.name());
            case GEAR_CONTROL_SYSTEM:
                GearControl.States gearState = (GearControl.States) stateO;
                double gearMaxValue = -Double.MAX_VALUE;
                HashMap<String, Double> gearValues = qTable.get(gearState.name());
                ArrayList<GearControl.Actions> gearCandidates = new ArrayList<>();
                for (Object tmp : this.possibleActions) {
                    GearControl.Actions action = (GearControl.Actions) tmp;
                    double value = gearValues.get(action.name());
                    if (gearMaxValue < value) {
                        gearMaxValue = value;
                        gearCandidates.clear();
                        gearCandidates.add(action);
                    } else if (gearMaxValue == value) {
                        gearCandidates.add(action);
                    }
                }
                int gearIndex = random.nextInt(gearCandidates.size());
                GearControl.Actions gearSelected = gearCandidates.get(gearIndex);
                return gearValues.get(gearSelected.name());
        }
        return 0.0;
    }

    /**
     * Returns the next action to take based on the current state.
     *
     * @param state The current state.
     * @return The next action.
     */
    public Object nextAction(Object state) {
        double probability = random.nextDouble();
        if (probability < epsilon) {
            return this.getRandomAction();
        } else {
            return this.getBestAction(state);
        }
    }

    /**
     * Returns a random action.
     *
     * @return A random action.
     */
    private Object getRandomAction() {
        int index = random.nextInt(this.possibleActions.size());
        return this.possibleActions.get(index);
    }

    /**
     * Returns the best action to take based on the current state.
     *
     * @param stateO The current state.
     * @return The best action.
     */
    private Object getBestAction(Object stateO) {
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM:
                SteerControl.States state = (SteerControl.States) stateO;
                double maxValue = -Double.MAX_VALUE;
                HashMap<String, Double> values = qTable.get(state.name());
                ArrayList<SteerControl.Actions> candidates = new ArrayList<>();
                for (Object tmp : this.possibleActions) {
                    SteerControl.Actions action = (SteerControl.Actions) tmp;
                    double value = values.get(action.name());
                    if (maxValue < value) {
                        maxValue = value;
                        candidates.clear();
                        candidates.add(action);
                    } else if (maxValue == value) {
                        candidates.add(action);
                    }
                }
                int index = random.nextInt(candidates.size());
                return candidates.get(index);
            case ACCELERATION_CONTROL_SYSTEM:
                AccelControl.States accelState = (AccelControl.States) stateO;
                double accelMaxValue = -Double.MAX_VALUE;
                HashMap<String, Double> accelValues = qTable.get(accelState.name());
                ArrayList<AccelControl.Actions> accelCandidates = new ArrayList<>();
                for (Object tmp : this.possibleActions) {
                    AccelControl.Actions action = (AccelControl.Actions) tmp;
                    double value = accelValues.get(action.name());
                    if (accelMaxValue < value) {
                        accelMaxValue = value;
                        accelCandidates.clear();
                        accelCandidates.add(action);
                    } else if (accelMaxValue == value) {
                        accelCandidates.add(action);
                    }
                }
                int accelIndex = random.nextInt(accelCandidates.size());
                return accelCandidates.get(accelIndex);
            case GEAR_CONTROL_SYSTEM:
                GearControl.States gearState = (GearControl.States) stateO;
                double gearMaxValue = -Double.MAX_VALUE;
                HashMap<String, Double> gearValues = qTable.get(gearState.name());
                ArrayList<GearControl.Actions> gearCandidates = new ArrayList<>();
                for (Object tmp : this.possibleActions) {
                    GearControl.Actions action = (GearControl.Actions) tmp;
                    double value = gearValues.get(action.name());
                    if (gearMaxValue < value) {
                        gearMaxValue = value;
                        gearCandidates.clear();
                        gearCandidates.add(action);
                    } else if (gearMaxValue == value) {
                        gearCandidates.add(action);
                    }
                }
                int gearIndex = random.nextInt(gearCandidates.size());
                return gearCandidates.get(gearIndex);
        }
        return null;
    }

    /**
     * Returns the best action to take based on the current state without any randomness.
     *
     * @param stateO The current state.
     * @return The best action.
     */
    public Object nextOnlyBestAction(Object stateO) {
        switch (this.system) {
            case STEERING_CONTROL_SYSTEM:
                SteerControl.States state = (SteerControl.States) stateO;
                double maxValue = -Double.MAX_VALUE;
                HashMap<String, Double> values = qTable.get(state.name());
                Object theBest = this.getRandomAction();
                for (Object tmp : this.possibleActions) {
                    SteerControl.Actions action = (SteerControl.Actions) tmp;
                    double value = values.get(action.name());
                    if (maxValue < value) {
                        maxValue = value;
                        theBest = action;
                    }
                }
                return theBest;
            case ACCELERATION_CONTROL_SYSTEM:
                AccelControl.States accelState = (AccelControl.States) stateO;
                double accelMaxValue = -Double.MAX_VALUE;
                HashMap<String, Double> accelValues = qTable.get(accelState.name());
                Object accelTheBest = this.getRandomAction();
                for (Object tmp : this.possibleActions) {
                    AccelControl.Actions action = (AccelControl.Actions) tmp;
                    double value = accelValues.get(action.name());
                    if (accelMaxValue < value) {
                        accelMaxValue = value;
                        accelTheBest = action;
                    }
                }
                return accelTheBest;
            case GEAR_CONTROL_SYSTEM:
                GearControl.States gearState = (GearControl.States) stateO;
                double gearMaxValue = -Double.MAX_VALUE;
                HashMap<String, Double> gearValues = qTable.get(gearState.name());
                Object gearTheBest = this.getRandomAction();
                for (Object tmp : this.possibleActions) {
                    GearControl.Actions action = (GearControl.Actions) tmp;
                    double value = gearValues.get(action.name());
                    if (gearMaxValue < value) {
                        gearMaxValue = value;
                        gearTheBest = action;
                    }
                }
                return gearTheBest;
        }
        return null;
    }

    /**
     * Saves the statistics to a file.
     *
     * @param newResults The new results to be added to the statistics.
     */
    public void saveStatistics(String newResults) {
        newResults = newResults + SEPARATOR + this.epsilon + SEPARATOR + this.learningRate + SEPARATOR + this.stateVisited.size();
        this.saveStatistics(STATISTICS_TEST_PATH, newResults);
    }

    /**
     * Saves the Q-table and statistics to files.
     *
     * @param newResults The new results to be added to the statistics.
     */
    public void saveQTableAndStatistics(String newResults) {
        this.epochs++;
        this.saveTable();
        newResults = newResults + SEPARATOR + this.epsilon + SEPARATOR + this.learningRate + SEPARATOR + this.stateVisited.size();
        this.saveStatistics(STATISTICS_TRAIN_PATH, newResults);
    }

    /**
     * Saves the statistics to a file.
     *
     * @param filePath   The file path.
     * @param newResults The new results to be added to the statistics.
     */
    private void saveStatistics(String filePath, String newResults) {
        List<String> content = new ArrayList<>();
        try (Scanner file = new Scanner(new File(filePath))) {
            while (file.hasNextLine()) {
                content.add(file.nextLine());
            }
        } catch (FileNotFoundException e) {
            System.out.println("ERROR!!! -> Could not load statistics from .csv file...");
            e.printStackTrace();
        }
        try (PrintWriter file = new PrintWriter((filePath))) {
            for (String line : content) {
                file.write(line + "\n");
            }
            file.write(newResults + "\n");
        } catch (FileNotFoundException e) {
            System.out.println("ERROR!!! -> Could not save statistics in .csv file...");
            e.printStackTrace();
        }
    }

    /**
     * Decreases the value of epsilon.
     */
    public void updateParams() {
        this.epsilon = getFactorAprendizaje_Exploracion(INITIAL_EPSILON, FINAL_EPSILON, this.epochs, 0, RANGE_EPOCHS);
//        this.learningRate = getFactorAprendizaje_Exploracion(INITIAL_LEARNING_RATE, FINAL_LEARNING_RATE, this.epochs, 0, MAX_EPOCHS);
    }

}