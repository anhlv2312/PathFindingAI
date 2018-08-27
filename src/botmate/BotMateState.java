package botmate;

import common.State;
import common.StateCostPair;
import problem.Box;
import problem.MovingBox;
import problem.MovingObstacle;
import problem.RobotConfig;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;


public class BotMateState implements State {

    /**
     * The robot configuration
     */
    private RobotConfig robotConfig;

    /**
     * A list of moving boxes and obstacles
     */
    private List<Box> movingBoxes;
    private List<Box> movingObstacles;

    /**
     * Create an problem state from an RobotConfig and list of moving objects
     *
     * @param robotConfig     the robot configuration
     * @param movingBoxes     a list of moving boxes
     * @param movingObstacles a list of moving obstacles
     */
    public BotMateState(RobotConfig robotConfig, List<Box> movingBoxes, List<Box> movingObstacles) {
        this.movingBoxes = new ArrayList<>();
        this.movingObstacles = new ArrayList<>();

        this.robotConfig = new RobotConfig(robotConfig.getPos(), robotConfig.getOrientation());
        //todo: Deep Copy of moving boxes;
        for (Box movingBox: movingBoxes) {
            this.movingBoxes.add(new MovingBox(movingBox.getPos(), movingBox.getWidth()));
        }
        //todo: Deep Copy of moving obstacle;
        for (Box movingObstacle: movingObstacles) {
            this.movingBoxes.add(new MovingObstacle(movingObstacle.getPos(), movingObstacle.getWidth()));
        }
    }

    public BotMateState(BotMateState previousState) {
        this.movingBoxes = new ArrayList<>();
        this.movingObstacles = new ArrayList<>();

        this.robotConfig = new RobotConfig(previousState.robotConfig.getPos(), previousState.robotConfig.getOrientation());
        for (Box movingBox: previousState.getMovingBoxes()) {
            this.movingBoxes.add(new MovingBox(movingBox.getPos(), movingBox.getWidth()));
        }
        for (Box movingObstacle: previousState.getMovingObstacles()) {
            this.movingBoxes.add(new MovingObstacle(movingObstacle.getPos(), movingObstacle.getWidth()));
        }
    }

    /**
     * Returns the initial robot config
     **/
    public RobotConfig getRobotConfig() {
        return robotConfig;
    }

    /**
     * Returns a list of moving boxes
     **/
    public List<Box> getMovingBoxes() {
        return movingBoxes;
    }

    /**
     * Returns a list of moving obstacles
     **/
    public List<Box> getMovingObstacles() {
        return movingObstacles;
    }

    public boolean isConnectedWith(BotMateState nextState) {



        //todo: check if the robot and moving object can move from this state to the next state

        // draw boundary of the current moving objects (this.active and nextState.active)
        // draw a rectangle that cover both of them,
        // check collision with any other obstacle

        return false;
    }


    /**
     * Return a list of all states reachable from this state.
     *
     * @return list of successors
     */
    @Override
    public List<StateCostPair> getSuccessors(State goal) {
        List<StateCostPair> successors = new ArrayList<StateCostPair>();



        // First get some of the samples (which are close to the current state)

        // Try to connect them
        // if is connecteed, calculate cost and heuristic to the goal using manhatan and add to succssors
        // then return successor

//        for (Point2D sample : this.) {
//            if (isConnectedWith(sample)) {
//                successors.add(new StateCostPair(moveToNewPosition(sample), BotMateUtility.calculateDistance(this.robotConfig.getPos(), sample) + heuristic(goal)));
//            }
//        }

        return successors;
    }

    /**
     * Check if this state is equivalent to state s
     *
     * @param s state to check equality with
     * @return true if all tiles are in the same position
     */
    private boolean isDone(BotMateState s) {

        return this.heuristic(s) <= 0.0000001;

    }


    /**
     * An estimation of cost from current state to s. The number of different cells divide by 2 which is admissible.
     *
     * @param s
     * @return a double number
     */
    @Override
    public Double heuristic(State s) {
        if (s instanceof BotMateState){
            BotMateState state = (BotMateState) s;
            return BotMateUtility.calculateDistance(this.getMovingBoxes().get(0).pos, state.getMovingBoxes().get(0).pos);
        } else {
            return 0.0;
        }
        //todo: use manhatan distance to calculate the distance currentBox of its goal
    }

    /**
     * Clone the BotMateState and move the bot to new position
     *
     * @param position which is connected with the robot currentposition
     * @return BotMateState
     */
    public BotMateState moveRobot(Point2D position, double orientation) {
        RobotConfig robotConfig = new RobotConfig(position, orientation);
        return new BotMateState(robotConfig, this.movingBoxes, this.movingObstacles);
    }

    public BotMateState moveBoxToPosition(int boxIndex, Point2D position) {
        BotMateState newState = new BotMateState(this);
        Box box = newState.getMovingBoxes().get(boxIndex);
        box.getPos().setLocation(position.getX() + box.getWidth()/2, position.getY() + box.getWidth()/2);
        return newState;
    }


    @Override
    public List<StateCostPair> getSuccessors() {
        return null;
    }

    @Override
    public boolean equals(State s) {
        BotMateState state;
        if (s instanceof BotMateState){
            state = (BotMateState) s;
        } else {
            return false;
        }

        // Only compare movingBoxes
        for (int i = 0; i < state.movingBoxes.size(); i++) {
            if (!state.movingBoxes.get(i).getPos().equals(this.movingBoxes.get(i).getPos())) {
                return false;
            }
        }

        return true;
    }

    @Override
    public String outputString() {
        StringBuilder output = new StringBuilder();

        output.append(this.robotConfig.getPos().getX()).append(" ");
        output.append(this.robotConfig.getPos().getY()).append(" ");
        output.append(this.robotConfig.getOrientation()).append(" ");

        for (Box box: this.movingBoxes) {
            // This is the position of the bottom left conner
            output.append(box.getPos().getX()).append(" ");
            output.append(box.getPos().getY()).append(" ");
        }

        for (Box box: this.movingObstacles) {
            // This is the position of the bottom left conner
            output.append(box.getPos().getX()).append(" ");
            output.append(box.getPos().getY()).append(" ");
        }

        return output.toString();
    }

}
