package botmate;

import common.State;
import common.StateCostPair;
import problem.Box;
import problem.RobotConfig;

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
    private List<Point2D> listPosition;
    private Box currentBox;
    private BotMateUtility botMateUtility;

    /**
     * Create an problem state from an RobotConfig and list of moving objects
     *
     * @param robotConfig     the robot configuration
     * @param movingBoxes     a list of moving boxes
     * @param movingObstacles a list of moving obstacles
     */
    public BotMateState(RobotConfig robotConfig, List<Box> movingBoxes, List<Box> movingObstacles) {
        this.robotConfig = new RobotConfig(robotConfig.getPos(), robotConfig.getOrientation());
        //todo: Deep Copy of moving boxes;
        this.movingBoxes = null;
        //todo: Deep Copy of moving obstacle;
        this.movingObstacles = null;
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

    public boolean isConnectedWith(Point2D newposition) {
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
    public List<StateCostPair> getSuccessors(State goal) {
        List<StateCostPair> successors = new ArrayList<StateCostPair>();

        // First get some of the samples (which are close to the current state)

        // Try to connect them
        // if is connecteed, calculate cost and heuristic to the goal using manhatan and add to succssors
        // then return successor

        for (Point2D sample : this.listPosition) {
            if (isConnectedWith(sample)) {
                successors.add(new StateCostPair(moveToNewPosition(sample), botMateUtility.calculateDistance(this.robotConfig.getPos(), sample) + heuristic(goal)));
            }
        }

        return successors;
    }

    /**
     * Check if this puzzle state is equivalent to state s
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
    public Double heuristic(BotMateState s) {
        return botMateUtility.calculateDistance(this.currentBox.pos, s.currentBox.pos);
        //todo: use manhatan distance to calculate the distance currentBox of its goal
    }


    /**
     * Clone the BotMatestate and move the bot to new position
     *
     * @param newposition which is connected with the robot currentposition
     * @return BotMateState
     */
    private BotMateState moveToNewPosition(Point2D newposition) {
        RobotConfig newRobotPosition = this.robotConfig;
        newRobotPosition.getPos().setLocation(newposition);
        return new BotMateState(newRobotPosition, this.movingBoxes, this.movingObstacles);
    }

    @Override
    public List<StateCostPair> getSuccessors() {
        return null;
    }

    @Override
    public boolean equals(State s) {
        return false;
    }

    @Override
    public String outputString() {
        return null;
    }

    @Override
    public Double heuristic(State s) {
        return null;
    }
}
