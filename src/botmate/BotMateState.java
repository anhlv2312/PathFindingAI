package botmate;

import common.State;
import problem.Box;
import problem.RobotConfig;
import java.util.List;

public class BotMateState {

    /** The robot configuration */
    private RobotConfig robotConfig;

    /** A list of moving boxes and obstacles */
    private List<Box> movingBoxes;
    private List<Box> movingObstacles;

    /**
     * Create an problem state from an RobotConfig and list of moving objects
     * @param robotConfig the robot configuration
     * @param movingBoxes a list of moving boxes
     * @param movingObstacles a list of moving obstacles
     */
    public BotMateState(RobotConfig robotConfig , List<Box> movingBoxes, List<Box> movingObstacles) {
        this.robotConfig = new RobotConfig(robotConfig.getPos(), robotConfig.getOrientation());
        //todo: Deep Copy of moving boxes;
        this.movingBoxes = null;
        //todo: Deep Copy of moving obstacle;
        this.movingObstacles = null;
    }

    /** Returns the initial robot config **/
    public RobotConfig getRobotConfig() { return robotConfig; }

    /** Returns a list of moving boxes **/
    public List<Box> getMovingBoxes() { return movingBoxes; }

    /** Returns a list of moving obstacles **/
    public List<Box> getMovingObstacles() { return movingObstacles; }

    public boolean isConnectedWith(BotMateState nextState) {
        //todo: check if the robot and moving object can move from this state to the next state
        return false;
    }



}
