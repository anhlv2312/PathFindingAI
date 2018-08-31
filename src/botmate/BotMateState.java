package botmate;

import common.State;
import common.StateCostPair;
import problem.*;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;


public class BotMateState implements State {

    /**
     * The robot configuration
     */
    private RobotConfig robotConfig;

    /**
     * A list of moving boxes and obstacles
     */

    private int movingBoxIndex;
    private List<Box> movingBoxes;
    private List<Box> movingObstacles;
    private List<StaticObstacle> staticObstacles;

    /**
     * Create an problem state from an RobotConfig and list of moving objects
     *
     * @param robotConfig     the robot configuration
     * @param movingBoxes     a list of moving boxes
     * @param movingObstacles a list of moving obstacles
     */
    public BotMateState(int movingBoxIndex, RobotConfig robotConfig, List<Box> movingBoxes, List<Box> movingObstacles, List<StaticObstacle> staticObstacles) {

        this.movingBoxIndex = movingBoxIndex;
        this.movingBoxes = new ArrayList<>();
        this.movingObstacles = new ArrayList<>();

        this.robotConfig = new RobotConfig(robotConfig.getPos(), robotConfig.getOrientation());
        //Deep Copy of moving boxes;
        for (Box box: movingBoxes) {
            this.movingBoxes.add(new MovingBox(box.getPos(), box.getWidth()));
        }
        //Deep Copy of moving obstacle;
        for (Box box: movingObstacles) {
            this.movingBoxes.add(new MovingObstacle(box.getPos(), box.getWidth()));
        }

        this.staticObstacles = staticObstacles;
    }

    public Box getMovingBox() {
        return this.getMovingBoxes().get(movingBoxIndex);
    }


    public int getMovingBoxIndex() {
        return movingBoxIndex;
    }

    public void setMovingBoxIndex(int movingBoxIndex) {
        this.movingBoxIndex = movingBoxIndex;
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

        Box box1 = this.getMovingBox();
        Box box2 = nextState.getMovingBox();

        Point2D center1 = new Point2D.Double(box1.getPos().getX() + box1.getWidth()/2, box1.getPos().getY() + box1.getWidth()/2);
        Point2D center2 = new Point2D.Double(box2.getPos().getX() + box2.getWidth()/2, box2.getPos().getY() + box2.getWidth()/2);

        Line2D line = new Line2D.Double(center1, center2);

        Rectangle2D boundary = line.getBounds2D();

        boundary =  new Rectangle2D.Double(boundary.getX() - BotMateSolver.MAX_BASE_STEP, boundary.getY() - BotMateSolver.MAX_BASE_STEP,
                boundary.getWidth() + 2 * BotMateSolver.MAX_BASE_STEP, boundary.getHeight() + 2 * BotMateSolver.MAX_BASE_STEP);


        // Check intersect with other moving box
        for (Box box: this.getMovingBoxes()) {
            if (box != this.getMovingBox() && boundary.intersects(box.getRect())) {
                return false;
            }

        }

        // Check intersect with other moving obstacle box
        for (Box box : this.getMovingObstacles()) {
            if (boundary.intersects(box.getRect())) {
                return false;
            }
        }

        // Check intersect with static obstacles
        for (StaticObstacle obstacle : staticObstacles) {
            if (boundary.intersects(obstacle.getRect())) {
                return false;
            } else {

            }
        }
        //todo: check if the robot and moving object can move from this state to the next state

        // draw boundary of the current moving objects (this.active and nextState.active)
        // draw a rectangle that cover both of them,
        // check collision with any other obstacle

        return true;
    }

    /**
     * An estimation of cost from current state to s.
     *
     * @param goalState
     * @return a double number
     */
    @Override
    public Double heuristic(State goalState) {
        if (goalState instanceof BotMateState){
            BotMateState goal = (BotMateState) goalState;
            return (Math.abs(this.getMovingBox().getPos().getX() - goal.getMovingBox().getPos().getX()) +
                    Math.abs(this.getMovingBox().getPos().getY() - goal.getMovingBox().getPos().getY()));
        } else {
            return 0.0;
        }
    }

    @Override
    public boolean equals(State s) {
        BotMateState state;
        if (s instanceof BotMateState){
            state = (BotMateState) s;
        } else {
            return false;
        }

        if (this.getMovingBox().getPos().distance(state.getMovingBox().getPos()) < BotMateSolver.MAX_BASE_STEP) {
            return true;
        }
        return false;

    }

    @Override
    public String outputString() {
        StringBuilder output = new StringBuilder();

        output.append(this.robotConfig.getPos().getX()).append(" ");
        output.append(this.robotConfig.getPos().getY()).append(" ");
        output.append(this.robotConfig.getOrientation()).append(" ");

        for (Box box: this.movingBoxes) {
            // This is the position of the bottom left conner
            output.append(box.getPos().getX() + box.getWidth()/2).append(" ");
            output.append(box.getPos().getY() + box.getWidth()/2).append(" ");
        }

        for (Box box: this.movingObstacles) {
            // This is the position of the bottom left conner
            output.append(box.getPos().getX() + box.getWidth()/2).append(" ");
            output.append(box.getPos().getY() + box.getWidth()/2).append(" ");
        }

        return output.toString();
    }

    @Override
    public List<StateCostPair> getSuccessors() {
        return null;
    }


    /**
     * Return a list of all states reachable from this state.
     *
     * @return list of successors
     */
    @Override
    public List<StateCostPair> getSuccessors(State goal) {


        List<StateCostPair> successors = new ArrayList<>();

        List<Point2D> positions = new ArrayList<>();

        Box movingBox = this.getMovingBox();
        double width = movingBox.getWidth();

        double[] delta = new double[]{width};

        for (double d: delta) {
            positions.add(new Point2D.Double(movingBox.getPos().getX() + d, movingBox.getPos().getY()));
            positions.add(new Point2D.Double(movingBox.getPos().getX() - d, movingBox.getPos().getY()));
            positions.add(new Point2D.Double(movingBox.getPos().getX(), movingBox.getPos().getY() + d));
            positions.add(new Point2D.Double(movingBox.getPos().getX(), movingBox.getPos().getY() - d));
        }

        BotMateState newState;

        for (Point2D position: positions) {
            newState = moveMovingBox(position);
            if (isConnectedWith(newState)){
                successors.add(new StateCostPair(newState, this.heuristic(goal)));
            }

        }
        return successors;
    }


    public BotMateState moveMovingBox(Point2D position) {

        Box movingBox = this.getMovingBox();
        Box newMovingBox = movingBox;
        List<Box> newMovingBoxes = new ArrayList<>();
        List<Box> newMovingObstacles = new ArrayList<>();

        RobotConfig newRobotConfig = new RobotConfig(this.robotConfig.getPos(), this.robotConfig.getOrientation());

        for (Box box: this.getMovingBoxes()) {
            MovingBox newBox = new MovingBox(box.getPos(), box.getWidth());
            newMovingBoxes.add(newBox);
            if (System.identityHashCode(box) == System.identityHashCode(movingBox) ) {
                newMovingBox = newBox;
            }
        }
        for (Box box: this.getMovingObstacles()) {
            MovingObstacle newBox = new MovingObstacle(box.getPos(), box.getWidth());
            newMovingObstacles.add(newBox);
            if (box == movingBox) {
                newMovingBox = newBox;

            }
        }

        newMovingBox.getPos().setLocation(position);

        return new BotMateState(movingBoxIndex, newRobotConfig, newMovingBoxes, newMovingObstacles, staticObstacles);

    }

}
