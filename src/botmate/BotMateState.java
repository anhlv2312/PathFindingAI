package botmate;

import common.State;
import common.StateCostPair;
import problem.*;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.*;


public class BotMateState implements State {

    private int movingBoxIndex;
    private RobotConfig robotConfig;
    private List<Box> movingBoxes;
    private List<Box> movingObstacles;
    private tester.Tester tester;

    private List<StateCostPair> successors = new ArrayList<>();

    /**
     * Create an problem state from an RobotConfig and list of moving objects
     *
     * @param robotConfig     the robot configuration
     * @param movingBoxes     a list of moving boxes
     * @param movingObstacles a list of moving obstacles
     */
    public BotMateState(int movingBoxIndex, RobotConfig robotConfig, List<Box> movingBoxes, List<Box> movingObstacles, tester.Tester tester) {

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

        this.tester = tester;
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

        if (this.getMovingBox().getPos().distance(state.getMovingBox().getPos()) > tester.MAX_ERROR) {
            return false;
        }

        if (this.getRobotConfig().getPos().distance(state.getRobotConfig().getPos()) > tester.MAX_ERROR) {
            return false;
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



    public void addSuccessor(StateCostPair stateCostPair) {
        successors.add(stateCostPair);
    }

    @Override
    public List<StateCostPair> getSuccessors() {
        return successors;
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
            List<Box> movingObjects = new ArrayList<>();
            movingObjects.addAll(newState.getMovingBoxes());
            movingObjects.addAll(newState.getMovingObstacles());
            if (tester.hasCollision(this.getRobotConfig(), movingObjects)){
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

        return new BotMateState(movingBoxIndex, newRobotConfig, newMovingBoxes, newMovingObstacles, tester);

    }


    public BotMateState moveRobot(Point2D position, double orientation) {
        RobotConfig newRobotConfig = new RobotConfig(position, orientation);
        return new BotMateState(movingBoxIndex, newRobotConfig, this.movingBoxes, this.movingObstacles, tester);
    }

    /**
     *          1 if robot on bottom of box
     *          2 if robot on left of box
     *          3 if robot on top of box
     *          4 if robot on right of box
     */
    public BotMateState moveRobotToMovingBox(int edge) {

        Double w = this.getMovingBox().getWidth();
        double bottomLeftX = this.getMovingBox().getPos().getX();
        double bottomLeftY = this.getMovingBox().getPos().getY();

        Point2D position;
        double orientation;
        switch (edge) {
            case 1:
                position = new Point2D.Double(bottomLeftX + w / 2, bottomLeftY - tester.MAX_ERROR);
                orientation = 0.0;
                break;
            case 2:
                position = new Point2D.Double(bottomLeftX - tester.MAX_ERROR, bottomLeftY + w / 2);
                orientation = Math.PI * 0.5;
                break;
            case 3:
                position = new Point2D.Double(bottomLeftX + w / 2, bottomLeftY + w + tester.MAX_ERROR);
                orientation = 0.0;
                break;
            case 4:
                position = new Point2D.Double(bottomLeftX + w + tester.MAX_ERROR, bottomLeftY + w / 2);
                orientation = Math.PI * 0.5;
                break;
            default:
                position = this.robotConfig.getPos();
                orientation = this.robotConfig.getOrientation();
        }

        return this.moveRobot(position, orientation);
    }

}
