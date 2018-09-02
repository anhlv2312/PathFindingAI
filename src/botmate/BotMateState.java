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
//            double distance = (Math.abs(this.getMovingBox().getPos().getX() -
//                    goal.getMovingBox().getPos().getX()) +
//                    Math.abs(this.getMovingBox().getPos().getY() -
//                            goal.getMovingBox().getPos().getY()));
//
            double boxDistance = this.getMovingBox().getPos().distance(goal.getMovingBox().getPos());
            double robotDistance = this.getRobotConfig().getPos().distance(goal.getRobotConfig().getPos());
            double angle = Math.abs(this.getRobotConfig().getOrientation()-goal.getRobotConfig().getOrientation());

            return boxDistance + robotDistance + angle;
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

        if (tester.isCoupled(robotConfig, getMovingBox()) > 0) {
            if (this.getMovingBox().getPos().distance(state.getMovingBox().getPos()) > tester.MAX_ERROR ) {
                return false;
            }
        } else {
            return (this.getRobotConfig().equals(state.getRobotConfig()));
        }


//
//        if (this.getMovingBox().getPos().distance(this.getMovingBox().getPos()) > tester.MAX_ERROR) {
//            return false;
//        }

//        for (int i=0; i < getMovingBoxes().size(); i++) {
//            if (!this.getMovingBoxes().get(i).getPos().equals(state.getMovingBoxes().get(i))) {
//                return false;
//            }
//        }

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

        int coupled = tester.isCoupled(robotConfig, getMovingBox());
        if (coupled < 0) {
            return generateRobotSuccessors(goal);
        } else {
            return generateBoxSuccessors(goal, coupled);
        }

    }


    private List<StateCostPair> generateRobotSuccessors(State goal) {
        List<StateCostPair> successors = new LinkedList<>();


        double[] angles = new double[]{0, Math.PI * 0.25, Math.PI * 0.5, Math.PI * 0.75};

        double d = 0.01;

        List<Point2D> positions = new ArrayList<>();

        List<RobotConfig> configs = new ArrayList<>();

        positions.add(new Point2D.Double(robotConfig.getPos().getX(), robotConfig.getPos().getY() - d));
        positions.add(new Point2D.Double(robotConfig.getPos().getX() - d, robotConfig.getPos().getY()));
        positions.add(new Point2D.Double(robotConfig.getPos().getX(), robotConfig.getPos().getY() + d));
        positions.add(new Point2D.Double(robotConfig.getPos().getX() + d, robotConfig.getPos().getY()));


        BotMateState newState;

        for (Point2D position: positions) {
            for (double angle : angles) {

                newState = this.moveRobotToPosition(position, angle);

                List<Box> movingObjects = new ArrayList<>();
                movingObjects.addAll(newState.getMovingBoxes());
                movingObjects.addAll(newState.getMovingObstacles());

                if (tester.hasCollision(newState.getRobotConfig(), movingObjects)) {
                    System.out.println(newState.getRobotConfig().getPos());
                    successors.add(new StateCostPair(newState, 1 + newState.heuristic(goal)));
                }
            }
        }

        return successors;
    }

    private List<StateCostPair> generateBoxSuccessors(State goal, int coupled) {

        List<StateCostPair> successors = new LinkedList<>();

        Map<Integer, Point2D> positions = new HashMap<>();
        Box movingBox = this.getMovingBox();
        double d = movingBox.getWidth();

        positions.put(3, new Point2D.Double(movingBox.getPos().getX(), movingBox.getPos().getY() - d));
        positions.put(4, new Point2D.Double(movingBox.getPos().getX() - d, movingBox.getPos().getY()));
        positions.put(1, new Point2D.Double(movingBox.getPos().getX(), movingBox.getPos().getY() + d));
        positions.put(2, new Point2D.Double(movingBox.getPos().getX() + d, movingBox.getPos().getY()));

        positions.remove(coupled);

        BotMateState newState;

        for (Map.Entry<Integer, Point2D> entry : positions.entrySet()) {
            newState = this.moveMovingBox(entry.getValue());
            newState = newState.moveRobotToMovingBox(entry.getKey());

            List<Box> movingObjects = new ArrayList<>();
            movingObjects.addAll(newState.getMovingBoxes());
            movingObjects.addAll(newState.getMovingObstacles());

            if (tester.hasCollision(newState.getRobotConfig(), movingObjects)) {
                successors.add(new StateCostPair(newState, 1 + newState.heuristic(goal)));
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


    public BotMateState moveRobotToPosition(Point2D position, double orientation) {
        RobotConfig newRobotConfig = new RobotConfig(position, orientation);
        return new BotMateState(movingBoxIndex, newRobotConfig, this.movingBoxes, this.movingObstacles, tester);
    }

    public BotMateState moveRobot(double deltaX, double deltaY, double deltaO) {
        Double currentX, currentY;
        currentX = this.getRobotConfig().getPos().getX();
        currentY = this.getRobotConfig().getPos().getY();

        Point2D position = new Point2D.Double(currentX + deltaX, currentY + deltaY);
        double orientation = this.getRobotConfig().getOrientation() + deltaO;

        RobotConfig newRobotConfig = new RobotConfig(position, orientation);
        return new BotMateState(movingBoxIndex, newRobotConfig, movingBoxes, movingObstacles, tester);
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
                position = new Point2D.Double(bottomLeftX + w / 2, bottomLeftY);
                orientation = 0.0;
                break;
            case 2:
                position = new Point2D.Double(bottomLeftX, bottomLeftY + w / 2);
                orientation = Math.PI/2;
                break;
            case 3:
                position = new Point2D.Double(bottomLeftX + w / 2, bottomLeftY + w);
                orientation = 0.0;
                break;
            case 4:
                position = new Point2D.Double(bottomLeftX + w, bottomLeftY + w / 2);
                orientation = Math.PI/2;
                break;
            default:
                position = this.robotConfig.getPos();
                orientation = this.robotConfig.getOrientation();
        }

        return this.moveRobotToPosition(position, orientation);
    }

    public BotMateState moveRobotOut() {

        int edge = tester.isCoupled(robotConfig, getMovingBox());
        Double width = this.getMovingBox().getWidth();

        switch (edge) {
            case 1:
                return this.moveRobot(0, -width/2, 0);
            case 2:
                return this.moveRobot(-width/2, 0, 0);
            case 3:
                return this.moveRobot(0, width/2, 0);
            case 4:
                return this.moveRobot(width/2, 0, 0);
            default:
                return null;
        }

    }

}
