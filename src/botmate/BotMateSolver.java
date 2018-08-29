package botmate;


import common.*;
import problem.*;
import tester.Tester;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class BotMateSolver {


    public static final int ROBOT_PRM_SAMPLES = 500;


    static ProblemSpec ps;
    static Tester tester;

    static List<BotMateState> solutionStates = new LinkedList<>();

    /**
     * Main method - solve the problem
     *
     * @param args the list of argument
     */
    public static void main(String args[]) {

        try {
            ps = new ProblemSpec();
            ps.loadProblem("bot.input1.txt");
            tester = new Tester(ps);
        } catch (IOException e) {
            System.out.println("IO Exception occurred");
        }

        BotMateState currentState = new BotMateState(ps.getInitialRobotConfig(), ps.getMovingBoxes(), ps.getMovingObstacles());
        solutionStates.add(currentState);
        // loop all the box
        for (int boxIndex = 0; boxIndex < ps.getMovingBoxes().size(); boxIndex++) {

            Box currentBox = currentState.getMovingBoxes().get(boxIndex);
            println("check box " + currentBox);

            Point2D currentGoal = ps.getMovingBoxEndPositions().get(boxIndex);

            // if currentBox not its goal then dot it
            if (currentBox.getPos().distance(currentGoal) > tester.MAX_ERROR) {

                println("not in goal " + boxIndex);
                // if the robot is not coupled with the robot, move to robot to the box
                if (tester.isCoupled(currentState.getRobotConfig(), ps.getMovingBoxes().get(boxIndex)) == -1 ) {
                    println("not coupled " + boxIndex);
                    currentState = moveRobotToBox(boxIndex, currentState);
                }
                println(currentState.getRobotConfig().getPos());
//                println("move box to goal " + boxIndex);
//                currentState = moveBoxToGoal(boxIndex, currentState);
            }
        }

        for (BotMateState state : solutionStates) {
            println(state.outputString());
//            for (String string: generateSteps(currentState, state)) {
//                println(string);
//            }
        }

    }

    private static BotMateState moveRobotToBox(int boxIndex, BotMateState initialState) {

        Point2D target = initialState.getMovingBoxes().get(boxIndex).getPos();
        BotMateState goalState = initialState.moveRobot(target, Math.PI * 0.5);

        List<BotMateState> possibleStates = new ArrayList<>();

        possibleStates.add(initialState);
        possibleStates.addAll(PRMForRobot(initialState, ROBOT_PRM_SAMPLES));
        possibleStates.add(goalState);

        println("sampleling " + possibleStates.size());

        for (BotMateState state : possibleStates) {
            for (BotMateState nextState : possibleStates) {
                // if the moving robot is not collide with other box
                if (!checkRobotCollide(boxIndex, state, nextState)) {
                    // Add the next state to the successor
                    state.addSuccessor(new StateCostPair(nextState, 1));
                }

            }
        }


        SearchAgent agent = new BFS();

        println("Searching");
        List<StateCostPair> solution = agent.search(initialState, goalState);

        if (solution != null) {
            for (StateCostPair scp : solution) {
                solutionStates.add((BotMateState) scp.state);
            }
            return goalState;
        } else {
            println("No Solution");
            return initialState;
        }
    }

    private static BotMateState moveBoxToGoal(int boxIndex, BotMateState initialState) {
        BotMateState goalState = initialState;
        return initialState;

    }

    private static List<BotMateState> PRMForBox(BotMateState state) {
        Box movingBox = state.getMovingBoxes().get(0);
        Point2D point = ps.getMovingBoxEndPositions().get(0);

        List<BotMateState> steps = new ArrayList<>();

        return steps;
    }

    private static List<BotMateState> PRMForRobot(BotMateState state, int numberOfSample) {
        double[] orientations = new double[] {0.0, 0.25, 0.5, 0.75};
        List<BotMateState> steps = new ArrayList<>();
        BotMateState step;
        for (int i = 0; i < numberOfSample; i++) {
            Point2D point = new Point2D.Double(Math.random(), Math.random());
                for (double o : orientations) {
                    step = state.moveRobot(point, Math.PI * o);
                    if (checkRobotPosition(step)) {
                        steps.add(step);
                        break;
                    }
                }
            }

        return steps;
    }

    // Check if the moving box collide with other obstacle when moving from state 1 to state 2
    private static boolean checkMovingBoxCollide(int boxIndex, BotMateState state1, BotMateState state2) {
        Line2D line;

        // For any moving box,
            Box box1 = state1.getMovingBoxes().get(boxIndex);
            Box box2 = state2.getMovingBoxes().get(boxIndex);

            // if the box is not moved
        if (!box1.getPos().equals(box2.getPos())) {

            // Draw a line between two center point
            line = new Line2D.Double(box1.getPos(), box2.getPos());

            // Get the boundary and add padding (w/2)
            Rectangle2D boundary = tester.grow(line.getBounds2D(), ps.getRobotWidth() / 2);

            // Check intersect with other moving box
            for (int j = 0; j < state1.getMovingBoxes().size(); j++) {
                Box box = state2.getMovingBoxes().get(j);
                if (boundary.intersects(box.getRect())) {
                    return true;
                }

            }

            // Check intersect with other moving obstacle box
            for (Box box : state2.getMovingObstacles()) {
                if (boundary.intersects(box.getRect())) {
                    return true;
                }
            }

            // Check intersect with static obstacles
            for (StaticObstacle obstacle : ps.getStaticObstacles()) {
                if (boundary.intersects(obstacle.getRect())) {
                    return true;
                }
            }
        }
        return false;
    }


    private static boolean checkRobotCollide(int boxIndex, BotMateState state1, BotMateState state2) {

        // Get robot config
        RobotConfig r1 = state1.getRobotConfig();
        RobotConfig r2 = state2.getRobotConfig();

        Point2D r1p1,r1p2;
        Point2D r2p1,r2p2;

        r1p1 = tester.getPoint1(r1);
        r1p2 = tester.getPoint2(r1);
        r2p1 = tester.getPoint1(r2);
        r2p2 = tester.getPoint2(r2);


        // Initial a list of line
        List<Line2D> lines = new ArrayList<>();

        // Draw lines between points the current and next robot,
        lines.add(new Line2D.Double(r1p1, r2p1));
        lines.add(new Line2D.Double(r1p1, r2p2));
        lines.add(new Line2D.Double(r1p2, r2p1));
        lines.add(new Line2D.Double(r1p2, r2p2));

        // Check collision between every line and obstacle
        for (Line2D line: lines) {
            for (int i = 0; i < state2.getMovingBoxes().size(); i++ ) {
                if (i == boxIndex) {
                    continue;
                }
                Box box = state2.getMovingBoxes().get(i);
                if (line.intersects(tester.grow(box.getRect(), tester.MAX_BASE_STEP))) {
                    return true;
                }
            }

            for (Box box: state2.getMovingObstacles()) {
                if (line.intersects(tester.grow(box.getRect(), tester.MAX_BASE_STEP))) {
                    return true;
                }
            }

            for (StaticObstacle obstacle: ps.getStaticObstacles()) {
                if (line.intersects(tester.grow(obstacle.getRect(),tester.MAX_BASE_STEP))) {
                    return true;
                }
            }

        }

        return false;
    }


    private static boolean checkRobotPosition(BotMateState state) {

        // Get robot config
        RobotConfig robot = state.getRobotConfig();

        // Draw lines between points the current and next robot,
        Line2D robotLine = new Line2D.Double(tester.getPoint1(robot), tester.getPoint2(robot));

        // Check collision with boundary
        double e = tester.MAX_ERROR;
        if (robotLine.intersectsLine(new Line2D.Double(-e,-e,-e,1+e))) { return true; }
        if (robotLine.intersectsLine(new Line2D.Double(-e,1+e,1+e,1+e))) { return true; }
        if (robotLine.intersectsLine(new Line2D.Double(-e,1+e,1+e,-e))) { return true; }
        if (robotLine.intersectsLine(new Line2D.Double(1+e,-e,-e,-e))) { return true; }


        // Check collision between every line and obstacle
        for (Box box: state.getMovingBoxes()) {
            if (robotLine.intersects(box.getRect())) {
                return false;
            }
        }

        for (Box box: state.getMovingObstacles()) {
            if (robotLine.intersects(box.getRect())) {
                return false;
            }
        }

        for (StaticObstacle obstacle: ps.getStaticObstacles()) {
            if (robotLine.intersects(obstacle.getRect())) {
                return false;
            }
        }

        return true;
    }

    private static List<String> generateSteps(BotMateState state1, BotMateState state2) {

        List<String> result = new LinkedList<>();
        result.add(state1.outputString());
        BotMateState movingState = new BotMateState(state1);
        RobotConfig r1 = state1.getRobotConfig();
        RobotConfig r2 = state2.getRobotConfig();
        Double numberOfSteps = Math.ceil(r1.getPos().distance(r2.getPos())/0.001);

        double deltaX = (r2.getPos().getX() - r1.getPos().getX())/numberOfSteps;
        double deltaY = (r2.getPos().getY() - r1.getPos().getY())/numberOfSteps;
        double deltaO = (r2.getOrientation() - r1.getOrientation())/numberOfSteps;

        for (int i = 0; i < numberOfSteps; i ++) {
            RobotConfig r3 = movingState.getRobotConfig();
            Point2D newPosition = new Point2D.Double(r3.getPos().getX() + deltaX, r3.getPos().getY() + deltaY);
            movingState = movingState.moveRobot(newPosition, r3.getOrientation() + deltaO);
            result.add(movingState.outputString());
        }

        return result;
    }

    private static void print(Object o) {
        System.out.print(o);
    }
    private static void println(Object o) {
        System.out.print(o);
        System.out.print("\n");
    }

    public static double calculateDistance(Point2D initial, Point2D goal) {
        return (Math.abs(initial.getX() - goal.getX()) + Math.abs(initial.getY() - goal.getY()));
    }

}
