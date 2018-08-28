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

    static ProblemSpec ps;
    static Tester tester;

    static List<StateCostPair> PathToGoals = new ArrayList<>();

    /**
     * Main method - solve the problem
     *
     * @param args the list of argument
     */
    public static void main(String args[]) {

        ps = new ProblemSpec();

        try {
            ps.loadProblem("bot.input1.txt");
        } catch (IOException e) {
            System.out.println("IO Exception occurred");
        }

        tester = new Tester(ps);

        BotMateState initialState = new BotMateState(ps.getInitialRobotConfig(), ps.getMovingBoxes(), ps.getMovingObstacles());
        BotMateState goalState = initialState.moveRobot(new Point2D.Double(0.75, 0.8), Math.PI * 0.5);


        if (tester.isCoupled(initialState.getRobotConfig(), ps.getMovingBoxes().get(0)) == -1 ) {

            List<BotMateState> possibleState = new ArrayList<>();
            possibleState.add(initialState);
            possibleState.addAll(PRMForRobot(initialState,500));
            possibleState.add(goalState);
            for (BotMateState state: possibleState) {
                for (BotMateState nextState: possibleState) {
                    if (isConnected(state, nextState)) {
                        state.addSuccessor(new StateCostPair(nextState, 1));
                    }
                }
            }
        }

        SearchAgent agent = new UCS();
        List<StateCostPair> solution = agent.search(initialState, goalState);
        BotMateState currentState = initialState;
        if (solution != null) {
            PathToGoals.addAll(solution);
            for (StateCostPair scp : PathToGoals) {
                for (String s: generateSteps(currentState, (BotMateState)scp.state)) {
                    System.out.println(s);
                }
                currentState = (BotMateState)scp.state;
//                System.out.println(scp.state.outputString());
            }
        } else {
            System.out.println("no solution");
        }

    }


    private static List<BotMateState> PRM(BotMateState state) {
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
                    if (!hasCollide(step)) {
                        steps.add(step);
                        break;
                    }
                }
            }

        return steps;
    }

    private static boolean isConnected(BotMateState state1, BotMateState state2) {

        if (checkMovingBoxCollide(state1, state2)) {
            return false;
        }

        if (checkRobotCollide(state1, state2)) {
            return false;
        }
        return true;
    }

    // Check if the moving box collide with other obstacle when moving from state 1 to state 2
    private static boolean checkMovingBoxCollide(BotMateState state1, BotMateState state2) {
        Line2D line;

        // For any moving box,
        for (int i = 0; i < state1.getMovingBoxes().size(); i++) {
            Box box1 = state1.getMovingBoxes().get(i);
            Box box2 = state2.getMovingBoxes().get(i);

            // if the box is not moved
            if (!box1.getPos().equals(box2.getPos())) {

                // Draw a line between two center point
                line = new Line2D.Double(box1.getPos(), box2.getPos());

                // Get the boundary and add padding (w/2)
                Rectangle2D boundary = tester.grow(line.getBounds2D(), ps.getRobotWidth() / 2);

                // Check intersect with other moving box
                for (int j = 0; j < state1.getMovingBoxes().size(); i++) {
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
        }
        return false;
    }


    private static boolean checkRobotCollide(BotMateState state1, BotMateState state2) {

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
            for (Box box: state2.getMovingBoxes()) {
                if (line.intersects(tester.grow(box.getRect(), 0.01))) {
                    return true;
                }
            }

            for (Box box: state2.getMovingObstacles()) {
                if (line.intersects(tester.grow(box.getRect(), 0.01))) {
                    return true;
                }
            }

            for (StaticObstacle obstacle: ps.getStaticObstacles()) {
                if (line.intersects(tester.grow(obstacle.getRect(),0.01))) {
                    return true;
                }
            }

        }

        return false;
    }


    private static boolean hasCollide(BotMateState state) {

        // Get robot config
        RobotConfig robot = state.getRobotConfig();

        // Draw lines between points the current and next robot,
        Line2D line = new Line2D.Double(tester.getPoint1(robot), tester.getPoint2(robot));

        // Check collision between every line and obstacle
        if (line.intersectsLine(new Line2D.Double(0,0,0,1))) { return true; }
        if (line.intersectsLine(new Line2D.Double(0,1,1,1))) { return true; }
        if (line.intersectsLine(new Line2D.Double(1,1,1,0))) { return true; }
        if (line.intersectsLine(new Line2D.Double(1,0,0,0))) { return true; }

        for (Box box: state.getMovingBoxes()) {
            if (line.intersects(box.getRect())) {
                return true;
            }
        }

        for (Box box: state.getMovingObstacles()) {
            if (line.intersects(box.getRect())) {
                return true;
            }
        }

        for (StaticObstacle obstacle: ps.getStaticObstacles()) {
            if (line.intersects(obstacle.getRect())) {
                return true;
            }
        }


        return false;
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

}
