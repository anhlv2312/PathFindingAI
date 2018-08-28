package botmate;


import common.BFS;
import common.SearchAgent;
import common.StateCostPair;
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
            ps.loadProblem("input3.txt");
        } catch (IOException e) {
            System.out.println("IO Exception occurred");
        }

        tester = new Tester(ps);

        BotMateState initialState = new BotMateState(ps.getInitialRobotConfig(), ps.getMovingBoxes(), ps.getMovingObstacles());
        BotMateState goalState = initialState.moveRobot(new Point2D.Double(0.8, 0.2), Math.PI * 0.5);

        if (tester.isCoupled(initialState.getRobotConfig(), ps.getMovingBoxes().get(0)) == -1 ) {

            List<BotMateState> possibleState = new ArrayList<>();
            possibleState.add(initialState);
            possibleState.addAll(PRMForRobot(initialState,100));
            possibleState.add(goalState);
            for (BotMateState state: possibleState) {
                for (BotMateState nextState: possibleState) {
                    if (isConnected(state, nextState)) {
                        double distance = state.getRobotConfig().getPos().distance(nextState.getRobotConfig().getPos());
                        state.addSuccessor(new StateCostPair(nextState, distance));
                    }
                }
            }

        }

        SearchAgent agent = new BFS();
        PathToGoals.addAll(agent.search(initialState, goalState));
        for (StateCostPair scp: PathToGoals) {
            System.out.println(((BotMateState)scp.state).getRobotConfig().getPos());
        }
//


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

            for (double o: orientations) {
                step = state.moveRobot(point, Math.PI * o);
                if (!checkRobotCollide(state, step)) {
                    steps.add(step);
//                    System.out.println(point);
                    break;
                }
            }

        }
        return steps;
    }

    private static boolean isConnected(BotMateState state1, BotMateState state2) {
        return (!checkMovingBoxCollide(state1, state2) && !checkRobotCollide(state1, state2));
        //Todo: check moving obstacle as well
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
                    if (j != i) {
                        Box box = state2.getMovingBoxes().get(j);
                        if (boundary.intersects(box.getRect())) {
                            return true;
                        }
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

        // Get robot width
        double width = ps.getRobotWidth();

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
                if (line.intersects(box.getRect())) {
                    return true;
                }
            }

            for (Box box: state2.getMovingObstacles()) {
                if (line.intersects(box.getRect())) {
                    return true;
                }
            }

            for (StaticObstacle obstacle: ps.getStaticObstacles()) {
                if (line.intersects(obstacle.getRect())) {
                    return true;
                }
            }

        }

        return false;
    }

}
