package botmate;


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

    static List<BotMateState> steps;

    /**
     * Main method - solve the problem
     *
     * @param args the list of argument
     */
    public static void main(String args[]) {

        ps = new ProblemSpec();



        try {
            ps.loadProblem("input3.txt");
//            ps.loadProblem("bot.input1.txt");
        } catch (IOException e) {
            System.out.println("IO Exception occurred");
        }


        tester = new Tester(ps);

        BotMateState initialState = new BotMateState(ps.getInitialRobotConfig(), ps.getMovingBoxes(), ps.getMovingObstacles());

//        BotMateState goalState = initialState.moveBoxToPosition(0, new Point2D.Double(0.2,0.1));

//        BotMateState goalState = initialState.moveBoxToPosition(0, ps.getMovingBoxEndPositions().get(0));

        BotMateState goalState = initialState.moveRobot(new Point2D.Double(0.2, 0.1), 1.0);


//        System.out.println(tester.isCoupled(initialState.getRobotConfig(), initialState.getMovingBoxes().get(0)));
//        System.out.println(robotLine(initialState.getRobotConfig()).intersects(initialState.getMovingBoxes().get(0).getRect()));

        System.out.println(isConnected(initialState, goalState));

    }

    private static Line2D robotLine(RobotConfig robot) {
        Line2D line = new Line2D.Double(robot.getX1(ps.getRobotWidth()), robot.getY1(ps.getRobotWidth()),
                robot.getX2(ps.getRobotWidth()), robot.getY2(ps.getRobotWidth()));
        return line;
    }


    private static List<BotMateState> PRM(BotMateState state) {
        Box movingBox = state.getMovingBoxes().get(0);
        Point2D point = ps.getMovingBoxEndPositions().get(0);


        List<BotMateState> steps = new ArrayList<>();


        return steps;
    }


    private static boolean isConnected(BotMateState state1, BotMateState state2) {
        return (!checkMovingBoxCollide(state1, state2) && !checkRobotCollide(state1, state2));
        //Todo: check moving obstacle as well
    }


    private static boolean checkMovingBoxCollide(BotMateState state1, BotMateState state2) {
        Box box1, box2;

        List<Line2D> lines = new ArrayList<>();

        Line2D line;

        for (int i = 0; i < state1.getMovingBoxes().size(); i++) {
            box1 = state1.getMovingBoxes().get(i);
            box2 = state2.getMovingBoxes().get(i);
            if (!box1.getPos().equals(box2.getPos())) {
                line = new Line2D.Double(box1.getPos(), box2.getPos());

                Rectangle2D boundary = tester.grow(line.getBounds2D(), ps.getRobotWidth() / 2);


                for (int j = 0; j < state1.getMovingBoxes().size(); i++) {
                    if (j != i) {
                        Box box = state2.getMovingBoxes().get(j);
                        if (boundary.intersects(box.getRect())) {
                            return true;
                        }
                    }
                }

                for (Box box : state2.getMovingObstacles()) {
                    if (boundary.intersects(box.getRect())) {
                        return true;
                    }
                }

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

        double width = ps.getRobotWidth();
        RobotConfig r1 = state1.getRobotConfig();
        RobotConfig r2 = state2.getRobotConfig();
        double r1x1, r1x2, r1y1, r1y2;
        double r2x1, r2x2, r2y1, r2y2;


        r1x1 = r1.getX1(width);
        r1x2 = r1.getX2(width);
        r1y1 = r1.getY1(width);
        r1y2 = r1.getY2(width);
        r2x1 = r2.getX1(width);
        r2x2 = r2.getX2(width);
        r2y1 = r2.getY1(width);
        r2y2 = r2.getY2(width);

        List<Line2D> lines = new ArrayList<>();

        lines.add(new Line2D.Double(r1x1, r1y1, r2x1, r2y1));
        lines.add(new Line2D.Double(r1x1, r1y1, r2x2, r2y2));
        lines.add(new Line2D.Double(r1x2, r1y2, r2x1, r2y1));
        lines.add(new Line2D.Double(r1x2, r1y2, r2x2, r2y2));

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
