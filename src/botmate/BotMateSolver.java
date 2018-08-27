package botmate;


import problem.*;
import tester.Tester;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
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
            ps.loadProblem("bot.input1.txt");
        } catch (IOException e) {
            System.out.println("IO Exception occurred");
        }


        tester = new Tester(ps);

        BotMateState initialState = new BotMateState(ps.getInitialRobotConfig(), ps.getMovingBoxes(), ps.getMovingObstacles());

        BotMateState goalState = initialState.moveBoxToPosition(0, ps.getMovingBoxEndPositions().get(0));

        System.out.println(initialState.equals(goalState));
        System.out.println(goalState.outputString());

        if (tester.isCoupled(initialState.getRobotConfig(), ps.getMovingBoxes().get(0))  == -1 ) {
            moveRobotToBox(initialState);
        }

    }

    private static Line2D robotLine(RobotConfig robot) {
        Line2D line = new Line2D.Double(robot.getX1(ps.getRobotWidth()), robot.getY1(ps.getRobotWidth()),
                robot.getX2(ps.getRobotWidth()), robot.getY2(ps.getRobotWidth()));
        return line;
    }


    private static List<BotMateState> moveRobotToBox(BotMateState state) {
        List<BotMateState> steps = new ArrayList<>();


        return steps;
    }

    private static List<BotMateState> PRM(BotMateState state) {
        Box movingBox = state.getMovingBoxes().get(0);
        Point2D point = ps.getMovingBoxEndPositions().get(0);


        List<BotMateState> steps = new ArrayList<>();


        return steps;
    }

}
