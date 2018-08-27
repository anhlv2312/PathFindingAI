package botmate;

import common.SearchAgent;
import problem.ProblemSpec;
import problem.Box;
import problem.RobotConfig;


import java.io.IOException;

public class BotMateSolver {
    /**
     * Main method - solve the problem
     *
     * @param args the list of argument
     */
    public static void main(String args[]) {
        SearchAgent agent;

        ProblemSpec ps = new ProblemSpec();
        try {
            ps.loadProblem("input1.txt");
        } catch (IOException e) {
            System.out.println("IO Exception occured");
        }

        for (Box currentBox: ps.getMovingBoxes()) {

            BotMateState initial = new BotMateState(ps.getInitialRobotConfig(), ps.getMovingBoxes(), ps.getMovingObstacles());
            BotMateState goal = new BotMateState(ps.getInitialRobotConfig(), ps.getMovingBoxes(), ps.getMovingObstacles());

        }

        BotMateState initial = new BotMateState(ps.getInitialRobotConfig(), ps.getMovingBoxes(), ps.getMovingObstacles());
        //todo: define goal by specify the position of final moving boxes;
        BotMateState goal = null;

    }

    private static void moveRobotToBox(RobotConfig robot, Box target) {

    }

    private static void moveBoxToGoal(RobotConfig robot, Box target) {

    }




}
