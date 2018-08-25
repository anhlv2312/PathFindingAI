package botmate;

import common.SearchAgent;
import problem.ProblemSpec;

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

        BotMateState initial = new BotMateState(ps.getInitialRobotConfig(), ps.getMovingBoxes(), ps.getMovingObstacles());
        //todo: define goal by specify the position of final moving boxes;
        BotMateState goal = null;

    }
}
