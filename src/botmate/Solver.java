package botmate;

import problem.Box;
import problem.ProblemSpec;
import tester.Tester;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Solver {


    static ProblemSpec ps;
    static Tester tester;
    static BotMateAgent agent = new BotMateAgent();
    static double robotWidth;

    public static void main(String args[]) {

        try {
            ps = new ProblemSpec();
            ps.loadProblem(args[0]);
            robotWidth = ps.getRobotWidth();
            tester = new Tester(ps);
        } catch (IOException e1) {
            System.out.println("FAILED: Invalid problem file");
            System.out.println(e1.getMessage());
            return;
        }

        List<Box> movingObstacles = new ArrayList<>();
        movingObstacles.addAll(ps.getMovingBoxes());
        movingObstacles.addAll(ps.getMovingObstacles());

        RobotState robotState = new RobotState(ps.getInitialRobotConfig(), movingObstacles);



    }
}
