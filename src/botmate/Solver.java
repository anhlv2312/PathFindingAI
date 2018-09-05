package botmate;

import problem.Box;
import problem.ProblemSpec;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class Solver {

    public static void main(String args[]) {

        ProblemSpec ps;
        RobotAgent robotAgent;

        try {
            ps = new ProblemSpec();
            ps.loadProblem(args[0]);
        } catch (IOException e1) {
            System.out.println("FAILED: Invalid problem file");
            System.out.println(e1.getMessage());
            return;
        }


        System.out.println("Start");
        List<Box> movingObstacles = new ArrayList<>();
        movingObstacles.addAll(ps.getMovingBoxes());
        movingObstacles.addAll(ps.getMovingObstacles());

        RobotState robotState = new RobotState(ps.getInitialRobotConfig(), movingObstacles);

        RobotState testState = robotState.moveRobot(0.7, 0.3, Math.PI/2);

        robotAgent = new RobotAgent(ps, robotState, testState.robotConfig);

        List<State> states = robotAgent.search();


        List<String> output = new LinkedList<>();
        for (State s : states) {
            output.add(s.toString());
        }

        writeOutputFile(args[1], output);

        System.out.println(robotState);
        System.out.println(testState);


    }

    private static void writeOutputFile(String fileName, List<String> output) {
        try {
            FileWriter fw = new FileWriter(fileName);
            BufferedWriter bw = new BufferedWriter(fw);

            System.out.println("Number of steps: " + output.size());
            bw.write(Integer.toString(output.size()));
            bw.write("\n");
            bw.flush();
            for (String string : output) {
                bw.write(string);
                bw.write("\n");
            }
            bw.close();

        } catch (IOException ex) {
            System.out.println("IO Exception occurred");
        }
    }
}
