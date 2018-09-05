package botmate;

import problem.Box;
import problem.MovingBox;
import problem.ProblemSpec;
import tester.Tester;

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
        MovingBoxAgent movingBoxAgent;
        Tester tester;

        try {
            ps = new ProblemSpec();
            ps.loadProblem(args[0]);
            tester = new Tester(ps);
        } catch (IOException e1) {
            System.out.println("FAILED: Invalid problem file");
            System.out.println(e1.getMessage());
            return;
        }


        System.out.println("Start");
        List<Box> movingObstacles = new ArrayList<>();
        List<State> states = new LinkedList<>();


        State initialState = new State(ps.getInitialRobotConfig(), ps.getMovingBoxes(), ps.getMovingObstacles());
        State currentState = initialState;


        int movingBoxIndex = 0;


        MovingBox movingBox = (MovingBox)ps.getMovingBoxes().get(movingBoxIndex);

        for (int i = 0; i < currentState.movingBoxes.size(); i++) {
            if (movingBoxIndex != i) {
                movingObstacles.add(currentState.movingBoxes.get(i));
            }
        }

        movingObstacles.addAll(currentState.getMovingObstacles());

        if ((tester.isCoupled(currentState.robotConfig, movingBox)) < 0) {

            RobotState robotState = new RobotState(currentState.robotConfig, currentState.movingBoxes,  currentState.movingObstacles);

            robotAgent = new RobotAgent(ps, robotState, movingBox);

            states.addAll(robotAgent.search());

            currentState = states.get(states.size()-1);

            System.out.println(currentState.toString());
        }


        movingObstacles.clear();
//
//
//        BoxState boxState = new BoxState(currentState.robotConfig, movingBox, movingObstacles);
//        Point2D movingBoxGoal = ps.getMovingBoxEndPositions().get(movingBoxIndex);
//        movingBoxAgent = new MovingBoxAgent(ps, boxState, movingBox, movingBoxGoal);
//
//
//        states.addAll(movingBoxAgent.search());



        List<String> output = new LinkedList<>();
        for (State s : states) {
            output.add(s.toString());
        }

        writeOutputFile(args[1], output);


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
