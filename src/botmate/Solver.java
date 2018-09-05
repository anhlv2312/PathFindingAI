package botmate;

import problem.Box;
import problem.MovingBox;
import problem.MovingObstacle;
import problem.ProblemSpec;
import tester.Tester;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

public class Solver {

    static ProblemSpec ps;
    static RobotAgent robotAgent;
    static MovingBoxAgent movingBoxAgent;
    static MovingObstacleAgent movingObstacleAgent;
    static Tester tester;

    public static void main(String args[]) {

        try {
            ps = new ProblemSpec();
            ps.loadProblem(args[0]);
            tester = new Tester(ps);
        } catch (IOException e1) {
            System.out.println("FAILED: Invalid problem file");
            System.out.println(e1.getMessage());
            return;
        }


        System.out.println("Start solving problem");

        List<State> states = new LinkedList<>();
        Box movingBox;

        State initialState = new State(ps.getInitialRobotConfig(), ps.getMovingBoxes(), ps.getMovingObstacles());
        State currentState = initialState;
        List<State> solution;


        for (int movingBoxIndex = 0; movingBoxIndex < ps.getMovingBoxes().size(); movingBoxIndex++) {

            movingBox = currentState.movingBoxes.get(movingBoxIndex);

            if (tester.isCoupled(currentState.robotConfig, movingBox) < 0) {
                System.out.println("Find robot path to MovingBox: " + movingBoxIndex);
                State robotState = new State(currentState.robotConfig, currentState.movingBoxes, currentState.movingObstacles);
                robotAgent = new RobotAgent(ps, robotState, movingBox);
                solution = robotAgent.search();
                if (solution != null) {
                    states.addAll(solution);
                } else {
                    System.out.println("No Solution");
                }
                currentState = states.get(states.size() - 1);
            }


            Point2D movingBoxGoal = ps.getMovingBoxEndPositions().get(movingBoxIndex);
            movingBoxAgent = new MovingBoxAgent(ps, currentState, movingBoxIndex, movingBoxGoal, true);

            solution = movingBoxAgent.search();

            if (solution == null) {

                System.out.println("No direct solution for MovingBox");



                movingBoxAgent = new MovingBoxAgent(ps, currentState, movingBoxIndex, movingBoxGoal, false);
                List<State> movingStates = movingBoxAgent.search();

                List<Rectangle2D> movingPaths = generateMovingPath(movingBoxIndex, movingStates);

                Set<Integer> movingObstacleIndexes = getMovingObstaclesIndexes(movingPaths, currentState.movingObstacles);

                for (int j = 0; j < movingObstacleIndexes.size(); j++) {

                    movingBox = currentState.movingObstacles.get(j);



                    if (tester.isCoupled(currentState.robotConfig, movingBox) < 0) {
                        System.out.println("Find robot path to MovingObstacle: " + j);
                        State robotState = new State(currentState.robotConfig, currentState.movingBoxes, currentState.movingObstacles);
                        robotAgent = new RobotAgent(ps, robotState, movingBox);
                        solution = robotAgent.search();
                        if (solution != null) {
                            states.addAll(solution);
                        } else {
                            System.out.println("No Solution for Robot");
                        }
                        currentState = states.get(states.size() - 1);
                    }

                    movingObstacleAgent = new MovingObstacleAgent(ps, currentState, j, movingPaths);

                    solution = movingObstacleAgent.search();

                    if (solution == null) {
                        System.out.println("No Solution for MovingObstacle");
                    }


                }

                states.addAll(movingStates);

                currentState = states.get(states.size() - 1);
            } else {

                states.addAll(solution);
                currentState = states.get(states.size() - 1);
            }

            int robotPosition = tester.isCoupled(currentState.robotConfig, currentState.movingBoxes.get(movingBoxIndex));
            currentState = currentState.moveRobotOut(robotPosition, tester.MAX_ERROR);

        }

        List<String> output = new LinkedList<>();
        for (State s : states) {

//            System.out.println(s.toString());
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


    public static List<Rectangle2D> generateMovingPath(int movingBoxIndex, List<State> movingStates) {

        List<Rectangle2D> rectangles = new ArrayList<>();

        for (int i = 0; i < movingStates.size() - 1; i++) {
            Box currentBox = movingStates.get(i).movingBoxes.get(movingBoxIndex);
            Box nextBox = movingStates.get(i+1).movingBoxes.get(movingBoxIndex);
            Point2D currentCenter = new Point2D.Double(currentBox.getPos().getX() + ps.getRobotWidth()/2, currentBox.getPos().getY()+ps.getRobotWidth()/2);
            Point2D nextCenter = new Point2D.Double(nextBox.getPos().getX() + ps.getRobotWidth()/2, nextBox.getPos().getY()+ps.getRobotWidth()/2);
            Line2D line1 = new Line2D.Double(currentCenter, nextCenter);
            Rectangle2D rectangle = tester.grow(line1.getBounds2D(), ps.getRobotWidth() / 2 + tester.MAX_ERROR);
            rectangles.add(rectangle);
        }
        return rectangles;
    }

    public static Set<Integer> getMovingObstaclesIndexes(List<Rectangle2D> movingPaths, List<Box> movingObstacles) {
        Set<Integer> indexes = new HashSet<>();

        for (int i=0; i < movingObstacles.size(); i++) {
            Box box = movingObstacles.get(i);
            for (Rectangle2D rect : movingPaths) {
                if (rect.intersects(box.getRect())) {
                    indexes.add(i);
                }
            }
        }
        return indexes;
    }
}
