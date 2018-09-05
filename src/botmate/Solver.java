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
    static State initialState, currentState;
    static List<State> solutionStates = new LinkedList<>();

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
        solveProblem();
        writeOutputFile(args[1]);
    }

    private static void solveProblem() {

        System.out.println("Start solving problem");

        initialState = new State(ps.getInitialRobotConfig(), ps.getMovingBoxes(), ps.getMovingObstacles());
        currentState = initialState;
        solutionStates.add(initialState);
        List<State> solution;

        for (int movingBoxIndex = 0; movingBoxIndex < ps.getMovingBoxes().size(); movingBoxIndex++) {

            Point2D movingBoxGoal = ps.getMovingBoxEndPositions().get(movingBoxIndex);
            solution = findDirectSolution(currentState, movingBoxIndex, movingBoxGoal);

            if (solution != null) {
                solutionStates.addAll(solution);
                currentState = solutionStates.get(solutionStates.size() - 1);
            } else {

                List<State> alternativeSolution = findAlternativeSolution(currentState, movingBoxIndex, movingBoxGoal);

                if (alternativeSolution == null) {
                    System.out.println("\tNo alternative solution!");
                } else {
                    solutionStates.addAll(alternativeSolution);
                    currentState = solutionStates.get(solutionStates.size() - 1);
                }

                System.out.println("Find solution again for: " + movingBoxIndex);
                solution = findDirectSolution(currentState, movingBoxIndex, movingBoxGoal);

                if (solution != null) {
                    solutionStates.addAll(solution);
                }

                currentState = solutionStates.get(solutionStates.size() - 1);
            }

            int robotPosition = tester.isCoupled(currentState.robotConfig, currentState.movingBoxes.get(movingBoxIndex));
            currentState = currentState.moveRobotOut(robotPosition, tester.MAX_ERROR);

        }
    }
    private static List<State> findDirectSolution(State state, int movingBoxIndex, Point2D movingBoxGoal) {
        System.out.println("Find direct solution for MovingBox: " + movingBoxIndex);
        movingBoxAgent = new MovingBoxAgent(ps, state, movingBoxIndex, movingBoxGoal, true);
        return movingBoxAgent.search();
    }

    private static List<State> findAlternativeSolution(State state, int movingBoxIndex, Point2D movingBoxGoal) {

        boolean hasSolution = true;
        List<State> states = new LinkedList<>();


        System.out.println("\tFind alternative solution");

        movingBoxAgent = new MovingBoxAgent(ps, state, movingBoxIndex, movingBoxGoal, false);
        List<State> alternativeSolution = movingBoxAgent.search();

        if (alternativeSolution == null) {
            System.out.println("\t\tNo Solution!");
        }

        List<Rectangle2D> movingPaths = generateMovingPath(movingBoxIndex, alternativeSolution);
        Set<Integer> movingObstacleIndexes = getMovingObstaclesIndexes(movingPaths, state.movingObstacles);

        for (int j : movingObstacleIndexes) {

            System.out.println("\t\tFind Solution for MovingObstacle " + j);
            movingObstacleAgent = new MovingObstacleAgent(ps, state, j, movingPaths);
            List<State> solution = movingObstacleAgent.search();

            if (solution == null) {
                System.out.println("\t\t\tNo Solution!");
                hasSolution = false;
            } else {
                states.addAll(solution);
                state = states.get(states.size() - 1);
            }

        }
        if (hasSolution) {
            return states;
        } else {
            return null;
        }

    }

    private static List<State> moveRobotToMovingBox(State state, int movingBoxIndex) {
        System.out.println("Find robot path to MovingBox: " + movingBoxIndex);
        Box movingBox = state.movingBoxes.get(movingBoxIndex);;
        return moveRobotToBox(state, movingBox);
    }

    private static List<State> moveRobotToMovingObstacle(State state, int movingObstacleIndex) {
        System.out.println("Find robot path to MovingObstacle: " + movingObstacleIndex);
        Box movingBox = state.movingObstacles.get(movingObstacleIndex);
        return moveRobotToBox(state, movingBox);
    }

    private static List<State> moveRobotToBox(State state, Box movingBox) {
        List<State> states = new LinkedList<>();
        if (tester.isCoupled(state.robotConfig, movingBox) < 0) {
            State robotState = new State(state.robotConfig, state.movingBoxes, state.movingObstacles);
            robotAgent = new RobotAgent(ps, robotState, movingBox);
            List<State> solution = robotAgent.search();
            if (solution == null) {
                System.out.println("\t\t\tNo Solution!");
            } else {
                states.addAll(solution);
            }
        }
        return states;
    }

    private static void writeOutputFile(String fileName) {

        List<String> output = new LinkedList<>();
        for (State s : solutionStates) {
            output.add(s.toString());
        }

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

            Point2D currentCenter = new Point2D.Double(currentBox.getPos().getX() + currentBox.getWidth()/2,
                    currentBox.getPos().getY() + currentBox.getWidth()/2);
            Point2D nextCenter = new Point2D.Double(nextBox.getPos().getX() + nextBox.getWidth()/2,
                    nextBox.getPos().getY() + nextBox.getWidth()/2);

            Line2D line = new Line2D.Double(currentCenter, nextCenter);

            Rectangle2D rectangle = tester.grow(line.getBounds2D(), ps.getRobotWidth() / 2);

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
