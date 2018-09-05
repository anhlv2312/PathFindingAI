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

        states.add(initialState);
        List<State> solution;

//        List<Rectangle2D> movingPaths = new ArrayList<>();
//
//        movingPaths.add(new Rectangle2D.Double(0.2, 0.65, 0.6, 0.1));
////        movingPaths.add(new Rectangle2D.Double(0.2, 0.2, 0.6, 0.1));
//
//        movingObstacleAgent = new MovingObstacleAgent(ps, currentState, 0, movingPaths);
//        solution = movingObstacleAgent.search();
//
//        System.out.println(solution);


//        java.awt.geom.Rectangle2D$Double[x=0.4625,y=0.6625,w=0.075,h=0.075]
//        java.awt.geom.Rectangle2D$Double[x=0.1499,y=0.3499,w=0.7001999999999999,h=0.10020000000000007]
//
//        Rectangle2D a = new Rectangle2D.Double(0.4625, 0.6625, 0.075, 0.075);
//        Rectangle2D b = new Rectangle2D.Double(0.1499, 0.3499, 0.7001999999999999, 0.10020000000000007);
//
//        System.out.println(b.contains(new Point2D.Double(0.4625, 0.6625)));



        for (int movingBoxIndex = 0; movingBoxIndex < ps.getMovingBoxes().size(); movingBoxIndex++) {

//            movingBox = currentState.movingBoxes.get(movingBoxIndex);
//
//            if (tester.isCoupled(currentState.robotConfig, movingBox) < 0) {
//                System.out.println("Find robot path to MovingBox: " + movingBoxIndex);
//                State robotState = new State(currentState.robotConfig, currentState.movingBoxes, currentState.movingObstacles);
//                robotAgent = new RobotAgent(ps, robotState, movingBox);
//                solution = robotAgent.search();
//                if (solution != null) {
//                    states.addAll(solution);
//                } else {
//                    System.out.println("No Solution");
//                }
//                currentState = states.get(states.size() - 1);
//            }



            System.out.println("Find direct solution for MovingBox: " + movingBoxIndex);
            Point2D movingBoxGoal = ps.getMovingBoxEndPositions().get(movingBoxIndex);
            movingBoxAgent = new MovingBoxAgent(ps, currentState, movingBoxIndex, movingBoxGoal, true);
            solution = movingBoxAgent.search();

            if (solution != null) {
                states.addAll(solution);
                currentState = states.get(states.size() - 1);
            } else {

                System.out.println("\tNo direct solution! find another solution");

                List<State> alternativeSolution = findAlternativeSolution(currentState, movingBoxIndex, movingBoxGoal);

                if (alternativeSolution == null) {
                    System.out.println("\tNo another solution!");
                } else {
                    states.addAll(alternativeSolution);
                    currentState = states.get(states.size() - 1);
                }

                System.out.println("Find solution again for: " + movingBoxIndex);
                movingBoxAgent = new MovingBoxAgent(ps, currentState, movingBoxIndex, movingBoxGoal, true);
                solution = movingBoxAgent.search();

                if (solution != null) {
                    states.addAll(solution);
                }

//                states.addAll(movingStates);
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



    private static List<State> findAlternativeSolution(State currentState, int movingBoxIndex, Point2D movingBoxGoal) {

        boolean hasSolution = true;
        List<State> states = new LinkedList<>();

        movingBoxAgent = new MovingBoxAgent(ps, currentState, movingBoxIndex, movingBoxGoal, false);
        List<State> alternativeSolution = movingBoxAgent.search();

        if (alternativeSolution == null) {
            System.out.println("\tNo Solution!");
        }

        List<Rectangle2D> movingPaths = generateMovingPath(movingBoxIndex, alternativeSolution);
        Set<Integer> movingObstacleIndexes = getMovingObstaclesIndexes(movingPaths, currentState.movingObstacles);

        for (int j : movingObstacleIndexes) {

//            movingBox = currentState.movingObstacles.get(j);
//            if (tester.isCoupled(currentState.robotConfig, movingBox) < 0) {
//                System.out.println("Find robot path to MovingObstacle: " + j);
//                State robotState = new State(currentState.robotConfig, currentState.movingBoxes, currentState.movingObstacles);
//                robotAgent = new RobotAgent(ps, robotState, movingBox);
//                solution = robotAgent.search();
//                if (solution != null) {
//                    states.addAll(solution);
//                } else {
//                    System.out.println("No Solution for Robot");
//                }
//                currentState = states.get(states.size() - 1);
//            }

            System.out.println("\tFind Solution for MovingObstacle " + j);
            movingObstacleAgent = new MovingObstacleAgent(ps, currentState, j, movingPaths);
            List<State> solution = movingObstacleAgent.search();

            if (solution == null) {
                System.out.println("\t\tNo Solution!");
                hasSolution = false;
            } else {
                states.addAll(solution);
                currentState = states.get(states.size() - 1);
            }

        }
        if (hasSolution) {
            return states;
        } else {
            return null;
        }

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
