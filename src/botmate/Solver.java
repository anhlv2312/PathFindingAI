package botmate;

import problem.Box;
import problem.ProblemSpec;
import problem.RobotConfig;
import tester.Tester;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

public class Solver {

    private static ProblemSpec ps;
    private static RobotAgent robotAgent;
    private static BoxAgent boxAgent;
    private static ObstacleAgent obstacleAgent;
    private static Tester tester;
    private static State initialState, currentState;
    private static List<State> solutionStates = new LinkedList<>();
    private static double boxStepWidth, obstacleStepWidth;

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

        initialState = new State(ps.getInitialRobotConfig(), ps.getMovingBoxes(), ps.getMovingObstacles());
        currentState = initialState;
        solutionStates.add(initialState);

        boxStepWidth = ps.getRobotWidth() - Tester.MAX_BASE_STEP;
        obstacleStepWidth = boxStepWidth;

        System.out.println("Start solving problem");

        solveProblem();
        writeOutputFile(args[1]);
    }

    private static void solveProblem() {
        List<State> solution;

        int goalCount = 0;
        int retry = 5;

        while (goalCount < ps.getMovingBoxes().size() && retry > 0) {
            for (int movingBoxIndex = 0; movingBoxIndex < ps.getMovingBoxes().size(); movingBoxIndex++) {

                System.out.println("Solving MovingBox: " + movingBoxIndex);
                Point2D movingBoxGoal = ps.getMovingBoxEndPositions().get(movingBoxIndex);
                solution = findDirectSolution(currentState, movingBoxIndex, movingBoxGoal);

                if (solution != null) {
                    addStatesToSolution(solution);
                    goalCount ++;
                } else {
                    System.out.println("\tNo direct solution, find alternatives");
                    List<State> alternativeSolution = findAlternativeSolution(currentState, movingBoxIndex, movingBoxGoal);

                    if (alternativeSolution != null) {
                        addStatesToSolution(alternativeSolution);
                        System.out.println("\tFind solution again");
                        solution = findDirectSolution(currentState, movingBoxIndex, movingBoxGoal);
                        if (solution != null) {
                            addStatesToSolution(solution);
                            goalCount++;
                        } else {
                            System.out.println("\t\tStill no Solution!");
                        }
                    } else {
                        System.out.println("\t\tNo alternative solution!");
                    }
                }
            }
            retry--;
            boxStepWidth = boxStepWidth/2 - Tester.MAX_BASE_STEP;
            obstacleStepWidth = boxStepWidth;
        }
    }


    private static void addStatesToSolution(List<State> solution) {
        if (solution != null) {
            solutionStates.addAll(solution);
        }
        currentState = solutionStates.get(solutionStates.size() - 1);
    }


    private static List<State> findDirectSolution(State state, int movingBoxIndex, Point2D movingBoxGoal) {
        List<State> states = new LinkedList<>();

        boxAgent = new BoxAgent(ps, state, movingBoxIndex, movingBoxGoal, boxStepWidth, true);
        List<State> solution = boxAgent.search();

        if (solution == null) {
            return null;
        } else {
            State firstState = solution.get(0);
            int targetEdge = tester.isCoupled(firstState.robotConfig, firstState.movingBoxes.get(movingBoxIndex));
            List<State> robotSolution = findPathToMovingBox(state, movingBoxIndex, targetEdge);
            if (robotSolution != null) {
                states.addAll(robotSolution);
                states.addAll(solution);
            } else {
                System.out.println("\t\t\tNo solution for robot!");
            }
        }
        return states;
    }

    private static List<State> findAlternativeSolution(State state, int movingBoxIndex, Point2D movingBoxGoal) {

        State tempState = state;

        List<State> states = new LinkedList<>();
        boxAgent = new BoxAgent(ps, tempState, movingBoxIndex, movingBoxGoal, boxStepWidth, false);
        List<State> alternativeSolution = boxAgent.search();

        if (alternativeSolution != null) {
            List<Rectangle2D> movingPaths = generateMovingPath(movingBoxIndex, alternativeSolution);
            Set<Integer> movingObstacleIndexes = getObstaclesIndexes(movingPaths, tempState.movingObstacles);
            for (int movingObstacleIndex : movingObstacleIndexes) {
                System.out.println("\t\tMovingObstacle " + movingObstacleIndex);
                obstacleAgent = new ObstacleAgent(ps, tempState, movingObstacleIndex, obstacleStepWidth, movingPaths);
                List<State> solution = obstacleAgent.search();
                if (solution == null) {
                    System.out.println("\t\t\tUnable to move!");
                    return null;
                } else {

                    State firstState = solution.get(0);
                    int targetEdge = tester.isCoupled(firstState.robotConfig, firstState.movingObstacles.get(movingObstacleIndex));
                    List<State> robotSolution = findPathToObstacle(tempState, movingObstacleIndex, targetEdge);
                    if (robotSolution != null) {
                        states.addAll(robotSolution);
                        states.addAll(solution);
                    } else {
                        System.out.println("\t\t\tNo solution for robot!");
                    }

                    tempState = states.get(states.size() - 1);
                }
            }
        }
        return states;
    }

    private static List<State> findPathToMovingBox(State state, int movingBoxIndex, int targetEdge) {
        System.out.println("\t\tFind robot path");
        Box movingBox = state.movingBoxes.get(movingBoxIndex);
        return findPathToBox(state, movingBox, targetEdge);
    }

    private static List<State> findPathToObstacle(State state, int movingObstacleIndex, int targetEdge) {
        System.out.println("\t\t\tFind robot path");
        Box movingBox = state.movingObstacles.get(movingObstacleIndex);
        return findPathToBox(state, movingBox, targetEdge);
    }

    private static List<State> findPathToBox(State state, Box movingBox, int targetEdge) {
        List<State> states = new LinkedList<>();
        if (tester.isCoupled(state.robotConfig, movingBox) < 0) {
            State robotState = new State(state.robotConfig, state.movingBoxes, state.movingObstacles);
            robotAgent = new RobotAgent(ps, robotState, movingBox, targetEdge);
            List<State> solution = robotAgent.search();
            if (solution == null) {
                return null;
            } else {
                states.addAll(solution);
            }
        }
        return states;
    }

    private static void writeOutputFile(String fileName) {

        List<String> output = new LinkedList<>();

        Iterator<State> stateIterator = solutionStates.iterator();
        State currentState = initialState;
        while (stateIterator.hasNext()) {
            State nextState = stateIterator.next();
//            output.add(currentState.toString());
            output.addAll(generateMoves(currentState, nextState));
            currentState = nextState;
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

    public static Set<Integer> getObstaclesIndexes(List<Rectangle2D> movingPaths, List<Box> movingObstacles) {
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


    public static int getMovingBOxDirection(int movingBoxIndex, State state1, State state2) {

        Box box1 = state1.movingBoxes.get(movingBoxIndex);
        Box box2 = state2.movingBoxes.get(movingBoxIndex);

        if (box2.getPos().getX() > box1.getPos().getX()) {
            return 2;
        } else if (box2.getPos().getX() < box1.getPos().getX()) {
            return 4;
        } else if (box2.getPos().getY() > box1.getPos().getY()) {
            return 1;
        } else if (box2.getPos().getY() < box1.getPos().getY()) {
            return 3;
        }
        return 1;
    }



    private static List<String> generateMoves(State state1, State state2) {

        List<String> result = new LinkedList<>();
        result.add(state1.toString());
        State tempState = state1;

        RobotConfig r1 = state1.robotConfig;
        RobotConfig r2 = state2.robotConfig;
        Double numberOfSteps;

        double angle = tester.normaliseAngle(r2.getOrientation()) - tester.normaliseAngle(r1.getOrientation());
        if (angle != 0) {
            numberOfSteps = (angle * ps.getRobotWidth() / 2) / Tester.MAX_BASE_STEP;
            double deltaO = angle / numberOfSteps;
            for (int i = 0; i < numberOfSteps; i++) {
                tempState = tempState.moveRobot(0, 0, deltaO);
                result.add(tempState.toString());
            }
        }

        tempState = tempState.moveRobotToPosition(r1.getPos(), r2.getOrientation());
        result.add(tempState.toString());

        numberOfSteps = Math.ceil(r1.getPos().distance(r2.getPos()) / Tester.MAX_BASE_STEP);
        double deltaX = (r2.getPos().getX() - r1.getPos().getX()) / numberOfSteps;
        double deltaY = (r2.getPos().getY() - r1.getPos().getY()) / numberOfSteps;

        for (int i = 0; i < numberOfSteps; i++) {
            tempState = tempState.moveRobot(deltaX, deltaY, 0);
            result.add(tempState.toString());
        }

        result.add(state2.toString());
        return result;
    }
//
//    // Get the next edge that the robot need to get to
//    public static int getDirection(State state1, State state2) {
//
//        Box box1 = state1.getMovingBox();
//        Box box2 = state2.getMovingBox();
//
//        if (box2.getPos().getX() > box1.getPos().getX()) {
//            return 2;
//        } else if (box2.getPos().getX() < box1.getPos().getX()) {
//            return 4;
//        } else if (box2.getPos().getY() > box1.getPos().getY()) {
//            return 1;
//        } else if (box2.getPos().getY() < box1.getPos().getY()) {
//            return 3;
//        }
//        return 1;
//    }
}
