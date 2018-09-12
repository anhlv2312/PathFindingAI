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

    private static final double MIN_STEP_SIZE = 0.01;
    private static final double MAX_TIMEOUT = 115;

    private static ProblemSpec ps;
    private static RobotAgent robotAgent;
    private static BoxAgent boxAgent;
    private static ObstacleAgent obstacleAgent;
    private static Tester tester;
    private static State initialState, globalCurrentState;
    private static List<State> solutionStates = new LinkedList<>();

    private static Set<Integer> solvedMovingBoxes;
    private static double stepSize;

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
        globalCurrentState = initialState;
        solutionStates.add(initialState);

        solvedMovingBoxes = new HashSet<>();
        stepSize = ps.getRobotWidth();

        System.out.println("Number of MovingBox: " + ps.getMovingBoxes().size());
        System.out.println("Number of Obstacle: " + ps.getMovingObstacles().size());
        double startTime = System.currentTimeMillis();
        double timer = 0;

        while (solvedMovingBoxes.size() < ps.getMovingBoxes().size() && timer < MAX_TIMEOUT) {
            stepSize = Math.round(stepSize * 100) / 100.0;
            System.out.println();
            System.out.println(String.format("Start solving with step size: %.2f", stepSize));
            for (int movingBoxIndex = 0; movingBoxIndex < ps.getMovingBoxes().size(); movingBoxIndex++ ) {
                if (!solvedMovingBoxes.contains(movingBoxIndex)) {
                    solveMovingBox(movingBoxIndex);
                }
            }
            if (stepSize <= MIN_STEP_SIZE) {
                stepSize = ps.getRobotWidth();
            } else {
                stepSize = stepSize/2;
            }
            timer = (System.currentTimeMillis() - startTime)/1000;
        }

        System.out.println();
        System.out.println(String.format("Total time: %.2f seconds", (System.currentTimeMillis() - startTime)/1000));

        writeOutputFile(args[1]);
    }

    private static boolean solveMovingBox(int movingBoxIndex) {

        System.out.println("    Solving MovingBox: " + movingBoxIndex);
        Point2D movingBoxGoal = ps.getMovingBoxEndPositions().get(movingBoxIndex);

        boxAgent = new BoxAgent(ps, globalCurrentState, movingBoxIndex, movingBoxGoal, stepSize);
        List<State> boxSolution = boxAgent.search();

        if (boxSolution != null) {
            List<Rectangle2D> movingPaths = generateMovingPath(movingBoxIndex, boxSolution);
            Set<Integer> obstacleIndexes = getObstaclesIndexes(movingPaths, globalCurrentState.movingObstacles);

            int obstacleCount = obstacleIndexes.size();

            for (int obstacleIndex : obstacleIndexes) {
                System.out.println("        Solving Obstacle: " + obstacleIndex);
                obstacleAgent = new ObstacleAgent(ps, globalCurrentState, obstacleIndex, stepSize, movingPaths);
                List<State> obstacleSolution = obstacleAgent.search();
                if (obstacleSolution == null) {
                    System.out.println("            Unable to move Obstacle!");
                    break;
                } else {
                    List<State> robotSolution = generateRobotToObstacle(globalCurrentState, obstacleIndex, obstacleSolution);
                    if (robotSolution != null) {
                        solutionStates.addAll(robotSolution);
                        globalCurrentState = solutionStates.get(solutionStates.size() - 1);
                    } else {
                        System.out.println("            No solution for Robot!");
                        break;
                    }
                }
                obstacleCount--;
            }

            if (obstacleCount == 0) {
                boxAgent = new BoxAgent(ps, globalCurrentState, movingBoxIndex, movingBoxGoal, stepSize);
                boxSolution = boxAgent.search();
                if (boxSolution != null) {
                    if (moveMovingBoxToGoal(globalCurrentState, movingBoxIndex, boxSolution)) {
                        solvedMovingBoxes.add(movingBoxIndex);
                        return true;
                    }
                }
            } else {
                System.out.println("        Skip MovingBox: " + movingBoxIndex);
            }
            globalCurrentState = solutionStates.get(solutionStates.size() - 1);
        } else {
            System.out.println("        No solution for MovingBox: " + movingBoxIndex);
        }
        return false;
    }


//    private static int findNearestMovingBox(State, )

    private static boolean moveMovingBoxToGoal(State currentState, int movingBoxIndex, List<State> boxSolution) {
        List<State> robotSolution = generateRobotToMovingBox(currentState, movingBoxIndex, boxSolution);
        if (robotSolution != null) {
            solutionStates.addAll(robotSolution);
            globalCurrentState = solutionStates.get(solutionStates.size() - 1);
            return true;
        } else {
            System.out.println("        No solution for Robot!");
            return false;
        }
    }

    private static List<Rectangle2D> generateMovingPath(int movingBoxIndex, List<State> movingStates) {

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

    private static Set<Integer> getObstaclesIndexes(List<Rectangle2D> movingPaths, List<Box> movingObstacles) {
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

    private static List<State> generateRobotToMovingBox(State firstState, int movingBoxIndex, List<State> solution) {
        Iterator<State> stateIterator = solution.iterator();
        List<State> states = new LinkedList<>();

        System.out.println("        Find path for Robot: " + solution.size() + " steps");

        State currentState = firstState;
        while (stateIterator.hasNext()) {
            State nextState = stateIterator.next();

            Box currentBox = currentState.movingBoxes.get(movingBoxIndex);
            int nextEdge = tester.isCoupled(nextState.robotConfig, nextState.movingBoxes.get(movingBoxIndex));

            RobotConfig nextRobotConfig = currentState.moveRobotToBox(currentBox, nextEdge).robotConfig;
            robotAgent = new RobotAgent(ps, currentState, nextRobotConfig, currentBox);
            List<State> robotSolution = robotAgent.search();
            if (robotSolution == null) {
                return null;
            } else {
                states.addAll(robotSolution);
                states.add(nextState);
            }
            currentState = nextState;
        }
        return states;
    }

    private static List<State> generateRobotToObstacle(State firstState, int obstacleIndex, List<State> solution) {

        Iterator<State> stateIterator = solution.iterator();
        List<State> states = new LinkedList<>();
        System.out.println("            Find path for Robot: " + solution.size() + " steps");
        State currentState = firstState;
        while (stateIterator.hasNext()) {

            State nextState = stateIterator.next();
            Box currentBox = currentState.movingObstacles.get(obstacleIndex);
            int nextEdge = tester.isCoupled(nextState.robotConfig, nextState.movingObstacles.get(obstacleIndex));
            RobotConfig nextRobotConfig = currentState.moveRobotToBox(currentBox, nextEdge).robotConfig;
            robotAgent = new RobotAgent(ps, currentState, nextRobotConfig, currentBox);
            List<State> robotSolution = robotAgent.search();
            if (robotSolution == null) {
                return null;
            } else {
                states.addAll(robotSolution);
                states.add(nextState);
            }
            currentState = nextState;
        }
        return states;
    }

    private static List<String> generateMoves(State currentState, State nextState) {

        List<String> result = new LinkedList<>();

        State tempState = currentState;

        RobotConfig r1 = currentState.robotConfig;
        RobotConfig r2 = nextState.robotConfig;
        double newOrientation = nextState.robotConfig.getOrientation();
        Double numberOfSteps;

        double angle = tester.normaliseAngle(r2.getOrientation()) - tester.normaliseAngle(r1.getOrientation());
        if (angle != 0) {
            numberOfSteps = Math.abs(angle) * ps.getRobotWidth() / 2 / Tester.MAX_BASE_STEP;
            double deltaO = angle / numberOfSteps;
            for (int i = 0; i < numberOfSteps; i++) {
                tempState = tempState.moveRobot(0, 0, deltaO);
                result.add(tempState.toString());
            }
            tempState = tempState.moveRobotToPosition(r1.getPos(), newOrientation);
            result.add(tempState.toString());
        }

        numberOfSteps = Math.ceil(r1.getPos().distance(r2.getPos()) / Tester.MAX_BASE_STEP);
        double deltaX = (r2.getPos().getX() - r1.getPos().getX()) / numberOfSteps;
        double deltaY = (r2.getPos().getY() - r1.getPos().getY()) / numberOfSteps;

        for (int i = 0; i < numberOfSteps; i++) {

            tempState = tempState.moveRobot(deltaX, deltaY, 0);

            for (int j = 0; j < tempState.movingBoxes.size(); j++) {
                int currentPosition = tester.isCoupled(r1, currentState.movingBoxes.get(j));
                int nextPosition = tester.isCoupled(r2, nextState.movingBoxes.get(j));
                if ((currentPosition == 1 || currentPosition == 3 ) && (nextPosition == 1 || nextPosition == 3 )) {
                    tempState = tempState.moveMovingBox(j, 0, deltaY, nextPosition);
                }
                if ((currentPosition == 2 || currentPosition == 4 ) && (nextPosition == 2 || nextPosition == 4 )) {
                    tempState = tempState.moveMovingBox(j, deltaX, 0, nextPosition);
                }
            }

            for (int j = 0; j < tempState.movingObstacles.size(); j++) {
                int currentPosition = tester.isCoupled(r1, currentState.movingObstacles.get(j));
                int nextPosition = tester.isCoupled(r2, nextState.movingObstacles.get(j));
                if ((currentPosition == 1 || currentPosition == 3 ) && (nextPosition == 1 || nextPosition == 3 )) {
                    tempState = tempState.moveObstacle(j, 0, deltaY, nextPosition);
                }
                if ((currentPosition == 2 || currentPosition == 4 ) && (nextPosition == 2 || nextPosition == 4 )) {
                    tempState = tempState.moveObstacle(j, deltaX, 0, nextPosition);
                }
            }

            result.add(tempState.toString());
        }

        result.add(nextState.toString());
        return result;
    }

    private static void writeOutputFile(String fileName) {

        List<String> output = new LinkedList<>();

        State currentState = initialState;
        Iterator<State> stateIterator = solutionStates.iterator();
        while (stateIterator.hasNext()) {
            State nextState = stateIterator.next();
            output.addAll(generateMoves(currentState, nextState));
            currentState = nextState;
        }

        try {
            FileWriter fw = new FileWriter(fileName);
            BufferedWriter bw = new BufferedWriter(fw);

            System.out.println("Number of Steps: " + output.size());
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
