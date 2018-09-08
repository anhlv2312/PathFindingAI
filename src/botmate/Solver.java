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
        currentState = initialState;
        solutionStates.add(initialState);

        stepSize = ps.getRobotWidth();

        System.out.println("Number of MovingBox: " + ps.getMovingBoxes().size());
        System.out.println("Number of Obstacle: " + ps.getMovingObstacles().size());
        double startTime = System.currentTimeMillis();

        solveProblem();

        System.out.println();
        System.out.println(String.format("Total time: %.2f seconds", (System.currentTimeMillis() - startTime)/1000));

        writeOutputFile(args[1]);
    }

    private static void solveProblem() {

        Set<Integer> solvedMovingBoxes = new HashSet<>();

        while (solvedMovingBoxes.size() < ps.getMovingBoxes().size() && stepSize > 0) {

            System.out.println();
            System.out.println(String.format("Start solving with step size: %.2f", stepSize));

            for (int movingBoxIndex = 0; movingBoxIndex < ps.getMovingBoxes().size(); movingBoxIndex++ ) {

                if (solvedMovingBoxes.contains(movingBoxIndex)) {
                    continue;
                }

                System.out.println("\tSolving MovingBox: " + movingBoxIndex);
                Point2D movingBoxGoal = ps.getMovingBoxEndPositions().get(movingBoxIndex);

                boxAgent = new BoxAgent(ps, currentState, movingBoxIndex, movingBoxGoal, stepSize);
                List<State> boxSolution = boxAgent.search();

                if (boxSolution != null) {

                    List<Rectangle2D> movingPaths = generateMovingPath(movingBoxIndex, boxSolution);
                    Set<Integer> obstacleIndexes = getObstaclesIndexes(movingPaths, currentState.movingObstacles);

                    if (obstacleIndexes.isEmpty()) {
                        State firstState = boxSolution.get(0);
                        int targetEdge = tester.isCoupled(firstState.robotConfig, firstState.movingBoxes.get(movingBoxIndex));
                        List<State> robotSolution = findPathToMovingBox(currentState, movingBoxIndex, targetEdge);
                        if (robotSolution != null) {
                            solutionStates.addAll(robotSolution);
                            solutionStates.addAll(generateSlideToMovingBox(movingBoxIndex, boxSolution));
                            solvedMovingBoxes.add(movingBoxIndex);
                        } else {
                            System.out.println("\tNo solution for Robot!");
                        }
                        currentState = solutionStates.get(solutionStates.size() - 1);
                    } else {
                        int obstacleCount = obstacleIndexes.size();
                        for (int obstacleIndex : obstacleIndexes) {
                            System.out.println("\t\tSolving Obstacle: " + obstacleIndex);
                            obstacleAgent = new ObstacleAgent(ps, currentState, obstacleIndex, stepSize, movingPaths);
                            List<State> obstacleSolution = obstacleAgent.search();
                            if (obstacleSolution == null) {
                                System.out.println("\t\t\tUnable to move Obstacle!");
                                break;
                            } else {
                                State firstState = obstacleSolution.get(0);
                                int targetEdge = tester.isCoupled(firstState.robotConfig, firstState.movingObstacles.get(obstacleIndex));
                                List<State> robotSolution = findPathToObstacle(currentState, obstacleIndex, targetEdge);
                                if (robotSolution != null) {
                                    solutionStates.addAll(robotSolution);
                                    solutionStates.addAll(generateSlideToObstacle(obstacleIndex, obstacleSolution));
                                } else {
                                    System.out.println("\t\t\t\tNo solution for Robot!");
                                    break;
                                }
                                currentState = solutionStates.get(solutionStates.size() - 1);
                            }
                            obstacleCount--;
                        }

                        if (obstacleCount == 0) {
                            boxAgent = new BoxAgent(ps, currentState, movingBoxIndex, movingBoxGoal, stepSize);
                            boxSolution = boxAgent.search();

                            State firstState = boxSolution.get(0);
                            int targetEdge = tester.isCoupled(firstState.robotConfig, firstState.movingBoxes.get(movingBoxIndex));
                            List<State> robotSolution = findPathToMovingBox(currentState, movingBoxIndex, targetEdge);
                            if (robotSolution != null) {
                                solutionStates.addAll(robotSolution);
                                solutionStates.addAll(generateSlideToMovingBox(movingBoxIndex, boxSolution));
                                solvedMovingBoxes.add(movingBoxIndex);
                            } else {
                                System.out.println("\t\tNo solution for Robot!");
                            }
                        } else {
                            System.out.println("\t\tSkip MovingBox: " + movingBoxIndex);
                        }
                        currentState = solutionStates.get(solutionStates.size() - 1);
                    }
                    currentState = solutionStates.get(solutionStates.size() - 1);
                }

            }

            stepSize = stepSize - ps.getRobotWidth()/10;

        }
    }

    private static List<State> findPathToMovingBox(State state, int movingBoxIndex, int targetEdge) {
        System.out.println("\t\tFind path to MovingBox: " + movingBoxIndex);
        Box movingBox = state.movingBoxes.get(movingBoxIndex);
        return findPathToBox(state, movingBox, targetEdge);
    }

    private static List<State> findPathToObstacle(State state, int obstacleIndex, int targetEdge) {
        System.out.println("\t\t\tFind path to Obstacle: " + obstacleIndex);
        Box movingBox = state.movingObstacles.get(obstacleIndex);
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

    private static List<String> generateMoves(State state1, State state2) {

        List<String> result = new LinkedList<>();
        result.add(state1.toString());
        State tempState = state1;

        RobotConfig r1 = state1.robotConfig;
        RobotConfig r2 = state2.robotConfig;
        double newOrientation = state2.robotConfig.getOrientation();
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

//
//    public static int getMovingBoxDirection(int movingBoxIndex, State state1, State state2) {
//
//        Box box1 = state1.movingBoxes.get(movingBoxIndex);
//        Box box2 = state2.movingBoxes.get(movingBoxIndex);
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


    public static List<State> generateSlideToMovingBox(int movingBoxIndex, List<State> solution) {

        List<State> steps = new LinkedList<>();
        Iterator<State> stateIterator = solution.iterator();

        if (stateIterator.hasNext()) {
            State currentState = stateIterator.next();
            while (stateIterator.hasNext()) {
                State nextState = stateIterator.next();

                int currentEdge = tester.isCoupled(currentState.robotConfig, currentState.movingBoxes.get(movingBoxIndex));
                int nextEdge = tester.isCoupled(nextState.robotConfig, nextState.movingBoxes.get(movingBoxIndex));

                steps.add(currentState);
                steps.addAll(slideRobot(currentState, currentEdge, nextEdge));
                currentState = nextState;
            }
            steps.add(currentState);
        }
        return steps;
    }

    public static List<State> generateSlideToObstacle(int obstacleIndex, List<State> solution) {

        List<State> steps = new LinkedList<>();
        Iterator<State> stateIterator = solution.iterator();

        if (stateIterator.hasNext()) {
            State currentState = stateIterator.next();
            while (stateIterator.hasNext()) {
                State nextState = stateIterator.next();

                int currentEdge = tester.isCoupled(currentState.robotConfig, currentState.movingObstacles.get(obstacleIndex));
                int nextEdge = tester.isCoupled(nextState.robotConfig, nextState.movingObstacles.get(obstacleIndex));

                steps.add(currentState);
                steps.addAll(slideRobot(currentState, currentEdge, nextEdge));
                currentState = nextState;
            }
            steps.add(currentState);
        }
        return steps;
    }

    public static List<State> slideRobot(State currentState, int currentEdge, int nextEdge) {

        List<State> steps = new ArrayList<>();

        double s = ps.getRobotWidth()/2;
        double x = Math.PI/2;

        if (currentEdge != nextEdge) {

            if (currentEdge == 1) {
                steps.add(currentState.moveRobot(0, -s, 0));
                steps.add(currentState.moveRobot(0, -s, x));
                if (nextEdge == 2) {
                    steps.add(currentState.moveRobot(-s, -s, x));
                    steps.add(currentState.moveRobot(-s, s, x));
                } else if (nextEdge == 4) {
                    steps.add(currentState.moveRobot(s, -s, x));
                    steps.add(currentState.moveRobot(s, s, x));

                }
            }
            if (currentEdge == 2) {
                steps.add(currentState.moveRobot(-s, 0, 0));
                steps.add(currentState.moveRobot(-s, 0, -x));
                if (nextEdge == 1) {
                    steps.add(currentState.moveRobot(-s, -s, -x));
                    steps.add(currentState.moveRobot(s, -s, -x));
                } else if (nextEdge == 3) {
                    steps.add(currentState.moveRobot(-s, s, -x));
                    steps.add(currentState.moveRobot(s, s, -x));
                }
            }
            if (currentEdge == 3) {
                steps.add(currentState.moveRobot(0, s, 0));
                steps.add(currentState.moveRobot(0, s, x));
                if (nextEdge == 2) {
                    steps.add(currentState.moveRobot(-s, s, x));
                    steps.add(currentState.moveRobot(-s, -s, x));
                } else if (nextEdge == 4) {
                    steps.add(currentState.moveRobot(s, s, x));
                    steps.add(currentState.moveRobot(s, -s, x));
                }
            }

            if (currentEdge == 4) {
                steps.add(currentState.moveRobot(s, 0, 0));
                steps.add(currentState.moveRobot(s, 0, -x));
                if (nextEdge == 1) {
                    steps.add(currentState.moveRobot(s, -s, -x));
                    steps.add(currentState.moveRobot(-s, -s, -x));
                } else if (nextEdge == 3) {
                    steps.add(currentState.moveRobot(s, s, -x));
                    steps.add(currentState.moveRobot(-s, s, -x));
                }

            }
        }


        return steps;
    }


}
