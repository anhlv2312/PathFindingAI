package botmate;

import problem.Box;
import problem.ProblemSpec;
import problem.RobotConfig;
import problem.StaticObstacle;
import tester.Tester;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

public class BotMateSolver {

    private static final int ROBOT_RANDOM_SAMPLES = 50;

    static ProblemSpec ps;
    static Tester tester;
    static BotMateAgent agent = new BotMateAgent();
    static double robotWidth;

    public static void main(String args[]) {

        try {
            ps = new ProblemSpec();
            ps.loadProblem("bot.input1.txt");
            robotWidth = ps.getRobotWidth();
            tester = new Tester(ps);
        } catch (IOException ex) {
            System.out.println("IO Exception occurred");
        }

        // State variables
        BotMateState initialState, currentState, nextState, goalState;

        // Goal variable
        Point2D movingBoxGoal;

        // Solution variable, each solution for each box is a list
        List<List<BotMateState>> solutions = new LinkedList<>();

        // Store the solution states that is combined with robots movement
        List<BotMateState> moveStates = new LinkedList<>();

        // Store the initial state (for later use)
        initialState = new BotMateState(0,
                ps.getInitialRobotConfig(),
                ps.getMovingBoxes(),
                ps.getMovingObstacles(),
                tester);

        currentState = initialState;


        for (int i = 0; i < ps.getMovingBoxes().size(); i++) {

            // Initialize a solution list for it
            List<BotMateState> solution = new LinkedList<>();

            // Set the movingBoxIndex so we know what box we are moving
            currentState = currentState.changeMovingBox(i);

            // Add the start State (the cost is not important)
            solution.add(currentState);

            // Get the goal position of the box
            movingBoxGoal = ps.getMovingBoxEndPositions().get(i);

            // Set the goal state by moving the box to the goal
            goalState = currentState.moveMovingBox(movingBoxGoal);

            // Search for solution
            System.out.println("Find solution to move box: " + i);
            solution.addAll(agent.search(currentState, goalState, false));

            // Update current State
            // Add successors always add a position to the last
            BotMateState lastState = solution.get(solution.size()-2);
            currentState = lastState.moveRobotOut();

            solution.add(lastState);

            // Add solution to the list
            solutions.add(solution);

        }

        // For each solution set for each box
        for (List<BotMateState> solution : solutions) {

            BotMateState firstState = solution.get(0);
            BotMateState secondState = solution.get(1);

            int direction = getDirection(firstState, secondState);
            BotMateState robotGoal = firstState.moveRobotToMovingBox(direction);

            System.out.println("Current Moving: " + firstState.getMovingBoxIndex());
            robotGoal = robotGoal.moveRobotOut();

            moveStates.add(firstState);

            List<BotMateState> a = moveRobotToBox(firstState, robotGoal);
            System.out.println(a.size());
            moveStates.addAll(a);

            moveStates.add(firstState.moveRobotToMovingBox(direction));

            // For each state in the solution
            for (int i = 1; i < solution.size() - 2; i++) {

                // Get two continuous steps,
                currentState = solution.get(i);
                nextState = solution.get(i + 1);

                if (i < solution.size() - 3) {
                    moveStates.addAll(slideRobot(currentState, nextState));
                } else {
                    moveStates.add(currentState);
                }
            }

        }


        // Generate output strings
        List<String> output = new LinkedList<>();

        currentState = initialState;
        for (BotMateState s : moveStates) {
//            output.add(s.outputString());
            output.addAll(generateMoves(currentState, s));
            currentState = s;
        }

        // Write to files
        try {
            FileWriter fw = new FileWriter("bot.output.txt");
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


    private static List<BotMateState> PRMForRobot(BotMateState state, int numberOfSample) {
        double[] orientations = new double[]{0.0, 0.25, 0.5, 0.75};

        Set<Point2D> points = new HashSet<>();

        // Add random point
        for (int i = 0; i < numberOfSample; i++) {
            points.add(new Point2D.Double(Math.random(), Math.random()));
        }

        points.addAll(getPointAroundObjects(state, robotWidth / 2));

        // Check each point if it is valid or not
        List<BotMateState> steps = new ArrayList<>();
        BotMateState newState;
        for (Point2D point : points) {

            for (double o : orientations) {
                newState = state.moveRobotToPosition(point, Math.PI * o);
                List<Box> movingObjects = new ArrayList<>();
                movingObjects.addAll(newState.getMovingBoxes());
                movingObjects.addAll(newState.getMovingObstacles());
                if (tester.hasCollision(newState.getRobotConfig(), movingObjects)) {
                    steps.add(newState);
                }
            }
        }
        System.out.println("Sample for robot: " + steps.size());
        return steps;
    }


    private static List<BotMateState> moveRobotToBox(BotMateState initialState, BotMateState goalState) {

        List<BotMateState> solution = new ArrayList<>();
        List<BotMateState> possibleStates = new ArrayList<>();

        possibleStates.add(initialState);
        possibleStates.addAll(PRMForRobot(initialState, ROBOT_RANDOM_SAMPLES));


        for (BotMateState state : possibleStates) {
//            System.out.println(state.outputString());
            for (BotMateState nextState : possibleStates) {
                // if the moving robot is not collide with other box
                if (!checkRobotCollide(state, nextState)) {
                    // Add the next state to the successor
                    double distance = state.getRobotConfig().getPos().distance(nextState.getRobotConfig().getPos());
                    if (distance > tester.MAX_ERROR) {
//                        System.out.println("\t" + nextState.outputString());
                        state.addSapmle(nextState);
                    }
                }

            }
        }


        System.out.println("Find solution robot to get to box: " + initialState.getMovingBoxIndex());
        solution.addAll(agent.search(initialState, goalState, true));
        return solution;
    }


    private static boolean checkRobotCollide(BotMateState state1, BotMateState state2) {

        List<Line2D> robotLines = new ArrayList<>();

        // Get robot config
        RobotConfig r1 = state1.getRobotConfig();
        RobotConfig r2 = state2.getRobotConfig();

        Line2D robotLine1 = new Line2D.Double(tester.getPoint1(r1), tester.getPoint2(r1));
        Line2D robotLine2 = new Line2D.Double(tester.getPoint1(r2), tester.getPoint2(r2));
        List<Point2D> robotBound1 = getVertices(tester.grow(robotLine1.getBounds2D(), robotWidth / 5));
        List<Point2D> robotBound2 = getVertices(tester.grow(robotLine2.getBounds2D(), robotWidth / 5));

        robotLines.add(new Line2D.Double(r1.getPos(), r2.getPos()));

        for (int i = 0; i < robotBound1.size(); i++) {
            robotLines.add(new Line2D.Double(robotBound1.get(i), robotBound2.get(i)));
        }

        // Check collision between every line and obstacle
        for (Line2D line : robotLines) {

            for (Box box : state2.getMovingBoxes()) {
                if (line.intersects(box.getRect())) {
                    return true;
                }
            }

            for (Box box : state2.getMovingObstacles()) {
                if (line.intersects(box.getRect())) {
                    return true;
                }
            }

            for (StaticObstacle obstacle : ps.getStaticObstacles()) {
                if (line.intersects(obstacle.getRect())) {
                    return true;
                }
            }

        }

        return false;
    }


    private static List<Point2D> getVertices(Rectangle2D rect) {
        List<Point2D> points = new ArrayList<>();
        Double w = rect.getWidth();
        Double h = rect.getHeight();
        double bottomLeftX = rect.getX();
        double bottomLeftY = rect.getY();
        points.add(new Point2D.Double(bottomLeftX, bottomLeftY));
        points.add(new Point2D.Double(bottomLeftX, bottomLeftY + h));
        points.add(new Point2D.Double(bottomLeftX + w, bottomLeftY + h));
        points.add(new Point2D.Double(bottomLeftX + w, bottomLeftY));
        return points;
    }


    public static List<Point2D> getPointsAroundRectangle(Rectangle2D rect) {
        //sample 8 points(4 vertices and 4 at the middle of vertices) around the object

        List<Point2D> pointList = new ArrayList<>();

        Point2D topLeft = new Point2D.Double();
        Point2D topRight = new Point2D.Double();
        Point2D bottomLeft = new Point2D.Double();
        Point2D BottomRight = new Point2D.Double();
        Point2D midUp = new Point2D.Double();
        Point2D midDown = new Point2D.Double();
        Point2D midLeft = new Point2D.Double();
        Point2D midRight = new Point2D.Double();

        topLeft.setLocation(rect.getMaxX(), rect.getMinY());
        topRight.setLocation(rect.getMaxX(), rect.getMaxY());
        bottomLeft.setLocation(rect.getMinX(), rect.getMinY());
        BottomRight.setLocation(rect.getMinX(), rect.getMaxY());
        midUp.setLocation((rect.getMaxX() + rect.getMinX()) / 2, rect.getMaxY());
        midDown.setLocation((rect.getMaxX() + rect.getMinX()) / 2, rect.getMinY());
        midLeft.setLocation(rect.getMinX(), (rect.getMinY() + rect.getMaxY()) / 2);
        midRight.setLocation(rect.getMaxX(), (rect.getMinY() + rect.getMaxY()) / 2);

        pointList.add(topLeft);
        pointList.add(topRight);
        pointList.add(bottomLeft);
        pointList.add(BottomRight);
        pointList.add(midUp);
        pointList.add(midDown);
        pointList.add(midLeft);
        pointList.add(midRight);

        return pointList;

    }

    public static List<Point2D> getPointAroundObjects(BotMateState state, double delta) {
        //this function creates samples around vertices of each object, and calculate the heuristics for each point.

        List<Point2D> points = new ArrayList<>();

        List<Rectangle2D> obstacleList = new ArrayList<>();
        for (StaticObstacle so : ps.getStaticObstacles()) {
            obstacleList.add(so.getRect());
        }
        for (Box b : state.getMovingBoxes()) {
            obstacleList.add(b.getRect());
        }
        for (Box b : state.getMovingObstacles()) {
            obstacleList.add(b.getRect());
        }

        //create samples around each obstacles

        for (Rectangle2D rect : obstacleList) {
            Rectangle2D grownRec = tester.grow(rect, delta);
            points.addAll(getPointsAroundRectangle(grownRec));
        }

        return points;
    }

    // Get the next edge that the robot need to get to
    public static int getDirection(BotMateState state1, BotMateState state2) {

        Box box1 = state1.getMovingBox();
        Box box2 = state2.getMovingBox();

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

    // Slide the robot to position, edgeOfState is the edge that the robot is stick to at that state
    public static List<BotMateState> slideRobot(BotMateState currentState, BotMateState nextState) {

        int currentEdge = tester.isCoupled(currentState.getRobotConfig(), currentState.getMovingBox());
        int nextEdge = tester.isCoupled(nextState.getRobotConfig(), nextState.getMovingBox());
        List<BotMateState> steps = new ArrayList<>();

        double s = robotWidth/2;
        double x = Math.PI/2;

        if (currentEdge == nextEdge) {
            return steps;
        }

        if (currentEdge == 1) {
            steps.add(currentState.moveRobot(0, 0, 0));
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
            steps.add(currentState.moveRobot(0, 0, 0));
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
            steps.add(currentState.moveRobot(0, 0, 0));
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
            steps.add(currentState.moveRobot(0, 0, 0));
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
        return steps;
    }

    private static List<String> generateMoves(BotMateState state1, BotMateState state2) {

        List<String> result = new LinkedList<>();
        result.add(state1.outputString());
        BotMateState tempState = state1;
        int direction = getDirection(state1, state2);

        Point2D robotPosition, boxPosition;

        // Get two robot config
        RobotConfig r1 = state1.getRobotConfig();
        RobotConfig r2 = state2.getRobotConfig();

        // Calculate the number of steps
        Double numberOfSteps = Math.ceil(r1.getPos().distance(r2.getPos()) / tester.MAX_BASE_STEP);

        // Calculate the delta values
        double deltaX = (r2.getPos().getX() - r1.getPos().getX()) / numberOfSteps;
        double deltaY = (r2.getPos().getY() - r1.getPos().getY()) / numberOfSteps;
        double deltaO = (tester.normaliseAngle(r2.getOrientation()) - tester.normaliseAngle(r1.getOrientation())) / numberOfSteps;

        // For each steps
        for (int i = 0; i < numberOfSteps; i++) {


            tempState = tempState.moveRobot(deltaX, deltaY, deltaO);

            Box box = tempState.getMovingBox();

            int coupled = tester.isCoupled(state1.getRobotConfig(), state1.getMovingBox());

//            if (coupled < 0 && tester.hasCollision(tempState.getRobotConfig(), tempState.getMovingBoxes())) {
//                tempState.moveRobotToMovingBox(direction);
//            }

            // only move the box if moving box of two state are the same
            if (state1.getMovingBoxIndex() == state2.getMovingBoxIndex()) {

                // if the box move horizontally
                if ((deltaX > 0 && coupled == 2) || (deltaX < 0 && coupled == 4)) {
                    boxPosition = new Point2D.Double(box.getPos().getX() + deltaX, box.getPos().getY());
                    tempState = tempState.moveMovingBox(boxPosition);
                }

                // if the box move vertically
                if ((deltaY > 0 && coupled == 1) || (deltaY < 0 && coupled == 3)) {
                    boxPosition = new Point2D.Double(box.getPos().getX(), box.getPos().getY() + deltaY);
                    tempState = tempState.moveMovingBox(boxPosition);
                }
            }

            result.add(tempState.outputString());
        }

        return result;
    }

}
