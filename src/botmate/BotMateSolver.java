package botmate;


import common.*;
import problem.*;
import tester.Tester;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

public class BotMateSolver {


    public static final int ROBOT_PRM_SAMPLES = 50;

    public static List<Line2D> boundaryLines = new ArrayList<>();


    static ProblemSpec ps;
    static Tester tester;

    static List<BotMateState> solutionStates = new LinkedList<>();

    /**
     * Main method - solve the problem
     *
     * @param args the list of argument
     */
    public static void main(String args[]) {

        double e = tester.MAX_BASE_STEP;
        boundaryLines.add(new Line2D.Double(-e, -e, -e, 1 + e));
        boundaryLines.add(new Line2D.Double(-e, 1 + e, 1 + e, 1 + e));
        boundaryLines.add(new Line2D.Double(1 + e, 1 + e, 1 + e, -e));
        boundaryLines.add(new Line2D.Double(1 + e, -e, -e, -e));

        try {
            ps = new ProblemSpec();
            ps.loadProblem("bot.input1.txt");
            tester = new Tester(ps);
        } catch (IOException ex) {
            System.out.println("IO Exception occurred");
        }


        BotMateState currentState = new BotMateState(ps.getInitialRobotConfig(), ps.getMovingBoxes(), ps.getMovingObstacles());
        solutionStates.add(currentState);

        // loop all the box
        for (int boxIndex = 0; boxIndex < ps.getMovingBoxes().size(); boxIndex++) {

            println("Moving Box:  " + boxIndex);
            Box currentBox = currentState.getMovingBoxes().get(boxIndex);

            Point2D currentGoal = ps.getMovingBoxEndPositions().get(boxIndex);

            // if currentBox not its goal then dot it
            if (currentBox.getPos().distance(currentGoal) > tester.MAX_ERROR) {

                println("\tNot in goal");
                // if the robot is not coupled with the robot, move to robot to the box
                if (tester.isCoupled(currentState.getRobotConfig(), ps.getMovingBoxes().get(boxIndex)) == -1) {
                    println("\tNot coupled");
                    currentState = moveRobotToBox(boxIndex, currentState);
                }
                println("\tMove to goal...");
                currentState = moveBoxToGoal(boxIndex, currentState, currentGoal);
            }
        }

        List<String> output = new LinkedList<>();
        currentState = solutionStates.get(0);
        println("Solution: ");
        for (BotMateState state : solutionStates) {

            println("\t" + state.outputString());

            for (String string : generateSteps(currentState, state)) {
                output.add(string);
            }
            currentState = state;
        }

        try {
            FileWriter fw = new FileWriter("bot.output.txt");
            BufferedWriter bw = new BufferedWriter(fw);

            println("Number of steps: " + output.size());
            bw.write(Integer.toString(output.size()));
            bw.write("\n");
            bw.flush();
            for (String string : output) {
                bw.write(string);
                bw.write("\n");

//            for (String string: generateSteps(currentState, state)) {
//                println(string);
//            }

            }
            bw.close();

        } catch (IOException ex) {
            System.out.println("IO Exception occurred");
        }


    }

    private static BotMateState moveRobotToBox(int boxIndex, BotMateState initialState) {

        Box box = initialState.getMovingBoxes().get(boxIndex);
        PriorityQueue<RobotConfig> targets = getRobotTargets(initialState.getRobotConfig(), box);

        RobotConfig target;
        while (!targets.isEmpty()) {
            target = targets.poll();

            BotMateState goalState = initialState.moveRobot(target.getPos(), target.getOrientation());

            List<BotMateState> possibleStates = new ArrayList<>();

            possibleStates.add(initialState);
            possibleStates.addAll(PRMForRobot(initialState, ROBOT_PRM_SAMPLES));
            possibleStates.add(goalState);

            println("\tNumber of sample: " + possibleStates.size());

            for (BotMateState state : possibleStates) {
                for (BotMateState nextState : possibleStates) {
                    // if the moving robot is not collide with other box
                    if (!checkRobotCollide(state, nextState)) {
                        // Add the next state to the successor
                        double distance = state.getRobotConfig().getPos().distance(nextState.getRobotConfig().getPos());
                        state.addSuccessor(new StateCostPair(nextState, 1 + distance));
                    }

                }
            }


            SearchAgent agent = new UCS();

            println("\tFind Solution...");
            List<StateCostPair> solution = agent.search(initialState, goalState);

            if (solution != null) {
                for (StateCostPair scp : solution) {
                    solutionStates.add((BotMateState) scp.state);
                }
                return goalState;
            }
        }

        return initialState;
    }

    private static BotMateState moveBoxToGoal(int boxIndex, BotMateState initialState, Point2D newPoint) {
        List<BotMateState> possibleStates = new ArrayList<>();

        BotMateState goalState = initialState.moveBoxToPosition(boxIndex, newPoint);
        possibleStates.add(initialState);
        possibleStates.addAll(PRMForBox(boxIndex, initialState, 100));
        possibleStates.add(goalState);


        println("\tNumber of sample: " + possibleStates.size());

        for (BotMateState state : possibleStates) {
            for (BotMateState nextState : possibleStates) {
                // if the moving robot is not collide with other box
                int check = checkMovingBoxCollide(boxIndex, state, nextState);
                if (check == 2) {
                    // Add the next state to the successor
                    double distance = state.getMovingBoxes().get(boxIndex).getPos().distance(nextState.getMovingBoxes().get(boxIndex).getPos());
                    state.addSuccessor(new StateCostPair(nextState, 1 + distance));
                }

            }
        }

        System.out.println("possible state");
        System.out.println(possibleStates.size());
        for (BotMateState state:possibleStates)
        {
            println(state.outputString());
        }

        SearchAgent agent = new UCS();

        println("\tFind Solution...");

        List<StateCostPair> solution = agent.search(initialState, goalState);

        if (solution != null) {
            for (StateCostPair scp : solution) {
                solutionStates.add((BotMateState) scp.state);
            }
            return goalState;
        }

        return initialState;
    }

    private static List<BotMateState> PRMForBox(int boxIndex, BotMateState state, int numberOfSample) {
        List<Point2D> points = new LinkedList<>();
        // Add random point
        for (int i = 0; i < numberOfSample; i++) {
            points.add(new Point2D.Double(Math.random(), Math.random()));
        }


        // Check each point if it is valid or not
        List<BotMateState> steps = new ArrayList<>();
        BotMateState step;
        for (Point2D point : points) {

            step = state.moveBoxToPosition(boxIndex, point);
            if (checkBoxPosition(boxIndex, step)) {
                steps.add(step);
            }
        }

        return steps;
    }

    private static List<BotMateState> PRMForRobot(BotMateState state, int numberOfSample) {
        double[] orientations = new double[]{0.0, 0.25, 0.5, 0.75};

        List<Point2D> points = new LinkedList<>();

        // Add random point
        for (int i = 0; i < numberOfSample; i++) {
            points.add(new Point2D.Double(Math.random(), Math.random()));
        }

        // Check each point if it is valid or not
        List<BotMateState> steps = new ArrayList<>();
        BotMateState step;
        for (Point2D point : points) {
            for (double o : orientations) {
                step = state.moveRobot(point, Math.PI * o);
                if (checkRobotPosition(step)) {
                    steps.add(step);
                    break;
                }
            }
        }


        return steps;
    }

    // Check if the moving box collide with other obstacle when moving from state 1 to state 2
    private static int checkMovingBoxCollide(int boxIndex, BotMateState state1, BotMateState state2) {
        Line2D line;

        // For any moving box,
        Box box1 = state1.getMovingBoxes().get(boxIndex);
        Box box2 = state2.getMovingBoxes().get(boxIndex);

        // if the box is not moved
        if (box1.getPos().equals(box2.getPos())) {
            return 0;
        } else {
            // Draw a line between two center point
            line = new Line2D.Double(box1.getPos(), box2.getPos());

            // Get the boundary and add padding (w/2)
            Rectangle2D boundary = tester.grow(line.getBounds2D(), (ps.getRobotWidth() / 2));
            println(boundary);

            // Check intersect with other moving box
            for (int j = 0; j < state1.getMovingBoxes().size(); j++) {
                if (boxIndex != j) {
                    Box box = state2.getMovingBoxes().get(j);
                    if (boundary.intersects(box.getRect())) {
                        return 1;
                    }
                }
            }

            // Check intersect with other moving obstacle box
            for (Box box : state2.getMovingObstacles()) {
                if (boundary.intersects(box.getRect())) {
                    return 1;
                }
            }

            // Check intersect with static obstacles
            for (StaticObstacle obstacle : ps.getStaticObstacles()) {
                if (boundary.intersects(obstacle.getRect())) {
                    return 1;
                }
            }

            return 2;
        }
    }


    private static boolean checkRobotCollide(BotMateState state1, BotMateState state2) {

        List<Line2D> robotLines = new ArrayList<>();

        // Get robot config
        RobotConfig r1 = state1.getRobotConfig();
        RobotConfig r2 = state2.getRobotConfig();

        Line2D robotLine1 = new Line2D.Double(tester.getPoint1(r1), tester.getPoint2(r1));
        Line2D robotLine2 = new Line2D.Double(tester.getPoint1(r2), tester.getPoint2(r2));
        List<Point2D> robotBound1 = getVertices(robotLine1.getBounds2D());
        List<Point2D> robotBound2 = getVertices(robotLine2.getBounds2D());

        robotLines.add(new Line2D.Double(r1.getPos(), r2.getPos()));

        for (int i = 0; i < robotBound1.size(); i++) {
            robotLines.add(new Line2D.Double(robotBound1.get(i), robotBound2.get(i)));
        }

        // Check collision between every line and obstacle
        for (Line2D line : robotLines) {

            // Check collision with boundary
            if (isCollideWithBoundary(line)) {
                return true;
            }


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


    private static boolean checkRobotPosition(BotMateState state) {

        // Get robot config
        RobotConfig robot = state.getRobotConfig();

        // Draw lines between points the current and next robot,
        Line2D robotLine = new Line2D.Double(tester.getPoint1(robot), tester.getPoint2(robot));

        // Check collision with boundary
        if (isCollideWithBoundary(robotLine)) {
            return false;
        }

        // Check collision between every line and obstacle
        for (Box box : state.getMovingBoxes()) {
            if (robotLine.intersects(box.getRect())) {
                return false;
            }
        }

        for (Box box : state.getMovingObstacles()) {
            if (robotLine.intersects(box.getRect())) {
                return false;
            }
        }

        for (StaticObstacle obstacle : ps.getStaticObstacles()) {
            if (robotLine.intersects(obstacle.getRect())) {
                return false;
            }
        }

        return true;
    }

    /*Moving Box Method*/

    private static boolean checkBoxPosition(int boxIndex, BotMateState state) {

        // Get the boundary and add padding (w/2)
         Rectangle2D boundary = tester.grow(state.getMovingBoxes().get(boxIndex).getRect(), state.getMovingBoxes().get(boxIndex).getWidth() / 2);
        //Rectangle2D boundary = state.getMovingBoxes().get(boxIndex).getRect();


        for (Line2D boundaryLine : boundaryLines) {
            if (boundaryLine.intersects(boundary)) {
                return false;
            }
        }

        for (Box box : state.getMovingObstacles()) {
            if (boundary.intersects(box.getRect())) {
                return false;
            }
        }

        int indexNextBox = 1 + boxIndex;
        while (indexNextBox < state.getMovingBoxes().size()) {
            if (boundary.intersects(state.getMovingBoxes().get(indexNextBox).getRect())) {
                return false;
            }
            indexNextBox++;
        }

        for (StaticObstacle obstacle : ps.getStaticObstacles()) {
            if (boundary.intersects(obstacle.getRect())) {
                return false;
            }
        }

        return true;
    }


    private static List<String> generateSteps(BotMateState state1, BotMateState state2) {

        List<String> result = new LinkedList<>();
        result.add(state1.outputString());
        BotMateState movingState = new BotMateState(state1);
        RobotConfig r1 = state1.getRobotConfig();
        RobotConfig r2 = state2.getRobotConfig();
        Double numberOfSteps = Math.ceil(r1.getPos().distance(r2.getPos()) / 0.001);

        double deltaX = (r2.getPos().getX() - r1.getPos().getX()) / numberOfSteps;
        double deltaY = (r2.getPos().getY() - r1.getPos().getY()) / numberOfSteps;
        double deltaO = (r2.getOrientation() - r1.getOrientation()) / numberOfSteps;

        for (int i = 0; i < numberOfSteps; i++) {
            RobotConfig r3 = movingState.getRobotConfig();
            Point2D newPosition = new Point2D.Double(r3.getPos().getX() + deltaX, r3.getPos().getY() + deltaY);
            movingState = movingState.moveRobot(newPosition, r3.getOrientation() + deltaO);
            result.add(movingState.outputString());
        }

        return result;
    }

    private static void print(Object o) {
        System.out.print(o);
    }

    private static void println(Object o) {
        System.out.print(o);
        System.out.print("\n");
    }

    public static double calculateDistance(Point2D initial, Point2D goal) {
        return (Math.abs(initial.getX() - goal.getX()) + Math.abs(initial.getY() - goal.getY()));
    }

    private static boolean isCollideWithBoundary(Line2D line) {
        for (Line2D boundaryLine : boundaryLines) {
            if (line.intersectsLine(boundaryLine)) {
                return true;
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

    private static PriorityQueue<RobotConfig> getRobotTargets(RobotConfig robot, Box box) {

        PriorityQueue<RobotConfig> targets = new PriorityQueue<>(new Comparator<RobotConfig>() {
            @Override
            public int compare(RobotConfig o1, RobotConfig o2) {
                if (o1.getPos().distance(robot.getPos()) - o2.getPos().distance(robot.getPos()) > 0) {
                    return 1;
                } else if (o1.getPos().distance(robot.getPos()) - o2.getPos().distance(robot.getPos()) < 0) {
                    return -1;
                } else {
                    return 0;
                }
            }
        });


        Double w = box.getWidth();
        double bottomLeftX = box.getPos().getX();
        double bottomLeftY = box.getPos().getY();

        targets.add(new RobotConfig(new Point2D.Double(bottomLeftX + w / 2, bottomLeftY - tester.MAX_ERROR), 0.0));
        targets.add(new RobotConfig(new Point2D.Double(bottomLeftX - tester.MAX_ERROR, bottomLeftY + w / 2), Math.PI * 0.5));
        targets.add(new RobotConfig(new Point2D.Double(bottomLeftX + w / 2, bottomLeftY + w + tester.MAX_ERROR), 0.0));
        targets.add(new RobotConfig(new Point2D.Double(bottomLeftX + w + tester.MAX_ERROR, bottomLeftY + w / 2), Math.PI * 0.5));
        return targets;
    }

    // Find middle points of any two objects (including Moving Boxes and Static Obstacle)
    private static List<Point2D> getMiddlePointBetweenObjects(BotMateState state) {
        return null;
    }

}
