package botmate;


import com.sun.javafx.scene.paint.GradientUtils;
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


    public static final int ROBOT_RANDOM_SAMPLES = 50;

    public static final int BOX_RANDOM_SAMPLES = 200;


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
            output.add(state.outputString());
//            for (String string : generateSteps(currentState, state)) {
//                output.add(string);
//            }
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
            possibleStates.addAll(PRMForRobot(initialState, ROBOT_RANDOM_SAMPLES));
            possibleStates.add(goalState);

            println("\tNumber of sample: " + possibleStates.size());

            for (BotMateState state : possibleStates) {
                for (BotMateState nextState : possibleStates) {
                    // if the moving robot is not collide with other box
                    if (!checkRobotCollide(state, nextState)) {
                        // Add the next state to the successor
                        double distance = state.getRobotConfig().getPos().distance(nextState.getRobotConfig().getPos());

                        if (distance > tester.MAX_ERROR) {
                            state.addSuccessor(new StateCostPair(nextState, 1 + distance));
                        }
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
            } else {
                println("\t No solution");
            }
        }

        return initialState;
    }

    private static BotMateState moveBoxToGoal(int boxIndex, BotMateState initialState, Point2D newPoint) {
        List<BotMateState> possibleStates = new ArrayList<>();

        BotMateState goalState = moveBoxToPosition(boxIndex, initialState, newPoint);
        possibleStates.add(initialState);
        possibleStates.addAll(PRMForBox(boxIndex, initialState, BOX_RANDOM_SAMPLES));
        possibleStates.add(goalState);

        println("\tNumber of sample: " + possibleStates.size());

        for (BotMateState state : possibleStates) {
            for (BotMateState nextState : possibleStates) {
                if (!checkMovingBoxCollide(boxIndex, state, nextState)) {
                    Box nextBox = nextState.getMovingBoxes().get(boxIndex);

                    // Add the next state to the successor
                    double distance = state.getMovingBoxes().get(boxIndex).getPos().distance(nextBox.getPos());
                    if (distance > tester.MAX_ERROR) {
                        state.addSuccessor(new StateCostPair(nextState, 1 + distance));
                    }
                }

            }
        }


        SearchAgent agent = new BFS();

        println("\tFind Solution...");
        List<StateCostPair> solution = agent.search(initialState, goalState);
        BotMateState previous = initialState;
        if (solution != null) {

            for (StateCostPair scp : solution) {

                solutionStates.add(generateMiddleBoxState(boxIndex,previous, (BotMateState)  scp.state));
                solutionStates.add((BotMateState)scp.state);
                previous = (BotMateState)scp.state;
            }
            return goalState;
        } else {
            println("\t No solution");
        }

        return initialState;
    }

    private static BotMateState generateMiddleBoxState(int boxIndex, BotMateState state1, BotMateState state2) {
        // For any moving box,
        Box box1 = state1.getMovingBoxes().get(boxIndex);
        Box box2 = state2.getMovingBoxes().get(boxIndex);


        double x1, y1, x2, y2;

        x1 = box1.getPos().getX();
        y1 = box1.getPos().getY();
        x2 = box2.getPos().getX();
        y2 = box2.getPos().getY();

        Point2D point1 = new Point2D.Double(x1,y2);
        Point2D point2 = new Point2D.Double(x2,y1);

        if (!checkMovingBoxPathCollide(boxIndex, state1, state2, point1)) {
            return moveBoxToPosition(boxIndex, state1, point1);
        }
        if (!checkMovingBoxPathCollide(boxIndex, state1, state2, point2)) {
            return moveBoxToPosition(boxIndex, state1, point2);
        }
        return state2;

    }

    private static List<BotMateState> PRMForBox(int boxIndex, BotMateState state, int numberOfSample) {
        List<Point2D> points = new LinkedList<>();
//         Add random point

        for (int i = 0; i < numberOfSample; i++) {
            points.add(new Point2D.Double(Math.random(), Math.random()));
        }

        points.addAll(getPointAroundObjects(state, state.getMovingBoxes().get(boxIndex).getWidth() + tester.MAX_BASE_STEP));

        // Get point around goal:

        double width = state.getMovingBoxes().get(boxIndex).getWidth();
        Point2D goal = ps.getMovingBoxEndPositions().get(boxIndex);
        points.add(new Point2D.Double(goal.getX() + width, goal.getY() + width));
        points.add(new Point2D.Double(goal.getX() + width, goal.getY() - width));
        points.add(new Point2D.Double(goal.getX() - width, goal.getY() + width));
        points.add(new Point2D.Double(goal.getX() - width, goal.getY() - width));

//        points.add(new Point2D.Double(goal.getX() + 0.5 * width, goal.getY() + 0.5 * width));
//        points.add(new Point2D.Double(goal.getX() + 0.5 * width, goal.getY() - 0.5 * width));
//        points.add(new Point2D.Double(goal.getX() - 0.5 * width, goal.getY() + 0.5 * width));
//        points.add(new Point2D.Double(goal.getX() - 0.5 * width, goal.getY() - 0.5 * width));

        points.add(new Point2D.Double(goal.getX() + 1.5 * width, goal.getY() + 1.5 * width));
        points.add(new Point2D.Double(goal.getX() + 1.5 * width, goal.getY() - 1.5 * width));
        points.add(new Point2D.Double(goal.getX() - 1.5 * width, goal.getY() + 1.5 * width));
        points.add(new Point2D.Double(goal.getX() - 1.5 * width, goal.getY() - 1.5 * width));

        // Check each point if it is valid or not
        List<BotMateState> steps = new ArrayList<>();
        BotMateState step;
        for (Point2D point : points) {

            step = moveBoxToPosition(boxIndex, state, point);
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

        points.addAll(getPointAroundObjects(state, tester.MAX_BASE_STEP));


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
    private static boolean checkMovingBoxCollide(int boxIndex, BotMateState state1, BotMateState state2) {

        // For any moving box,
        Box box1 = state1.getMovingBoxes().get(boxIndex);
        Box box2 = state2.getMovingBoxes().get(boxIndex);


        double x1, y1, x2, y2;

        x1 = box1.getPos().getX() + box1.getWidth()/2;
        y1 = box1.getPos().getY() + box1.getWidth()/2;
        x2 = box2.getPos().getX() + box1.getWidth()/2;
        y2 = box2.getPos().getY() + box1.getWidth()/2;

        Point2D point1 = new Point2D.Double(x1,y2);
        Point2D point2 = new Point2D.Double(x2,y1);

        return checkMovingBoxPathCollide(boxIndex, state1, state2, point1) ||
                checkMovingBoxPathCollide(boxIndex, state1, state2, point2);
    }

    private static boolean checkMovingBoxPathCollide(int boxIndex, BotMateState state1, BotMateState state2, Point2D point) {

        // For any moving box,
        Box box1 = state1.getMovingBoxes().get(boxIndex);
        Box box2 = state2.getMovingBoxes().get(boxIndex);

        Point2D center1 = new Point2D.Double(box1.getPos().getX() + box1.getWidth()/2, box1.getPos().getY() + box1.getWidth()/2);
        Point2D center2 = new Point2D.Double(box2.getPos().getX() + box2.getWidth()/2, box2.getPos().getY() + box2.getWidth()/2);

        Line2D line11 = new Line2D.Double(center1, point);
        Line2D line12 = new Line2D.Double(point, center2);

        Rectangle2D boundary1 = tester.grow(line11.getBounds2D(), ps.getRobotWidth()/2 + tester.MAX_BASE_STEP);
        Rectangle2D boundary2 = tester.grow(line12.getBounds2D(), ps.getRobotWidth()/2 + tester.MAX_BASE_STEP);

        if (isCollideWithBoundary(boundary1) || isCollideWithBoundary(boundary2)) {
            return true;
        }

        // Check intersect with other moving box
        for (int i = 0; i<state1.getMovingBoxes().size(); i++) {
            if (i != boxIndex) {
                Box box = state1.getMovingBoxes().get(i);
                if (boundary1.intersects(box.getRect()) || boundary2.intersects(box.getRect())) {
                    return true;
                }
            }
        }

        // Check intersect with other moving obstacle box
        for (Box box : state2.getMovingObstacles()) {
            if (boundary1.intersects(box.getRect()) || boundary2.intersects(box.getRect())) {
                return true;
            }
        }

        // Check intersect with static obstacles
        for (StaticObstacle obstacle : ps.getStaticObstacles()) {
            if (boundary1.intersects(obstacle.getRect()) || boundary2.intersects(obstacle.getRect())) {
                return true;
            } else {

            }
        }

        return false;
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


        if (isCollideWithBoundary(boundary)) {
            return false;
        }

        for (int j = 0; j < state.getMovingBoxes().size(); j++) {
            if (j != boxIndex) {
                if (boundary.intersects(state.getMovingBoxes().get(j).getRect())) {
                    return false;
                }
            }
        }

        for (Box box : state.getMovingObstacles()) {
            if (boundary.intersects(box.getRect())) {
                return false;
            }

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

    private static boolean isCollideWithBoundary(Rectangle2D rect) {
        for (Line2D boundaryLine : boundaryLines) {
            if (rect.intersectsLine(boundaryLine)) {
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


    public static List<Point2D> getPointAroundObjects(BotMateState state, double delta){
        //this function creates samples around vertices of each object, and calculate the heuristics for each point.

        List<Point2D> points = new ArrayList<>();

        List<Rectangle2D> obstacleList =new ArrayList<>();
        for (StaticObstacle so: ps.getStaticObstacles()){
            obstacleList.add(so.getRect());
        }
        for(Box b:state.getMovingBoxes()){
            obstacleList.add(b.getRect());
        }
        for (Box b:state.getMovingObstacles()){
            obstacleList.add(b.getRect());
        }

        //create samples around each obstacles

        for (Rectangle2D rect: obstacleList){
            Rectangle2D grownRec = tester.grow(rect, delta);

            points.addAll(getPointsAroundRectangle(grownRec));
        }

        return points;
    }

    public static List<Point2D> getPointsAroundRectangle(Rectangle2D rect) {
        //sample 8 points(4 vertices and 4 at the middle of vertices) around the object

        List<Point2D> pointList = new ArrayList<>();

        Point2D topLeft= new Point2D.Double();
        Point2D topRight=new Point2D.Double();
        Point2D bottomLeft=new Point2D.Double();
        Point2D BottomRight=new Point2D.Double();
        Point2D midUp=new Point2D.Double();
        Point2D midDown=new Point2D.Double();
        Point2D midLeft=new Point2D.Double();
        Point2D midRight=new Point2D.Double();

        //get 9 points acounrd object
        topLeft.setLocation(rect.getMaxX(),rect.getMinY());
        topRight.setLocation(rect.getMaxX(),rect.getMaxY());
        bottomLeft.setLocation(rect.getMinX(),rect.getMinY());
        BottomRight.setLocation(rect.getMinX(),rect.getMaxY());
        midUp.setLocation((rect.getMaxX()+rect.getMinX())/2,rect.getMaxY());
        midDown.setLocation((rect.getMaxX()+rect.getMinX())/2,rect.getMinY());
        midLeft.setLocation(rect.getMinX(),(rect.getMinY()+rect.getMaxY())/2);
        midRight.setLocation(rect.getMaxX(),(rect.getMinY()+rect.getMaxY())/2);

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


    public static BotMateState moveBoxToPosition(int boxIndex, BotMateState state, Point2D position) {
        MovingBox newMovingbox = new MovingBox(position, state.getMovingBoxes().get(boxIndex).getWidth());

        List<Box> newMovingBoxList = new ArrayList<>();

        //todo: Deep Copy of moving boxes;
        for (int counterBox = 0; counterBox <  state.getMovingBoxes().size(); counterBox++) {
            if (counterBox == boxIndex) {
                newMovingBoxList.add(newMovingbox);
            } else {
                newMovingBoxList.add( state.getMovingBoxes().get(counterBox));
            }
        }
        return new BotMateState(state.getRobotConfig(), newMovingBoxList, state.getMovingObstacles());
    }
}
