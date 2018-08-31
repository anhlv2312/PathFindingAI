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

    private static final int ROBOT_RANDOM_SAMPLES = 200;

    static ProblemSpec ps;
    static Tester tester;

    public static void main(String args[]) {

        try {
            ps = new ProblemSpec();
            ps.loadProblem("input3.txt");
            tester = new Tester(ps);
        } catch (IOException ex) {
            System.out.println("IO Exception occurred");
        }

        BotMateState initialState, currentState, goalState;
        Point2D movingBoxGoal;
        List<StateCostPair> solution = new LinkedList<>();
        SearchAgent agent = new Astar();

        initialState = new BotMateState(0,
                ps.getInitialRobotConfig(),
                ps.getMovingBoxes(),
                ps.getMovingObstacles(),
                tester);

        currentState = initialState;

        // loop all the box
        for (int i=0; i < ps.getMovingBoxes().size(); i++) {

            currentState.setMovingBoxIndex(i);
            solution.add(new StateCostPair(currentState,0));



            if (tester.isCoupled(currentState.getRobotConfig(), ps.getMovingBoxes().get(i)) < 0) {
                goalState = currentState.moveRobotToMovingBox(1);;
                solution.addAll(moveRobotToBox(currentState, goalState));
                currentState = goalState;

                goalState = currentState.moveRobotToMovingBox(2);;
                solution.addAll(moveRobotToBox(currentState, goalState));
                currentState = goalState;
            }

            movingBoxGoal = ps.getMovingBoxEndPositions().get(i);
            goalState = currentState.moveMovingBox(movingBoxGoal);
            solution.addAll(agent.search(currentState, goalState));
            currentState = goalState;
        }


        List<String> output = new LinkedList<>();

        currentState = initialState;
        BotMateState state;
        for (StateCostPair stateCostPair: solution) {

            state = (BotMateState)stateCostPair.state;
            System.out.println(state.outputString());
//            output.add(state.outputString());
            output.addAll(generateMoves(currentState, state));
            currentState = state;

        }

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


    private static List<String> generateMoves(BotMateState state1, BotMateState state2) {


//        List<String> result = new LinkedList<>();
//
//        if (state1.getMovingBoxIndex () != state2.getMovingBoxIndex ()) {
//            return result;
//        }
//
//        BotMateState temp = state1;
//
//        Box movingBox1 = state1.getMovingBox();
//        Box movingBox2 = state2.getMovingBox();
//
//        Double numberOfSteps = Math.ceil(movingBox1.getPos().distance(movingBox2.getPos()) / 0.001);
//
//        double deltaX = (movingBox2.getPos().getX() - movingBox1.getPos().getX()) / numberOfSteps;
//        double deltaY = (movingBox2.getPos().getY() - movingBox1.getPos().getY()) / numberOfSteps;
//        for (int i = 0; i < numberOfSteps; i++) {
//            Point2D position = new Point2D.Double(temp.getMovingBox().getPos().getX() + deltaX, temp.getMovingBox().getPos().getY() + deltaY);
//            temp = temp.moveMovingBox(position);
//            result.add(temp.outputString());
//        }

        List<String> result = new LinkedList<>();
        result.add(state1.outputString());
        BotMateState temp = state1;
        RobotConfig r1 = state1.getRobotConfig();
        RobotConfig r2 = state2.getRobotConfig();
        Double numberOfSteps = Math.ceil(r1.getPos().distance(r2.getPos()) / 0.001);

        double deltaX = (r2.getPos().getX() - r1.getPos().getX()) / numberOfSteps;
        double deltaY = (r2.getPos().getY() - r1.getPos().getY()) / numberOfSteps;
        double deltaO = (r2.getOrientation() - r1.getOrientation()) / numberOfSteps;

        for (int i = 0; i < numberOfSteps; i++) {
            RobotConfig r3 = temp.getRobotConfig();
            Point2D newPosition = new Point2D.Double(r3.getPos().getX() + deltaX, r3.getPos().getY() + deltaY);
            temp = temp.moveRobot(newPosition, r3.getOrientation() + deltaO);
            result.add(temp.outputString());
        }

        return result;

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
        BotMateState newState;
        for (Point2D point : points) {

            for (double o : orientations) {
                newState = state.moveRobot(point, Math.PI * o);
                List<Box> movingObjects = new ArrayList<>();
                movingObjects.addAll(newState.getMovingBoxes());
                movingObjects.addAll(newState.getMovingObstacles());
                if (tester.hasCollision(newState.getRobotConfig(), movingObjects)) {
                    steps.add(newState);
                }
            }
        }

        return steps;
    }



    private static List<StateCostPair> moveRobotToBox(BotMateState initialState, BotMateState goalState) {


        List<StateCostPair> solution;
        List<BotMateState> possibleStates = new ArrayList<>();

        possibleStates.add(initialState);
        possibleStates.addAll(PRMForRobot(initialState, ROBOT_RANDOM_SAMPLES));
        possibleStates.add(goalState);


        for (BotMateState state : possibleStates) {
            for (BotMateState nextState : possibleStates) {
                // if the moving robot is not collide with other box
                if (!checkRobotCollide(state, nextState)) {
                    // Add the next state to the successor
                    double distance = state.getRobotConfig().getPos().distance(nextState.getRobotConfig().getPos());
                    if (distance > tester.MAX_ERROR && distance <  0.2) {
                        state.addSuccessor(new StateCostPair(nextState, distance));
                    }
                }

            }
        }

        SearchAgent agent = new BFS();
        solution = agent.search(initialState, goalState);
        if (solution == null) {
            System.out.println("No Solution for robot");
        }
        return solution;
    }



    private static boolean checkRobotCollide(BotMateState state1, BotMateState state2) {



        List<Line2D> robotLines = new ArrayList<>();

        // Get robot config
        RobotConfig r1 = state1.getRobotConfig();
        RobotConfig r2 = state2.getRobotConfig();

//        if (r1.getPos() != r2.getPos() && r1.getOrientation() != r2.getOrientation()) {
//            return true;
//        }

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

            for (Box box: state2.getMovingBoxes()) {
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

        Point2D topLeft= new Point2D.Double();
        Point2D topRight=new Point2D.Double();
        Point2D bottomLeft=new Point2D.Double();
        Point2D BottomRight=new Point2D.Double();
        Point2D midUp=new Point2D.Double();
        Point2D midDown=new Point2D.Double();
        Point2D midLeft=new Point2D.Double();
        Point2D midRight=new Point2D.Double();

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
}
