package botmate;

import problem.*;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.*;

public class RobotAgent extends SearchAgent {

    Box targetBox;
    int targetEdge;

    public RobotAgent(ProblemSpec ps, State initialState, Box targetBox, int targetEdge) {
        super(ps, initialState);
        this.targetBox = targetBox;
        this.targetEdge = targetEdge;
    }

    @Override
    public boolean isFound(State currentState) {
        return (tester.isCoupled(currentState.robotConfig, targetBox) == targetEdge);
    }

    public double calculateCost(RobotConfig currentConfig, RobotConfig nextConfig) {
        double distance = currentConfig.getPos().distance(nextConfig.getPos());
        double rotation = currentConfig.getOrientation() - nextConfig.getOrientation() * robotWidth / 2;
        return distance + rotation;
    }

    public double calculateHeuristic(RobotConfig currentConfig) {
        double distance = currentConfig.getPos().distance(targetBox.getPos());
        return distance;
    }

    @Override
    public List<SearchNode> getSuccessors(State currentState) {

        double[] orientations = new double[]{0, Math.PI * 0.5, };

        List<Point2D> positions = new ArrayList<>();
        positions.addAll(getPointAroundObstacles(currentState, robotWidth/2));
        positions.addAll(getPointAroundObstacles(currentState, tester.MAX_ERROR));
        positions.addAll(getPointsAroundRectangle(targetBox.getRect(), tester.MAX_ERROR));
        positions.add(currentState.robotConfig.getPos());

        List<State> states = new ArrayList<>();
        State tempState;

        for (Point2D position: positions) {
            for (double orientation: orientations) {
                tempState = currentState.moveRobotToPosition(position, orientation);
                if (checkRobotMovingCollision(currentState, tempState.robotConfig)) {
                    states.add(tempState);
                }
            }
        }


//        System.out.println(currentState.toString());
        List<SearchNode> nodes = new ArrayList<>();
        for (State state: states) {
            double cost = calculateCost(currentState.robotConfig, state.robotConfig);
            double heuristic = calculateHeuristic(state.robotConfig);
            nodes.add(new SearchNode(state, cost, heuristic));
//            System.out.println("\t" + state.toString());
        }

        return nodes;
    }


    public boolean checkRobotMovingCollision(State state, RobotConfig nextConfig) {

        List<Line2D> movingLines = new ArrayList<>();
        // Get robot config

        Point2D r1p1 = tester.getPoint1(state.robotConfig);
        Point2D r1p2 = tester.getPoint2(state.robotConfig);
        Point2D r2p1 = tester.getPoint1(nextConfig);
        Point2D r2p2 = tester.getPoint2(nextConfig);

        movingLines.add(new Line2D.Double(r1p1, r1p2));
        movingLines.add(new Line2D.Double(r2p1, r2p2));
        movingLines.add(new Line2D.Double(r1p1, r2p1));
        movingLines.add(new Line2D.Double(r1p1, r2p2));
        movingLines.add(new Line2D.Double(r1p2, r2p1));
        movingLines.add(new Line2D.Double(r1p2, r2p2));

        Rectangle2D border = new Rectangle2D.Double(0,0,1,1);

        if (!border.contains(r1p1) || !border.contains(r1p2) || !border.contains(r2p1) || !border.contains(r2p2)) {
            return false;
        }

        for (Line2D line: movingLines) {
            for (Box box: state.movingBoxes) {
                if (tester.isCoupled(state.robotConfig, box) > 0) {
                    continue;
                }
                if (line.intersects(box.getRect())) {
                    return false;
                }
            }

            for (Box box: state.movingObstacles) {
                if (tester.isCoupled(state.robotConfig, box) > 0) {
                    continue;
                }
                if (line.intersects(box.getRect())) {
                    return false;
                }
            }

            for (StaticObstacle obstacle: staticObstacles) {
                if (line.intersects(obstacle.getRect())) {
                    return false;
                }
            }
        }

        if ((state.robotConfig.getOrientation() - nextConfig.getOrientation()) != 0) {
            Rectangle2D robotRect;
            double bottomLeftX = state.robotConfig.getPos().getX()-robotWidth/2;
            double bottomLeftY = state.robotConfig.getPos().getY()-robotWidth/2;
            robotRect = new Rectangle2D.Double(bottomLeftX, bottomLeftY, robotWidth, robotWidth);

            for (Box box: state.movingObstacles) {
                if (robotRect.intersects(box.getRect())) {
                    return false;
                }
            }
        }

        return true;
    }

    public List<RobotConfig> getPossibleRobotConfig(Box movingBox) {

        List<RobotConfig> robotConfigs = new ArrayList<>();
        Double w = movingBox.getWidth();
        double bottomLeftX = movingBox.getPos().getX();
        double bottomLeftY = movingBox.getPos().getY();

        robotConfigs.add(new RobotConfig(new Point2D.Double(bottomLeftX + w/2, bottomLeftY), 0));
        robotConfigs.add(new RobotConfig(new Point2D.Double(bottomLeftX, bottomLeftY + w/2), Math.PI/2));
        robotConfigs.add(new RobotConfig(new Point2D.Double(bottomLeftX + w/2, bottomLeftY + w), 0));
        robotConfigs.add(new RobotConfig(new Point2D.Double(bottomLeftX + w, bottomLeftY + w/2), Math.PI/2));

        return robotConfigs;
    }

    public List<Point2D> getPossibleRobotPoint(Box movingBox) {

        List<Point2D> robotPoints = new ArrayList<>();
        Double w = movingBox.getWidth();
        double bottomLeftX = movingBox.getPos().getX();
        double bottomLeftY = movingBox.getPos().getY();

        robotPoints.add(new Point2D.Double(bottomLeftX + w/2, bottomLeftY));
        robotPoints.add(new Point2D.Double(bottomLeftX, bottomLeftY + w/2));
        robotPoints.add(new Point2D.Double(bottomLeftX + w/2, bottomLeftY + w));
        robotPoints.add(new Point2D.Double(bottomLeftX + w, bottomLeftY + w/2));

        return robotPoints;
    }


    public List<Point2D> getPointsAroundRectangle(Rectangle2D rectangle, double delta) {
        //sample 8 points(4 vertices and 4 at the middle of vertices) around the object

        List<Point2D> pointList = new ArrayList<>();
        Rectangle2D rect = tester.grow(rectangle, delta);

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

    public List<Point2D> getPointAroundObstacles(State currentState, double delta) {
        //this function creates samples around vertices of each object, and calculate the heuristics for each point.

        List<Point2D> points = new ArrayList<>();

        List<Rectangle2D> obstacleList = new ArrayList<>();
        for (StaticObstacle so : staticObstacles) {
            obstacleList.add(so.getRect());
        }

        for (Box b : currentState.movingObstacles) {
            obstacleList.add(b.getRect());
        }

        //create samples around each obstacles

        for (Rectangle2D rect : obstacleList) {
            points.addAll(getPointsAroundRectangle(rect, delta));
        }

        return points;
    }

    
}
