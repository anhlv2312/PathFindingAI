package botmate;

import problem.*;
import tester.Tester;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.*;

public class RobotAgent extends SearchAgent {

    private RobotConfig targetConfig;
    private Box movingBox;

    RobotAgent(ProblemSpec ps, State initialState, RobotConfig targetConfig, Box movingBox) {
        super(ps, initialState);
        this.targetConfig = targetConfig;
        this.movingBox = movingBox;
    }

    @Override
    public boolean isFound(State currentState) {
        return currentState.robotConfig.getPos().distance(targetConfig.getPos()) < Tester.MAX_ERROR;
    }

    @Override
    public List<SearchNode> getSuccessors(State currentState) {

        double[] orientations = new double[]{0, Math.PI * 0.5};
        Set<Point2D> positions = new HashSet<>();

        positions.add(targetConfig.getPos());
        positions.addAll(getPointsAroundRectangle(movingBox.getRect(), Tester.MAX_ERROR));
        positions.addAll(getPointsAroundRectangle(movingBox.getRect(), robotWidth/2));
        positions.addAll(getPointsAroundObstacles(getObstacles(currentState), robotWidth/2));

        List<State> possibleStates = new ArrayList<>();
        State tempState;

        for (Point2D position: positions) {
            for (double orientation: orientations) {
                tempState = currentState.moveRobotToPosition(position, orientation);
                if (checkRobotMovingCollision(currentState, tempState.robotConfig)) {
                    possibleStates.add(tempState);
                }
            }
        }
        List<SearchNode> nodes = new ArrayList<>();
        for (State nextState: possibleStates) {
            nodes.add(new SearchNode(nextState));
        }

        return nodes;
    }


    private boolean checkRobotMovingCollision(State state, RobotConfig nextConfig) {

        Rectangle2D border = new Rectangle2D.Double(0,0,1,1);

        double angle = state.robotConfig.getOrientation() - nextConfig.getOrientation();

        RobotConfig tempRobotConfig;
        if (angle == 0) {
            tempRobotConfig = new RobotConfig(state.robotConfig.getPos(), state.robotConfig.getOrientation());
        } else {

            Rectangle2D robotRect;
            double bottomLeftX = state.robotConfig.getPos().getX()-robotWidth/2;
            double bottomLeftY = state.robotConfig.getPos().getY()-robotWidth/2;

            robotRect = new Rectangle2D.Double(bottomLeftX, bottomLeftY, robotWidth, robotWidth);
            Set<Point2D> robotPoints = getPointsAroundRectangle(robotRect, 0);

            for (Point2D point: robotPoints) {
                if (!border.contains(point) ) {
                    return false;
                }
            }

            for (Box box: state.movingBoxes) {
                if (robotRect.intersects(box.getRect())) {
                    return false;
                }
            }

            for (Box box: state.movingObstacles) {
                if (robotRect.intersects(box.getRect())) {
                    return false;
                }
            }

            for (StaticObstacle box: staticObstacles) {
                if (robotRect.intersects(box.getRect())) {
                    return false;
                }
            }

            tempRobotConfig = new RobotConfig(state.robotConfig.getPos(), nextConfig.getOrientation());
        }

        List<Line2D> movingLines = new ArrayList<>();

        Point2D r1p1 = tester.getPoint1(tempRobotConfig);
        Point2D r1p2 = tester.getPoint2(tempRobotConfig);
        Point2D r2p1 = tester.getPoint1(nextConfig);
        Point2D r2p2 = tester.getPoint2(nextConfig);

        movingLines.add(new Line2D.Double(r1p1, r1p2));
        movingLines.add(new Line2D.Double(r2p1, r2p2));
        movingLines.add(new Line2D.Double(r1p1, r2p1));
        movingLines.add(new Line2D.Double(r1p1, r2p2));
        movingLines.add(new Line2D.Double(r1p2, r2p1));
        movingLines.add(new Line2D.Double(r1p2, r2p2));

        if (!border.contains(r1p1) || !border.contains(r1p2) || !border.contains(r2p1) || !border.contains(r2p2)) {
            return false;
        }

        for (Line2D line: movingLines) {

            for (Box box : state.movingBoxes) {
                if (line.intersects(box.getRect())) {
                    return false;
                }
            }

            for (Box box: state.movingObstacles) {
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

        return true;
    }

    private Set<Point2D> getPointsAroundRectangle(Rectangle2D rectangle, double delta) {
        //sample 8 points(4 vertices and 4 at the middle of vertices) around the object


        //Todo: add 8 more point that help the robot to rotate through the corner
        Set<Point2D> pointList = new HashSet<>();
        Rectangle2D boundary = tester.grow(rectangle, delta);

        Point2D topLeft = new Point2D.Double();
        Point2D topRight = new Point2D.Double();
        Point2D bottomLeft = new Point2D.Double();
        Point2D bottomRight = new Point2D.Double();
        Point2D midUp = new Point2D.Double();
        Point2D midDown = new Point2D.Double();
        Point2D midLeft = new Point2D.Double();
        Point2D midRight = new Point2D.Double();


        topLeft.setLocation(boundary.getMaxX(), boundary.getMinY());
        topRight.setLocation(boundary.getMaxX(), boundary.getMaxY());
        bottomLeft.setLocation(boundary.getMinX(), boundary.getMinY());
        bottomRight.setLocation(boundary.getMinX(), boundary.getMaxY());
        midUp.setLocation((boundary.getMaxX() + boundary.getMinX()) / 2, boundary.getMaxY() + Tester.MAX_BASE_STEP);
        midDown.setLocation((boundary.getMaxX() + boundary.getMinX()) / 2, boundary.getMinY() - Tester.MAX_BASE_STEP);
        midLeft.setLocation(boundary.getMinX() - Tester.MAX_BASE_STEP, (boundary.getMinY() + boundary.getMaxY()) / 2);
        midRight.setLocation(boundary.getMaxX() + Tester.MAX_BASE_STEP, (boundary.getMinY() + boundary.getMaxY()) / 2);

        pointList.add(topLeft);
        pointList.add(topRight);
        pointList.add(bottomLeft);
        pointList.add(bottomRight);
        pointList.add(midUp);
        pointList.add(midDown);
        pointList.add(midLeft);
        pointList.add(midRight);

        double error = Tester.MAX_ERROR;
        Point2D leftUp = new Point2D.Double();
        Point2D rightUp = new Point2D.Double();
        Point2D leftDown = new Point2D.Double();
        Point2D rightDown = new Point2D.Double();
        Point2D upLeft = new Point2D.Double();
        Point2D downLeft = new Point2D.Double();
        Point2D upRight = new Point2D.Double();
        Point2D downRight = new Point2D.Double();

        leftUp.setLocation(rectangle.getMinX() - error, boundary.getMaxY());
        rightUp.setLocation(rectangle.getMaxX() + error, boundary.getMaxY());
        leftDown.setLocation(rectangle.getMinX() - error, boundary.getMinY());
        rightDown.setLocation(rectangle.getMaxX() + error, boundary.getMinY());
        upLeft.setLocation(boundary.getMinX(), rectangle.getMaxY() + error);
        downLeft.setLocation(boundary.getMinX(), rectangle.getMinY() - error);
        upRight.setLocation(boundary.getMaxX(), rectangle.getMaxY() + error);
        downRight.setLocation(boundary.getMaxX(), rectangle.getMaxY() - error);

        pointList.add(leftUp);
        pointList.add(rightUp);
        pointList.add(leftDown);
        pointList.add(rightDown);
        pointList.add(upLeft);
        pointList.add(downLeft);
        pointList.add(upRight);
        pointList.add(downRight);

        return pointList;

    }


    private Set<Point2D> getPointsAroundObstacles(List<Rectangle2D> rectangles, double delta) {

        Set<Point2D> points = new HashSet<>();
        for (Rectangle2D rectangle : rectangles) {
            points.addAll(getPointsAroundRectangle(rectangle, delta));
        }

        return points;
    }

    private List<Rectangle2D> getObstacles(State currentState) {

        List<Rectangle2D> obstacles = new ArrayList<>();
        Line2D connectionLine = new Line2D.Double(currentState.robotConfig.getPos(), targetConfig.getPos());

        for (Box box : currentState.movingBoxes) {
            Point2D center = new Point2D.Double(box.getPos().getX() + box.getWidth(), box.getPos().getY() + box.getWidth());

            if (targetConfig.getPos().distance(center) <= box.getWidth()) {
                obstacles.add(box.getRect());
            }

            if (currentState.robotConfig.getPos().distance(center) <= box.getWidth()) {
                obstacles.add(box.getRect());
            }
            if (connectionLine.intersects(box.getRect())) {
                obstacles.add(box.getRect());
            }
        }

        for (Box box : currentState.movingObstacles) {
            Point2D center = new Point2D.Double(box.getPos().getX() + box.getWidth(), box.getPos().getY() + box.getWidth());

            if (targetConfig.getPos().distance(center) <= box.getWidth()) {
                obstacles.add(box.getRect());
            }

            if (currentState.robotConfig.getPos().distance(center) <= box.getWidth()) {
                obstacles.add(box.getRect());
            }

            if (connectionLine.intersects(box.getRect())) {
                obstacles.add(box.getRect());
            }
        }

        for (StaticObstacle box : staticObstacles) {

            Line2D robotLine = new Line2D.Double(tester.getPoint1(targetConfig), tester.getPoint2(targetConfig));

            if (tester.grow(box.getRect(), robotWidth/2).intersectsLine(robotLine)) {
                obstacles.add(box.getRect());
            }

            if (connectionLine.intersects(box.getRect())) {
                obstacles.add(box.getRect());
            }
        }

        return obstacles;
    }
    
}
