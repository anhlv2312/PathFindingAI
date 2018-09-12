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
        positions.addAll(getPointsAroundObstacles(getObstacles(currentState), robotWidth/2 + Tester.MAX_ERROR));

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
            List<Point2D> robotPoints = getPointsAroundRectangle(robotRect, 0);

            for (Point2D point: robotPoints) {
                if (!border.contains(point)) {
                    return false;
                }
            }

            for (Box box: state.movingBoxes) {
                if (tester.isCoupled(state.robotConfig, box) > 0) {
                    if (robotRect.intersects(tester.grow(box.getRect(), -Tester.MAX_ERROR))) {
                        return false;
                    }
                } else if (robotRect.intersects(box.getRect())) {
                    return false;
                }
            }

            for (Box box: state.movingObstacles) {
                if (tester.isCoupled(state.robotConfig, box) > 0) {
                    if (robotRect.intersects(tester.grow(box.getRect(), -Tester.MAX_ERROR))) {
                        return false;
                    }
                } else if (robotRect.intersects(box.getRect())) {
                    return false;
                }
            }

            for (StaticObstacle obstacle: staticObstacles) {
                if (robotRect.intersects(obstacle.getRect())) {
                    return false;
                } else if (robotRect.intersects(obstacle.getRect())) {
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
            for (Box box: state.movingBoxes) {
                if (line.intersects(tester.grow(box.getRect(), -Tester.MAX_ERROR))) {
                    return false;
                }
            }

            for (Box box: state.movingObstacles) {
                if (line.intersects(tester.grow(box.getRect(), -Tester.MAX_ERROR))) {
                    return false;
                }
            }

            for (StaticObstacle obstacle: staticObstacles) {
                if (line.intersects(obstacle.getRect())) {
                    return false;
                } else if (line.intersects(obstacle.getRect())) {
                    return false;
                }
            }
        }

        return true;
    }

    private List<Point2D> getPointsAroundRectangle(Rectangle2D rectangle, double delta) {
        //sample 8 points(4 vertices and 4 at the middle of vertices) around the object


        //Todo: add 8 more point that help the robot to rotate through the corner
        List<Point2D> pointList = new ArrayList<>();
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
        midUp.setLocation((boundary.getMaxX() + boundary.getMinX()) / 2, boundary.getMaxY());
        midDown.setLocation((boundary.getMaxX() + boundary.getMinX()) / 2, boundary.getMinY());
        midLeft.setLocation(boundary.getMinX(), (boundary.getMinY() + boundary.getMaxY()) / 2);
        midRight.setLocation(boundary.getMaxX(), (boundary.getMinY() + boundary.getMaxY()) / 2);

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
        rightDown.setLocation(rectangle.getMinX() + error, boundary.getMinY());
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

    private List<Point2D> getPointsAroundObstacles(List<Box> obstacles, double delta) {

        List<Point2D> points = new ArrayList<>();
        for (Box obstacle : obstacles) {
            points.addAll(getPointsAroundRectangle(obstacle.getRect(), delta));
        }

        for (StaticObstacle obstacle : staticObstacles) {
            points.addAll(getPointsAroundRectangle(obstacle.getRect(), delta));
        }

        return points;
    }

    private List<Box> getObstacles(State currentState) {

        List<Box> obstacles = new ArrayList<>();
        Line2D connectionLine = new Line2D.Double(currentState.robotConfig.getPos(), targetConfig.getPos());

        for (Box box : currentState.movingBoxes) {
            if (connectionLine.intersects(box.getRect())) {
                obstacles.add(box);
            }
        }

        for (Box box : currentState.movingObstacles) {
            if (connectionLine.intersects(box.getRect())) {
                obstacles.add(box);
            }
        }

        return obstacles;
    }
    
}
