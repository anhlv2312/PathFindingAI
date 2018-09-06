package botmate;

import problem.*;
import tester.Tester;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.*;

public class RobotAgent extends SearchAgent {

    private Box targetBox;
    private int targetEdge;
    private RobotConfig targetConfig;

    public RobotAgent(ProblemSpec ps, State initialState, Box movingBox, int targetEdge) {
        super(ps, initialState);
        this.targetBox = movingBox;
        this.targetEdge = targetEdge;
        this.targetConfig = initialState.moveRobotToBox(movingBox, targetEdge).robotConfig;
    }

    @Override
    public boolean isFound(State currentState) {
        return (tester.isCoupled(currentState.robotConfig, targetBox) == targetEdge);
    }

    public double calculateCost(RobotConfig currentConfig, RobotConfig nextConfig) {
        double distance = currentConfig.getPos().distance(nextConfig.getPos());
        double rotation = (currentConfig.getOrientation() - nextConfig.getOrientation()) * robotWidth / 2;
        return distance + rotation;
    }

    public double calculateHeuristic(RobotConfig currentConfig) {
        //todo: Fix this, should not be target box, should
        double distance = currentConfig.getPos().distance(targetConfig.getPos());
        return distance;
    }

    @Override
    public List<SearchNode> getSuccessors(State currentState) {

        double[] orientations = new double[]{0, Math.PI * 0.5, };
        List<Point2D> positions = new ArrayList<>();

        if (currentState.robotConfig.getPos().distance(targetConfig.getPos()) < robotWidth) {
            //todo: do something here
            positions.addAll(getPointsAroundRectangle(targetBox.getRect(), Tester.MAX_ERROR));
//            positions.add(targetConfig.getPos());
        }

        positions.addAll(getPointsAroundObstacles(currentState, robotWidth/2 + Tester.MAX_ERROR));
//        positions.addAll(getPointsAroundObstacles(currentState, Tester.MAX_ERROR));

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
//            System.out.println(state.toString());
        }

        return nodes;
    }


    public boolean checkRobotMovingCollision(State state, RobotConfig nextConfig) {

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
                if (tester.isCoupled(state.robotConfig, box) > 0) {
                    if (line.intersects(tester.grow(box.getRect(), -Tester.MAX_ERROR))) {
                        return false;
                    }
                } else if (line.intersects(box.getRect())) {
                    return false;
                }
            }

            for (Box box: state.movingObstacles) {
                if (tester.isCoupled(state.robotConfig, box) > 0) {
                    if (line.intersects(tester.grow(box.getRect(), -Tester.MAX_ERROR))) {
                        return false;
                    }
                } else if (line.intersects(box.getRect())) {
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


    public List<Point2D> getPointsAroundRectangle(Rectangle2D rectangle, double delta) {
        //sample 8 points(4 vertices and 4 at the middle of vertices) around the object

        List<Point2D> pointList = new ArrayList<>();
        Rectangle2D rect = tester.grow(rectangle, delta);

        Point2D topLeft = new Point2D.Double();
        Point2D topRight = new Point2D.Double();
        Point2D bottomLeft = new Point2D.Double();
        Point2D bottomRight = new Point2D.Double();
        Point2D midUp = new Point2D.Double();
        Point2D midDown = new Point2D.Double();
        Point2D midLeft = new Point2D.Double();
        Point2D midRight = new Point2D.Double();

        topLeft.setLocation(rect.getMaxX(), rect.getMinY());
        topRight.setLocation(rect.getMaxX(), rect.getMaxY());
        bottomLeft.setLocation(rect.getMinX(), rect.getMinY());
        bottomRight.setLocation(rect.getMinX(), rect.getMaxY());
        midUp.setLocation((rect.getMaxX() + rect.getMinX()) / 2, rect.getMaxY());
        midDown.setLocation((rect.getMaxX() + rect.getMinX()) / 2, rect.getMinY());
        midLeft.setLocation(rect.getMinX(), (rect.getMinY() + rect.getMaxY()) / 2);
        midRight.setLocation(rect.getMaxX(), (rect.getMinY() + rect.getMaxY()) / 2);

        pointList.add(topLeft);
        pointList.add(topRight);
        pointList.add(bottomLeft);
        pointList.add(bottomRight);
        pointList.add(midUp);
        pointList.add(midDown);
        pointList.add(midLeft);
        pointList.add(midRight);

        return pointList;

    }

    public List<Point2D> getPointsAroundObstacles(State currentState, double delta) {
        List<Point2D> points = new ArrayList<>();

        for (Box box : currentState.movingBoxes) {
            points.addAll(getPointsAroundRectangle(box.getRect(), delta));
        }

        for (Box box : currentState.movingObstacles) {
            points.addAll(getPointsAroundRectangle(box.getRect(), delta));
        }
        for (StaticObstacle obstacle : staticObstacles) {
            points.addAll(getPointsAroundRectangle(obstacle.getRect(), delta));
        }
        return points;
    }

    
}
