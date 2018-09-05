package botmate;

import problem.*;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.*;

public class RobotAgent extends SearchAgent {

    Box targetBox;

    public RobotAgent(ProblemSpec ps, State initialState, Box targetBox) {
        super(ps, initialState);
        this.targetBox = targetBox;
    }

    @Override
    public boolean isFound(State currentState) {
        return (tester.isCoupled(currentState.robotConfig, targetBox) > 0);
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

        double[] orientations = new double[]{0, Math.PI * 0.25, Math.PI * 0.5, Math.PI * 0.75};

        List<Point2D> positions = new ArrayList<>();
        positions.addAll(getPointAroundObstacles(currentState, robotWidth/2));
        positions.addAll(getPointAroundObstacles(currentState, tester.MAX_ERROR));
        positions.addAll(getPointsAroundRectangle(targetBox.getRect(), tester.MAX_ERROR));

        List<State> states = new ArrayList<>();
        State tempState;
        for (Point2D position: positions) {
            for (double orientation: orientations) {
                tempState = currentState.moveRobotToPosition(position, orientation);
                if (checkRobotMovingCollision(currentState, tempState.robotConfig)) {
                    states.add(tempState);
                    break;
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
        RobotConfig currentConfig = state.robotConfig;

        Point2D r1p1 = tester.getPoint1(currentConfig);
        Point2D r1p2 = tester.getPoint2(currentConfig);
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
                if (line.intersects(box.getRect())) {
                    return false;
                }
            }
        }

        for (Line2D line: movingLines) {
            for (Box box: state.movingObstacles) {
                if (line.intersects(box.getRect())) {
                    return false;
                }
            }
        }

        for (Line2D line: movingLines) {
            for (StaticObstacle obstacle: staticObstacles) {
                if (line.intersects(obstacle.getRect())) {
                    return false;
                }
            }
        }

        if ((currentConfig.getOrientation() - nextConfig.getOrientation()) != 0) {
            Rectangle2D robotRect;
            double bottomLeftX = currentConfig.getPos().getX()-robotWidth/2;
            double bottomLeftY = currentConfig.getPos().getY()-robotWidth/2;
            robotRect = new Rectangle2D.Double(bottomLeftX, bottomLeftY, robotWidth, robotWidth);

            for (Box box: state.movingObstacles) {
                if (robotRect.intersects(box.getRect())) {
                    return false;
                }
            }
        }

        return true;
    }


}
