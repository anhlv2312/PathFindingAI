package botmate;

import problem.*;

import java.awt.geom.Point2D;
import java.util.*;

public class RobotAgent extends SearchAgent {

    MovingBox target;

    public RobotAgent(ProblemSpec ps, State initialState, MovingBox target) {
        super(ps, initialState);
        this.target = target;
    }

    @Override
    public boolean isFound(State currentState) {
        return (tester.isCoupled(currentState.robotConfig, target) > 0);
    }

    public double calculateCost(RobotConfig currentConfig, RobotConfig nextConfig) {
        double distance = currentConfig.getPos().distance(nextConfig.getPos());
        double rotation = currentConfig.getOrientation() - nextConfig.getOrientation() * robotWidth / 2;
        return distance + rotation;
    }

    public double calculateHeuristic(RobotConfig currentConfig) {
        double distance = currentConfig.getPos().distance(target.getPos());
        return distance;
    }

    @Override
    public List<SearchNode> getSuccessors(State currentState) {

        double[] orientations = new double[]{0, Math.PI * 0.25, Math.PI * 0.5, Math.PI * 0.75};

        List<Point2D> positions = getPointAroundObstacles(currentState, robotWidth/2);
        positions.addAll(getPossibleRobotPoint(target));

        List<RobotState> states = new ArrayList<>();
        RobotState tempState;
        for (Point2D position: positions) {
            for (double orientation: orientations) {
                tempState = currentState.moveRobotToPosition(position, orientation);
                if (checkRobotMovingCollision(currentState, tempState.robotConfig)) {
                    states.add(tempState);
                }
            }
        }

        List<SearchNode> nodes = new ArrayList<>();
        for (RobotState state: states) {
            double cost = calculateCost(currentState.robotConfig, state.robotConfig);
            double heuristic = calculateHeuristic(state.robotConfig);
            nodes.add(new SearchNode(state, cost, heuristic));
        }

        return nodes;
    }

}
