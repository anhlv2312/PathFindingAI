package botmate;

import problem.*;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.*;

public class RobotAgent extends SearchAgent {

    RobotConfig target;

    public RobotAgent(ProblemSpec ps, State initialState, RobotConfig target) {
        super(ps, initialState);
        this.target = target;
    }

    private boolean isFound(RobotState currentState) {
        return checkRobotMovingCollision(currentState, target);
    }

    public double calculateRobotCost(RobotConfig currentConfig, RobotConfig nextConfig) {
        double distance = currentConfig.getPos().distance(nextConfig.getPos());
        double rotation = Math.abs(currentConfig.getOrientation() - nextConfig.getOrientation()) * robotWidth/2;
        return distance + rotation;
    }

    private List<SearchNode> getSuccessors(RobotState currentState) {

        double delta = robotWidth/2;
        double[] orientations = new double[]{0.0, 0.25, 0.5, 0.75};

        List<RobotState> states = new ArrayList<>();
        for (double o: orientations) {
            states.add(currentState.moveRobot(delta, 0, Math.PI * o));
            states.add(currentState.moveRobot(0, delta, Math.PI * o));
            states.add(currentState.moveRobot(-delta, 0, Math.PI * o));
            states.add(currentState.moveRobot(0, -delta, Math.PI * o));
        }

        List<SearchNode> nodes = new ArrayList<>();

        for (RobotState state: states) {
            if (tester.hasCollision(state.robotConfig, state.movingObstacles) &&
                    checkRobotMovingCollision(currentState, state.robotConfig)) {
                double gCost = calculateRobotCost(currentState.robotConfig, state.robotConfig);
                double hCost = calculateRobotCost(state.robotConfig, target);
                nodes.add(new SearchNode(state, gCost, hCost));
            }
        }

        return nodes;
    }

    private boolean checkRobotMovingCollision(RobotState state, RobotConfig nextConfig) {

        List<Line2D> movingLines = new ArrayList<>();
        // Get robot config
        RobotConfig currentConfig = state.robotConfig;

        Point2D r1p1 = tester.getPoint1(currentConfig);
        Point2D r1p2 = tester.getPoint2(currentConfig);
        Point2D r2p1 = tester.getPoint1(nextConfig);
        Point2D r2p2 = tester.getPoint2(nextConfig);


        movingLines.add(new Line2D.Double(r1p1, r2p1));
        movingLines.add(new Line2D.Double(r1p1, r2p2));
        movingLines.add(new Line2D.Double(r1p2, r2p1));
        movingLines.add(new Line2D.Double(r1p2, r2p2));


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

    public List<State> search() {


        Set<String> visited = new HashSet<>();
        SearchNode initialNode = new SearchNode(initialState);

        container.add(initialNode);

        while (!container.isEmpty()) {

            //the node in having the lowest f_score value
            SearchNode currentNode = container.poll();


            RobotState currentState = (RobotState)currentNode.state;

            visited.add(currentState.toString());

            //goal found
            if (isFound(currentState)) {
                System.out.println("Found:" + currentState.toString() + " [" + container.size() );
                List<State> pathToGoal = new LinkedList<>();
                while (currentNode.parent != null) {
                    pathToGoal.add(currentNode.state);
                    currentNode = currentNode.parent;
                }
                Collections.reverse(pathToGoal);

                // reset for next search
                container.clear();

                return pathToGoal;
            }

            //check every child of current node

            System.out.println(currentState.toString());
            List<SearchNode> nodes = getSuccessors(currentState);

            for (SearchNode node : nodes) {
                State child = node.state;
                double tempGCost = currentNode.gCost + node.gCost;
                double tempFCost = tempGCost + node.hCost;

                if ((visited.contains(child.toString())) &&
                        (tempGCost >= node.fCost)) {
                    continue;
                }
                else if ((!container.contains(child)) ||
                        (tempGCost < node.fCost)) {
                    node.parent = currentNode;
                    node.gCost = tempGCost;
                    node.fCost = tempFCost;
                    if (container.contains(child)) {
                        container.remove(child);
                    }
                    container.add(node);
                }
            }

        }

        return null;

    }

}
