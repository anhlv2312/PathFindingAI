package botmate;

import problem.Box;
import problem.ProblemSpec;
import problem.RobotConfig;
import problem.StaticObstacle;
import tester.Tester;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.*;

public abstract class SearchAgent {

    PriorityQueue<SearchNode> container;
    Tester tester;
    double robotWidth;
    State initialState;
    List<StaticObstacle> staticObstacles;

    public SearchAgent(ProblemSpec ps, State initialState) {
        tester = new Tester(ps);
        robotWidth = ps.getRobotWidth();
        staticObstacles = ps.getStaticObstacles();
        container = new PriorityQueue<>();
        this.initialState = initialState;
    }

    public abstract boolean isFound(State currentState);
    public abstract List<SearchNode>  getSuccessors(State currentState);

    public List<State> search() {

        Set<String> visited = new HashSet<>();
        SearchNode initialNode = new SearchNode(initialState);

        container.add(initialNode);

        while (!container.isEmpty()) {

            //the node in having the lowest f_score value
            SearchNode currentNode = container.poll();

            State currentState = currentNode.state;

            visited.add(currentState.toString());
            //goal found

            if (isFound(currentState)) {
                List<State> pathToGoal = new LinkedList<>();
                while (currentNode.parent != null) {
                    pathToGoal.add(currentNode.state);
                    currentNode = currentNode.parent;
                }

                pathToGoal.add(initialState);
                Collections.reverse(pathToGoal);

                // reset for next search
                container.clear();

                return pathToGoal;
            }

            //check every child of current node

            List<SearchNode> nodes = getSuccessors(currentState);

            for (SearchNode node : nodes) {


                double newTotalCost = currentNode.totalCost + node.cost;
                double priority = newTotalCost + node.heuristic;

                if ((visited.contains(node.state.toString())) &&
                        (priority >= node.totalCost)) {
                    continue;
                }

                else if ((!container.contains(node.state.toString())) ||
                        (priority < node.totalCost)) {
                    node.parent = currentNode;
                    node.totalCost = newTotalCost;
                    node.priority = priority;
                    if (container.contains(node.state.toString())) {
                        container.remove(node.state.toString());
                    }
                    container.add(node);
                }

            }

        }

        System.out.println("Not found");

        return null;

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
}
