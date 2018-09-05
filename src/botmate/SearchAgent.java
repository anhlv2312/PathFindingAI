package botmate;

import problem.Box;
import problem.ProblemSpec;
import problem.StaticObstacle;
import tester.Tester;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.*;

public class SearchAgent {

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

    private static List<Point2D> getPointsAroundRectangle(Rectangle2D rect) {
        //sample 8 points(4 vertices and 4 at the middle of vertices) around the object

        List<Point2D> pointList = new ArrayList<>();

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

        for (Box b : currentState.getMovingObstacles()) {
            obstacleList.add(b.getRect());
        }

        //create samples around each obstacles

        for (Rectangle2D rect : obstacleList) {
            Rectangle2D grownRec = tester.grow(rect, delta);
            points.addAll(getPointsAroundRectangle(grownRec));
        }

        return points;
    }
}
