package botmate;

import problem.Box;
import problem.ProblemSpec;
import problem.RobotConfig;
import problem.StaticObstacle;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

public class RobotAgent extends SearchAgent {

    public RobotAgent(ProblemSpec ps) {
        super(ps);
    }

    private boolean isFound(RobotState currentState, RobotState goalState) {
        return checkRobotCollision(currentState, goalState);
    }

    private boolean checkRobotCollision(RobotState state1, RobotState state2) {

        List<Line2D> robotLines = new ArrayList<>();

        // Get robot config
        RobotConfig r1 = state1.robotConfig;
        RobotConfig r2 = state2.robotConfig;

        Line2D robotLine1 = new Line2D.Double(tester.getPoint1(r1), tester.getPoint2(r1));
        Line2D robotLine2 = new Line2D.Double(tester.getPoint1(r2), tester.getPoint2(r2));

        // check rotate and then check two line line
        // check with boundary
        List<Point2D> robotBound1 = getVertices(tester.grow(robotLine1.getBounds2D(), robotWidth / 5));
        List<Point2D> robotBound2 = getVertices(tester.grow(robotLine2.getBounds2D(), robotWidth / 5));

        robotLines.add(new Line2D.Double(r1.getPos(), r2.getPos()));

        for (int i = 0; i < robotBound1.size(); i++) {
            robotLines.add(new Line2D.Double(robotBound1.get(i), robotBound2.get(i)));
        }

        // Check collision between every line and obstacle
        for (Line2D line : robotLines) {

            for (Box box : state2.movingObstacles) {
                if (line.intersects(box.getRect())) {
                    return true;
                }
            }

            for (StaticObstacle obstacle : staticObstacles) {
                if (line.intersects(obstacle.getRect())) {
                    return true;
                }
            }

        }

        return false;


    }

}
