package botmate;

import problem.Box;
import problem.MovingObstacle;
import problem.RobotConfig;

import java.awt.geom.Point2D;
import java.util.List;

public class RobotState implements State {

    public  RobotConfig robotConfig;
    public List<Box> movingObstacles;

    public RobotState(RobotConfig robotConfig, List<Box> movingObstacles) {
        this.robotConfig = robotConfig;

        for (Box box: movingObstacles) {
            this.movingObstacles.add(new MovingObstacle(box.getPos(), box.getWidth()));

        }
    }

    public List<Successor> getSuccessors() {
        return null;
    }

    public List<Successor> getSuccessors(State goalState) {
        //TODO: write successor;
        return null;
    }

    public RobotState moveRobotToPosition(Point2D position, double orientation) {
        RobotConfig newRobotConfig = new RobotConfig(position, orientation);
        return new RobotState(newRobotConfig, movingObstacles);
    }

    public RobotState moveRobot(double deltaX, double deltaY, double deltaO) {
        Point2D position = new Point2D.Double(robotConfig.getPos().getX() + deltaX, robotConfig.getPos().getY() + deltaY);
        double orientation = robotConfig.getOrientation() + deltaO;
        return moveRobotToPosition(position, orientation);
    }
}
