package botmate;

import problem.Box;
import problem.MovingObstacle;
import problem.RobotConfig;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

public class RobotState extends State {

    RobotConfig robotConfig;
    List<Box> movingObstacles;

    public RobotState(RobotConfig robotConfig, List<Box> movingObstacles) {
        super(robotConfig, null, movingObstacles);
        this.robotConfig = new RobotConfig(robotConfig.getPos(), robotConfig.getOrientation());
        this.movingObstacles = new ArrayList<>();
        for (Box box: movingObstacles) {
            this.movingObstacles.add(new MovingObstacle(box.getPos(), box.getWidth()));
        }
    }

    public List<Box> getMovingObstacles() {
        return movingObstacles;
    }

    public RobotState moveRobotToPosition(Point2D position, double orientation) {
        RobotConfig newRobotConfig = new RobotConfig(robotConfig.getPos(), orientation);
        newRobotConfig.getPos().setLocation(position);
        RobotState newRobotState = new RobotState(newRobotConfig, movingObstacles);
        return newRobotState;
    }

    public RobotState moveRobot(double deltaX, double deltaY, double deltaO) {
        double newX = robotConfig.getPos().getX() + deltaX;
        double newY = robotConfig.getPos().getY() + deltaY;
        double newOrientation = robotConfig.getOrientation() + deltaO;
        return moveRobotToPosition(new Point2D.Double(newX, newY), newOrientation);
    }

}
