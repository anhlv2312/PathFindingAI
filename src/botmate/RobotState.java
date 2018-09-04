package botmate;

import problem.Box;
import problem.MovingObstacle;
import problem.RobotConfig;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

public class RobotState implements State {

    public  RobotConfig robotConfig;
    public List<Box> movingObstacles;

    public RobotState(RobotConfig robotConfig, List<Box> movingObstacles) {
        this.robotConfig = robotConfig;

        this.movingObstacles = new ArrayList<>();
        for (Box box: movingObstacles) {
            this.movingObstacles.add(new MovingObstacle(box.getPos(), box.getWidth()));
        }
    }

    public List<SearchNode> getSuccessors() {
        return null;
    }

    public List<SearchNode> getSuccessors(State goalState) {
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

    @Override
    public String toString() {

        StringBuilder output = new StringBuilder();

        output.append(String.format("%.5f ", this.robotConfig.getPos().getX()));
        output.append(String.format("%.5f ", this.robotConfig.getPos().getY()));
        output.append(String.format("%.5f ", this.robotConfig.getOrientation()));

        for (Box box: movingObstacles) {
            output.append(String.format("%.5f ", box.getPos().getX() + box.getWidth()/2));
            output.append(String.format("%.5f ", box.getPos().getY() + box.getWidth()/2));
        }

        return output.toString();
    }
}
