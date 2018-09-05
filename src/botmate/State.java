package botmate;

import problem.Box;
import problem.MovingBox;
import problem.MovingObstacle;
import problem.RobotConfig;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

public class State {

    RobotConfig robotConfig;
    List<Box> movingBoxes;
    List<Box> movingObstacles;

    public State(RobotConfig robotConfig, List<Box> movingBoxes, List<Box> movingObstacles) {

        this.robotConfig = new RobotConfig(robotConfig.getPos(), robotConfig.getOrientation());

        this.movingBoxes = new ArrayList<>();
        for (Box box : movingBoxes) {
            this.movingBoxes.add(new MovingBox(box.getPos(), box.getWidth()));
        }

        this.movingObstacles = new ArrayList<>();
        for (Box box: movingObstacles) {
            this.movingObstacles.add(new MovingObstacle(box.getPos(), box.getWidth()));
        }

    }


    List<Box> getMovingObstacles() {
        return movingObstacles;
    }

    public String toString() {

        StringBuilder output = new StringBuilder();

        output.append(String.format("%.5f ", robotConfig.getPos().getX()));
        output.append(String.format("%.5f ", robotConfig.getPos().getY()));
        output.append(String.format("%.5f ", robotConfig.getOrientation()));

        for (Box box: movingBoxes) {
            output.append(String.format("%.5f ", box.getPos().getX() + box.getWidth()/2));
            output.append(String.format("%.5f ", box.getPos().getY() + box.getWidth()/2));
        }

        for (Box box: movingObstacles) {
            output.append(String.format("%.5f ", box.getPos().getX() + box.getWidth()/2));
            output.append(String.format("%.5f ", box.getPos().getY() + box.getWidth()/2));
        }

        return output.toString();
    }

    public State moveRobotToPosition(Point2D position, double orientation) {
        RobotConfig newRobotConfig = new RobotConfig(robotConfig.getPos(), orientation);
        newRobotConfig.getPos().setLocation(position);
        State newRobotState = new State(newRobotConfig, movingBoxes, movingObstacles);
        return newRobotState;
    }

//    public RobotState moveRobot(double deltaX, double deltaY, double deltaO) {
//        double newX = robotConfig.getPos().getX() + deltaX;
//        double newY = robotConfig.getPos().getY() + deltaY;
//        double newOrientation = robotConfig.getOrientation() + deltaO;
//        return moveRobotToPosition(new Point2D.Double(newX, newY), newOrientation);
//    }


//    public State moveBoxToPosition(Point2D position) {
//        State newBoxState = new State(robotConfig, movingBoxes, movingObstacles);
//        newBoxState.movingBoxes.get().getPos().setLocation(position);
//        return newBoxState;
//    }
//
//    public State moveBox(double deltaX, double deltaY, int robotPosition) {
//        Point2D position = new Point2D.Double(movingBox.getPos().getX() + deltaX, movingBox.getPos().getY() + deltaY);
//        return moveBoxToPosition(position);
//    }


}
