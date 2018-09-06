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

    public String toString() {
        StringBuilder output = new StringBuilder();

        String doubleFormat = "%.6f";
        output.append(String.format(doubleFormat + " ", robotConfig.getPos().getX()));
        output.append(String.format(doubleFormat + " ", robotConfig.getPos().getY()));
        output.append(String.format(doubleFormat + " ", robotConfig.getOrientation()));

        for (Box box: movingBoxes) {
            output.append(String.format(doubleFormat + " ", box.getPos().getX() + box.getWidth()/2));
            output.append(String.format(doubleFormat + " ", box.getPos().getY() + box.getWidth()/2));
        }

        for (Box box: movingObstacles) {
            output.append(String.format(doubleFormat + " ", box.getPos().getX() + box.getWidth()/2));
            output.append(String.format(doubleFormat + " ", box.getPos().getY() + box.getWidth()/2));
        }

        return output.toString();
    }

    public State moveRobotToPosition(Point2D position, double orientation) {
        RobotConfig newRobotConfig = new RobotConfig(robotConfig.getPos(), orientation);
        newRobotConfig.getPos().setLocation(position);
        State newRobotState = new State(newRobotConfig, movingBoxes, movingObstacles);
        return newRobotState;
    }

    public State moveRobot(double deltaX, double deltaY, double deltaO) {
        double newX = robotConfig.getPos().getX() + deltaX;
        double newY = robotConfig.getPos().getY() + deltaY;
        double newOrientation = robotConfig.getOrientation() + deltaO;
        return moveRobotToPosition(new Point2D.Double(newX, newY), newOrientation);
    }

    public State moveMovingBoxToPosition(int movingBoxIndex, Point2D position) {
        State newBoxState = new State(robotConfig, movingBoxes, movingObstacles);
        newBoxState.movingBoxes.get(movingBoxIndex).getPos().setLocation(position);
        return newBoxState;
    }

    public State moveMovingBox(int movingBoxIndex, double deltaX, double deltaY, int robotPosition) {
        Point2D currentPosition = movingBoxes.get(movingBoxIndex).getPos();
        Point2D newPosition = new Point2D.Double(currentPosition.getX() + deltaX, currentPosition.getY() + deltaY);
        State tempState = moveMovingBoxToPosition(movingBoxIndex, newPosition);
        tempState = tempState.moveRobotToMovingBox(movingBoxIndex, robotPosition);
        return tempState;
    }

    public State moveObstacleToPosition(int movingObstacleIndex, Point2D position) {
        State newBoxState = new State(robotConfig, movingBoxes, movingObstacles);
        newBoxState.movingObstacles.get(movingObstacleIndex).getPos().setLocation(position);
        return newBoxState;
    }

    public State moveObstacle(int movingObstacleIndex, double deltaX, double deltaY, int robotPosition) {
        Point2D currentPosition = movingObstacles.get(movingObstacleIndex).getPos();
        Point2D newPosition = new Point2D.Double(currentPosition.getX() + deltaX, currentPosition.getY() + deltaY);
        State tempState = moveObstacleToPosition(movingObstacleIndex, newPosition);
        tempState = tempState.moveRobotToObstacle(movingObstacleIndex, robotPosition);
        return tempState;
    }

    public State moveRobotToMovingBox(int movingBoxIndex, int robotPosition) {
        Box box =  this.movingBoxes.get(movingBoxIndex);
        return this.moveRobotToBox(box, robotPosition);
    }


    public State moveRobotToObstacle(int movingObstacleIndex, int robotPosition) {
        Box box =  this.movingObstacles.get(movingObstacleIndex);
        return this.moveRobotToBox(box, robotPosition);
    }

    private State moveRobotToBox(Box box, int edge){
        double w, x, y, o;
        Point2D p;

        w = box.getWidth();
        x = box.getPos().getX();
        y = box.getPos().getY();

        switch (edge) {
            case 1:
                p = new Point2D.Double(x + w/2, y);
                o = 0.0;
                break;
            case 2:
                p = new Point2D.Double(x, y + w/2);
                o = Math.PI/2;
                break;
            case 3:
                p = new Point2D.Double(x + w/2, y + w);
                o = 0.0;
                break;
            case 4:
                p = new Point2D.Double(x + w, y + w/2);
                o = Math.PI/2;
                break;
            default:
                p = this.robotConfig.getPos();
                o = this.robotConfig.getOrientation();
        }
        return this.moveRobotToPosition(p, o);
    }

    public State moveRobotOutOfBox(int robotPosition, double delta) {
        switch (robotPosition) {
            case 1:
                return this.moveRobot(0, -delta, 0);
            case 2:
                return this.moveRobot(-delta, 0, 0);
            case 3:
                return this.moveRobot(0, delta, 0);
            case 4:
                return this.moveRobot(delta, 0, 0);
            default:
                return this;
        }

    }

}
