package botmate;

import problem.Box;
import problem.MovingBox;
import problem.MovingObstacle;

import java.awt.geom.Point2D;
import java.util.List;

public class BoxState implements State{

    public int robotPosition;
    public Box movingBox;
    public List<Box> movingObstacles;

    public BoxState(Box movingBox, int robotPosition, List<Box> movingObstacles) {

        this.movingBox = new MovingBox(movingBox.getPos(), movingBox.getWidth());
        this.robotPosition = robotPosition;
        for (Box box: movingObstacles) {
            this.movingObstacles.add(new MovingObstacle(box.getPos(), box.getWidth()));
        }

    }

    public List<SearchNode> getSuccessors() {
        return null;
    }

    public List<SearchNode> getSuccessors(State goalState) {
        List<SearchNode> searchNodes;
        //TODO: write successor;
        return null;
    }

    public BoxState moveBoxToPosition(Point2D position, int robotPosition) {
        MovingBox newBox = new MovingBox(position, movingBox.getWidth());
        return new BoxState(newBox, robotPosition, movingObstacles);
    }

    public BoxState moveBox(double deltaX, double deltaY, int robotPosition) {
        Point2D position = new Point2D.Double(movingBox.getPos().getX() + deltaX, movingBox.getPos().getY() + deltaY);
        return moveBoxToPosition(position, robotPosition);
    }


}
