package botmate;

import problem.Box;
import problem.MovingBox;
import problem.MovingObstacle;
import problem.RobotConfig;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

public class BoxState extends State {

    MovingBox movingBox;

    public BoxState(RobotConfig robotConfig, MovingBox movingBox, List<Box> movingObstacles) {

        super(robotConfig, null, movingObstacles);
        this.movingBox = new MovingBox(movingBox.getPos(), movingBox.getWidth());
        this.robotConfig = new RobotConfig(robotConfig.getPos(), robotConfig.getOrientation());

        this.movingObstacles = new ArrayList<>();
        for (Box box: movingObstacles) {
            this.movingObstacles.add(new MovingObstacle(box.getPos(), box.getWidth()));
        }

    }


    public BoxState moveBoxToPosition(Point2D position) {
        BoxState newBoxState = new BoxState (robotConfig, movingBox, movingObstacles);
        newBoxState.movingBox.getPos().setLocation(position);
        return newBoxState;
    }

    public BoxState moveBox(double deltaX, double deltaY, int robotPosition) {
        Point2D position = new Point2D.Double(movingBox.getPos().getX() + deltaX, movingBox.getPos().getY() + deltaY);
        return moveBoxToPosition(position);
    }


}
