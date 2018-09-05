package botmate;

import problem.Box;
import problem.MovingObstacle;
import problem.RobotConfig;
import java.util.ArrayList;
import java.util.List;

public class RobotState extends State {

    RobotConfig robotConfig;


    public RobotState(RobotConfig robotConfig, List<Box> movingBoxes, List<Box> movingObstacles) {
        super(robotConfig, movingBoxes, movingObstacles);
        this.robotConfig = new RobotConfig(robotConfig.getPos(), robotConfig.getOrientation());
        this.movingObstacles = new ArrayList<>();
        for (Box box: movingObstacles) {
            this.movingObstacles.add(new MovingObstacle(box.getPos(), box.getWidth()));
        }
    }

    public List<Box> getMovingObstacles() {
        return movingObstacles;
    }

}
