package botmate;

import problem.Box;
import problem.MovingObstacle;
import problem.RobotConfig;

import java.util.ArrayList;
import java.util.List;

public class State {

    RobotConfig robotConfig;
    List<Box> movingBoxes;
    List<Box> movingObstacles;

    public State(RobotConfig robotConfig, List<Box> movingBoxes, List<Box> movingObstacles) {

        this.robotConfig = robotConfig;
        this.movingBoxes = new ArrayList<>();

        if (movingBoxes != null) {
            for (Box box : movingBoxes) {
                this.movingBoxes.add(new MovingObstacle(box.getPos(), box.getWidth()));
            }
        }

        this.movingObstacles = new ArrayList<>();
        for (Box box: movingObstacles) {
            this.movingObstacles.add(new MovingObstacle(box.getPos(), box.getWidth()));
        }

    }


    List<Box> getMovingBoxes() {
        return movingBoxes;
    }

    List<Box> getMovingObstacles() {
        return movingObstacles;
    }

    public String toString() {

        StringBuilder output = new StringBuilder();

        output.append(String.format("%.5f ", robotConfig.getPos().getX()));
        output.append(String.format("%.5f ", robotConfig.getPos().getY()));
        output.append(String.format("%.5f ", robotConfig.getOrientation()));

        for (Box box: movingObstacles) {
            output.append(String.format("%.5f ", box.getPos().getX() + box.getWidth()/2));
            output.append(String.format("%.5f ", box.getPos().getY() + box.getWidth()/2));
        }

        return output.toString();
    }
}
