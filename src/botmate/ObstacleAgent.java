package botmate;

import problem.Box;
import problem.ProblemSpec;
import problem.StaticObstacle;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

public class ObstacleAgent extends SearchAgent {

    private int movingObstacleIndex;
    private double stepWidth;
    private List<Rectangle2D> movingPaths;

    public ObstacleAgent(ProblemSpec ps, State initialState, int movingObstacleIndex, double stepWidth, List<Rectangle2D> movingPaths) {

        super(ps, initialState);
        this.movingObstacleIndex = movingObstacleIndex;
        this.stepWidth = stepWidth;
        this.movingPaths = movingPaths;
    }

    @Override
    public boolean isFound(State currentState) {
        Box movingBox = currentState.movingObstacles.get(movingObstacleIndex);
        for (Rectangle2D rectangle: movingPaths) {
            if (rectangle.intersects(movingBox.getRect())){
                return false;
            }
        }
        return true;
    }

    @Override
    public List<SearchNode> getSuccessors(State currentState) {

        Box movingObstacle =  currentState.movingObstacles.get(movingObstacleIndex);
        List<State> states = new ArrayList<>();
        int robotPosition = tester.isCoupled(currentState.robotConfig, movingObstacle);

        State moveLeft = currentState.moveObstacle(movingObstacleIndex, -stepWidth, 0, 4);
        State moveUp = currentState.moveObstacle(movingObstacleIndex, 0, stepWidth, 1);
        State moveRight = currentState.moveObstacle(movingObstacleIndex, stepWidth, 0, 2);
        State moveDown = currentState.moveObstacle(movingObstacleIndex, 0, -stepWidth, 3);

        switch (robotPosition) {
            case 1:
                states.add(moveLeft);
                states.add(moveUp);
                states.add(moveRight);
                break;
            case 2:
                states.add(moveUp);
                states.add(moveRight);
                states.add(moveDown);
                break;
            case 3:
                states.add(moveRight);
                states.add(moveDown);
                states.add(moveLeft);
                break;
            case 4:
                states.add(moveDown);
                states.add(moveLeft);
                states.add(moveUp);
                break;
            default:
                states.add(moveLeft);
                states.add(moveUp);
                states.add(moveRight);
                states.add(moveDown);
        }

        List<SearchNode> nodes = new ArrayList<>();
        for (State state : states) {
            if (checkMovingObstacleCollision(state, movingObstacleIndex)) {
                nodes.add(new SearchNode(state));
            }
        }
        return nodes;
    }


    public boolean checkMovingObstacleCollision(State state, int movingObstacleIndex) {

        Box movingBox = state.movingObstacles.get(movingObstacleIndex);
        Rectangle2D border = new Rectangle2D.Double(0,0,1,1);

        Point2D bottomLeft = movingBox.getPos();
        Point2D topRight = new Point2D.Double(bottomLeft.getX() + movingBox.getWidth(),
                bottomLeft.getY() + movingBox.getWidth());

        if (!border.contains(bottomLeft) || !border.contains(topRight)) {
            return false;
        }

        for (int i=0; i < state.movingObstacles.size(); i++) {
            if (i != movingObstacleIndex) {
                Box box = state.movingObstacles.get(i);
                if (movingBox.getRect().intersects(box.getRect())) {
                    return false;
                }
            }
        }

        for (Box box : state.movingBoxes) {
            if (movingBox.getRect().intersects(box.getRect())) {
                return false;
            }
        }

        for (StaticObstacle obstacle: staticObstacles) {
            if (movingBox.getRect().intersects(obstacle.getRect())) {
                return false;
            }
        }
        return true;
    }

}
