package botmate;

import problem.Box;
import problem.ProblemSpec;
import problem.StaticObstacle;
import tester.Tester;

import java.awt.geom.Line2D;
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
        List<State> possibleStates = new ArrayList<>();
        int robotPosition = tester.isCoupled(currentState.robotConfig, movingObstacle);

        State moveLeft = currentState.moveObstacle(movingObstacleIndex, -stepWidth, 0, 4);
        State moveUp = currentState.moveObstacle(movingObstacleIndex, 0, stepWidth, 1);
        State moveRight = currentState.moveObstacle(movingObstacleIndex, stepWidth, 0, 2);
        State moveDown = currentState.moveObstacle(movingObstacleIndex, 0, -stepWidth, 3);

        switch (robotPosition) {
            case 1:
                possibleStates.add(moveLeft);
                possibleStates.add(moveUp);
                possibleStates.add(moveRight);
                break;
            case 2:
                possibleStates.add(moveUp);
                possibleStates.add(moveRight);
                possibleStates.add(moveDown);
                break;
            case 3:
                possibleStates.add(moveRight);
                possibleStates.add(moveDown);
                possibleStates.add(moveLeft);
                break;
            case 4:
                possibleStates.add(moveDown);
                possibleStates.add(moveLeft);
                possibleStates.add(moveUp);
                break;
            default:
                possibleStates.add(moveLeft);
                possibleStates.add(moveUp);
                possibleStates.add(moveRight);
                possibleStates.add(moveDown);
        }

        List<SearchNode> nodes = new ArrayList<>();
        for (State nextState : possibleStates) {
            if (checkMovingObstacleCollision(nextState, movingObstacleIndex)) {
                nodes.add(new SearchNode(nextState));
            }
        }
        return nodes;
    }


    public boolean checkMovingObstacleCollision(State state, int movingObstacleIndex) {

        Box movingBox = state.movingObstacles.get(movingObstacleIndex);
        Rectangle2D border = new Rectangle2D.Double(tester.MAX_BASE_STEP,tester.MAX_BASE_STEP,1 - tester.MAX_BASE_STEP,1 - tester.MAX_BASE_STEP);

        Point2D bottomLeft = movingBox.getPos();
        Point2D topRight = new Point2D.Double(bottomLeft.getX() + movingBox.getWidth(),
                bottomLeft.getY() + movingBox.getWidth());

        Line2D robotLine = new Line2D.Double(tester.getPoint1(state.robotConfig), tester.getPoint2(state.robotConfig));

        if (!border.contains(bottomLeft) || !border.contains(topRight)) {
            return false;
        }

        for (int i=0; i < state.movingObstacles.size(); i++) {
            if (i != movingObstacleIndex) {
                Box box = state.movingObstacles.get(i);
                if (movingBox.getRect().intersects(tester.grow(box.getRect(), Tester.MAX_BASE_STEP))) {
                    return false;
                }
                if (robotLine.intersects(box.getRect())) {
                    return false;
                }
            }

        }

        for (Box box : state.movingBoxes) {
            if (movingBox.getRect().intersects(tester.grow(box.getRect(), Tester.MAX_BASE_STEP))) {
                return false;
            }
            if (robotLine.intersects(box.getRect())) {
                return false;
            }
        }

        for (StaticObstacle box: staticObstacles) {
            if (movingBox.getRect().intersects(tester.grow(box.getRect(), Tester.MAX_BASE_STEP))) {
                return false;
            }
            if (robotLine.intersects(box.getRect())) {
                return false;
            }
        }
        return true;
    }

}
