package botmate;

import problem.Box;
import problem.ProblemSpec;
import problem.StaticObstacle;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

public class MovingObstacleAgent extends SearchAgent {

    int movingObstacleIndex;
    double boxWidth;
    List<Rectangle2D> movingPaths;

    public MovingObstacleAgent(ProblemSpec ps, State initialState, int movingObstacleIndex, List<Rectangle2D> movingPaths) {

        super(ps, initialState);
        this.movingObstacleIndex = movingObstacleIndex;
        boxWidth = initialState.movingObstacles.get(movingObstacleIndex).getWidth();
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

        double delta = robotWidth/2 - tester.MAX_BASE_STEP;

        List<State> states = new ArrayList<>();

//        int robotPosition = tester.isCoupled(currentState.robotConfig, currentState.movingObstacles.get(movingObstacleIndex));
//
//        switch (robotPosition) {
//            case 1:
//                states.add(currentState.moveMovingObstacle(movingObstacleIndex, -delta, 0, 4));
//                states.add(currentState.moveMovingObstacle(movingObstacleIndex, 0, delta, 1));
//                states.add(currentState.moveMovingObstacle(movingObstacleIndex, delta, 0, 2));
//                break;
//            case 2:
//                states.add(currentState.moveMovingObstacle(movingObstacleIndex, 0, delta, 1));
//                states.add(currentState.moveMovingObstacle(movingObstacleIndex, delta, 0, 2));
//                states.add(currentState.moveMovingObstacle(movingObstacleIndex, 0, -delta, 3));
//                break;
//            case 3:
//                states.add(currentState.moveMovingObstacle(movingObstacleIndex, delta, 0, 2));
//                states.add(currentState.moveMovingObstacle(movingObstacleIndex, 0, -delta, 3));
//                states.add(currentState.moveMovingObstacle(movingObstacleIndex, -delta, 0, 4));
//                break;
//            case 4:
//                states.add(currentState.moveMovingObstacle(movingObstacleIndex, 0, -delta, 3));
//                states.add(currentState.moveMovingObstacle(movingObstacleIndex, -delta, 0, 4));
//                states.add(currentState.moveMovingObstacle(movingObstacleIndex, 0, delta, 1));
//                break;
//            default:
//                System.out.println("not attached to moving obstacle yet!");
//
//
//        }


        states.add(currentState.moveMovingObstacle(movingObstacleIndex, -delta, 0, 4));
        states.add(currentState.moveMovingObstacle(movingObstacleIndex, 0, delta, 1));
        states.add(currentState.moveMovingObstacle(movingObstacleIndex, delta, 0, 2));
        states.add(currentState.moveMovingObstacle(movingObstacleIndex, 0, -delta, 3));

        List<SearchNode> nodes = new ArrayList<>();
        for (State state : states) {
            if (checkMovingObstacleCollision(state, movingObstacleIndex)) {
//                double cost = calculateCost(currentState, state);
//                double heuristic = calculateHeuristic(state);
                nodes.add(new SearchNode(state, 1, 1));
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
