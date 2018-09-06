package botmate;

import problem.Box;
import problem.ProblemSpec;
import problem.StaticObstacle;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

public class BoxAgent extends SearchAgent {

    Point2D target;
    int movingBoxIndex;
    double boxWidth;
    double stepWidth;
    boolean direct;

    public BoxAgent(ProblemSpec ps, State initialState, int movingBoxIndex, Point2D target, double stepWidth, boolean direct) {
        super(ps, initialState);
        this.movingBoxIndex = movingBoxIndex;
        this.target = target;
        this.stepWidth = stepWidth;
        this.direct = direct;
        boxWidth = initialState.movingBoxes.get(movingBoxIndex).getWidth();
    }

    public double calculateCost(State currentState, State nextState) {

        Point2D currentPos = currentState.movingBoxes.get(movingBoxIndex).getPos();
        Point2D nextPos = nextState.movingBoxes.get(movingBoxIndex).getPos();

        double distance = Math.abs(nextPos.getX() - currentPos.getX()) + Math.abs(nextPos.getX() - currentPos.getX());
        return distance;
    }

    public double calculateHeuristic(State nextState) {
        Point2D nextPos = nextState.movingBoxes.get(movingBoxIndex).getPos();
        double distance = Math.abs(nextPos.getX() - target.getX()) + Math.abs(nextPos.getX() - target.getX());
//        return nextPos.distance(target);
        return distance;
    }

    @Override
    public boolean isFound(State currentState) {
        return currentState.movingBoxes.get(movingBoxIndex).getPos().distance(target) < tester.MAX_BASE_STEP;
    }

    @Override
    public List<SearchNode> getSuccessors(State currentState) {

        Box movingBox =  currentState.movingBoxes.get(movingBoxIndex);

        List<State> states = new ArrayList<>();

        if (movingBox.getPos().distance(target) > stepWidth) {
            states.add(currentState.moveMovingBox(movingBoxIndex, -stepWidth, 0, 4));
            states.add(currentState.moveMovingBox(movingBoxIndex, 0, stepWidth, 1));
            states.add(currentState.moveMovingBox(movingBoxIndex, stepWidth, 0, 2));
            states.add(currentState.moveMovingBox(movingBoxIndex, 0, -stepWidth, 3));
        } else {
            double gapX = Math.abs(movingBox.getPos().getX() - target.getX());
            double gapY = Math.abs(movingBox.getPos().getY() - target.getY());
            states.add(currentState.moveMovingBox(movingBoxIndex, gapX, 0, 2));
            states.add(currentState.moveMovingBox(movingBoxIndex, 0, gapY, 1));
            states.add(currentState.moveMovingBox(movingBoxIndex, -gapX, 0, 4));
            states.add(currentState.moveMovingBox(movingBoxIndex, 0, -gapY, 3));
        }

//        int robotPosition = tester.isCoupled(currentState.robotConfig, movingBox);
//
//        switch (robotPosition) {
//            case 1:
//                states.add(currentState.moveMovingBox(movingBoxIndex, -delta, 0, 4));
//                states.add(currentState.moveMovingBox(movingBoxIndex, 0, delta, 1));
//                states.add(currentState.moveMovingBox(movingBoxIndex, delta, 0, 2));
//                break;
//            case 2:
//                states.add(currentState.moveMovingBox(movingBoxIndex, 0, delta, 1));
//                states.add(currentState.moveMovingBox(movingBoxIndex, delta, 0, 2));
//                states.add(currentState.moveMovingBox(movingBoxIndex, 0, -delta, 3));
//                break;
//            case 3:
//                states.add(currentState.moveMovingBox(movingBoxIndex, delta, 0, 2));
//                states.add(currentState.moveMovingBox(movingBoxIndex, 0, -delta, 3));
//                states.add(currentState.moveMovingBox(movingBoxIndex, -delta, 0, 4));
//                break;
//            case 4:
//                states.add(currentState.moveMovingBox(movingBoxIndex, 0, -delta, 3));
//                states.add(currentState.moveMovingBox(movingBoxIndex, -delta, 0, 4));
//                states.add(currentState.moveMovingBox(movingBoxIndex, 0, delta, 1));
//                break;
//            default:
//                System.out.println("not attached yet!");
//
//
//        }




        List<SearchNode> nodes = new ArrayList<>();
        for (State state: states) {
            if (checkMovingBoxCollision(state, movingBoxIndex)) {
                double cost = calculateCost(currentState, state);
                double heuristic = calculateHeuristic(state);
                nodes.add(new SearchNode(state, cost, heuristic));
            }
        }



        return nodes;
    }


    public boolean checkMovingBoxCollision(State state, int movingBoxIndex) {

        Box movingBox = state.movingBoxes.get(movingBoxIndex);

        Rectangle2D border = new Rectangle2D.Double(0,0,1,1);

        Point2D bottomLeft = movingBox.getPos();
        Point2D topRight = new Point2D.Double(bottomLeft.getX() + movingBox.getWidth(),
                bottomLeft.getY() + movingBox.getWidth());

        if (!border.contains(bottomLeft) || !border.contains(topRight)) {
            return false;
        }

        for (int i=0; i < state.movingBoxes.size(); i++) {
            if (i != movingBoxIndex) {
                Box box = state.movingBoxes.get(i);
                if (movingBox.getRect().intersects(box.getRect())) {
                    return false;
                }
            }
        }

        if (direct) {
            for (Box box : state.movingObstacles) {
                if (movingBox.getRect().intersects(box.getRect())) {
                    return false;
                }
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
