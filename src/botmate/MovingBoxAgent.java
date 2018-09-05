package botmate;

import problem.Box;
import problem.ProblemSpec;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

public class MovingBoxAgent extends SearchAgent {

    Box movingBox;
    Point2D target;
    int robotPosition;

    public MovingBoxAgent(ProblemSpec ps, State initialState, Box movingBox, Point2D target) {
        super(ps, initialState);
        this.movingBox = movingBox;
        this.target = target;
        robotPosition = tester.isCoupled(initialState.robotConfig, movingBox);
    }

    public double calculateCost(Point2D currentPos, Point2D nextPos) {
        double distance = Math.abs(nextPos.getX() - currentPos.getX()) + Math.abs(nextPos.getX() - currentPos.getX());
        return distance;
    }

    public double calculateHeuristic(Point2D currentPos) {
        Point2D bottomLeft = new Point2D.Double(target.getX() - movingBox.getWidth(), target.getY()- movingBox.getWidth());
        double distance = currentPos.distance(bottomLeft);
        return distance;
    }

    @Override
    public boolean isFound(State currentState) {
        return false;
    }

    @Override
    public List<SearchNode> getSuccessors(State currentState) {
        double delta = robotWidth;

        BoxState boxState = (BoxState)currentState;


        List<BoxState> states = new ArrayList<>();

        switch (robotPosition) {
            case 1:
                states.add(boxState.moveBox(-delta, 0, 4));
                states.add(boxState.moveBox(0, delta, 1));
                states.add(boxState.moveBox(delta, 0, 2));
                break;
            case 2:
                states.add(boxState.moveBox(0, delta, 1));
                states.add(boxState.moveBox(delta, 0, 2));
                states.add(boxState.moveBox(0, -delta, 3));
                break;
            case 3:
                states.add(boxState.moveBox(delta, 0, 2));
                states.add(boxState.moveBox(0, -delta, 3));
                states.add(boxState.moveBox(-delta, 0, 4));
                break;
            case 4:
                states.add(boxState.moveBox(0, -delta, 3));
                states.add(boxState.moveBox(-delta, 0, 4));
                states.add(boxState.moveBox(0, delta, 1));
                break;
        }


        List<SearchNode> nodes = new ArrayList<>();
        for (BoxState state: states) {
            if (checkMovingBoxCollision(state, state.movingBox)) {
                double cost = calculateCost(boxState.movingBox.getPos(), state.movingBox.getPos());
                double heuristic = calculateHeuristic(state.movingBox.getPos());
                nodes.add(new SearchNode(state, cost, heuristic));
            }
        }

        System.out.println("not attached yet!");

        return null;
    }


}
