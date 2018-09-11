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

public class BoxAgent extends SearchAgent {

    private Point2D target;
    private int movingBoxIndex;
    private double stepWidth;

    BoxAgent(ProblemSpec ps, State initialState, int movingBoxIndex, Point2D target, double stepWidth) {
        super(ps, initialState);
        this.movingBoxIndex = movingBoxIndex;
        this.target = target;
        this.stepWidth = stepWidth;
    }

    private double calculateMovingCost(State currentState, State nextState) {
        Point2D currentPos = currentState.movingBoxes.get(movingBoxIndex).getPos();
        Point2D nextPos = nextState.movingBoxes.get(movingBoxIndex).getPos();
        return Math.abs(nextPos.getX() - currentPos.getX()) + Math.abs(nextPos.getY() - currentPos.getY());

    }

    private double calculateRobotCost(State currentState, State nextState) {

        int currentEdge = tester.isCoupled(currentState.robotConfig, currentState.movingBoxes.get(movingBoxIndex));
        int nextEdge = tester.isCoupled(nextState.robotConfig, nextState.movingBoxes.get(movingBoxIndex));

        Box movingBox = nextState.movingBoxes.get(movingBoxIndex);

        if (currentEdge == nextEdge) {
            return 0;
        } else if (Math.abs(currentEdge - nextEdge) == 2) {
            return 2 * movingBox.getWidth();
        } else {
            return movingBox.getWidth();
        }

    }

    private double calculateObstacleCost(State nextState) {
        double distance = 0;
        Box movingBox = nextState.movingBoxes.get(movingBoxIndex);
        for (Box box : nextState.movingObstacles) {
            if (movingBox.getRect().intersects(box.getRect())) {
                distance += 2 * movingBox.getWidth();
            }
        }
        return distance;
    }

    private double calculateHeuristic(State nextState) {
        Point2D nextPos = nextState.movingBoxes.get(movingBoxIndex).getPos();
        return Math.abs(nextPos.getX() - target.getX()) + Math.abs(nextPos.getY() - target.getY());
    }

    @Override
    public boolean isFound(State currentState) {
        return currentState.movingBoxes.get(movingBoxIndex).getPos().distance(target) < Tester.MAX_ERROR;
    }

    @Override
    public List<SearchNode> getSuccessors(State currentState) {

        Box movingBox =  currentState.movingBoxes.get(movingBoxIndex);

        List<State> possibleStates = new ArrayList<>();

        if (movingBox.getPos().distance(target) < stepWidth) {
            double gapX = Math.abs(movingBox.getPos().getX() - target.getX());
            double gapY = Math.abs(movingBox.getPos().getY() - target.getY());
            possibleStates.add(currentState.moveMovingBox(movingBoxIndex, gapX, 0, 2));
            possibleStates.add(currentState.moveMovingBox(movingBoxIndex, 0, gapY, 1));
            possibleStates.add(currentState.moveMovingBox(movingBoxIndex, -gapX, 0, 4));
            possibleStates.add(currentState.moveMovingBox(movingBoxIndex, 0, -gapY, 3));
        } else {
            int robotPosition = tester.isCoupled(currentState.robotConfig, movingBox);

            State moveLeft = currentState.moveMovingBox(movingBoxIndex, -stepWidth, 0, 4);
            State moveUp = currentState.moveMovingBox(movingBoxIndex, 0, stepWidth, 1);
            State moveRight = currentState.moveMovingBox(movingBoxIndex, stepWidth, 0, 2);
            State moveDown = currentState.moveMovingBox(movingBoxIndex, 0, -stepWidth, 3);

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
        }

        List<SearchNode> nodes = new ArrayList<>();
        for (State nextState: possibleStates) {
            if (checkMovingBoxCollision(nextState)) {
                double movingCost = calculateMovingCost(currentState, nextState);
                double robotCost = calculateRobotCost(currentState, nextState);
                double obstacleCost = calculateObstacleCost(nextState);
                double heuristic = calculateHeuristic(nextState);
                nodes.add(new SearchNode(nextState, movingCost + robotCost + obstacleCost, heuristic));
            }
        }

        return nodes;
    }



    private boolean checkMovingBoxCollision(State state) {

        Box movingBox = state.movingBoxes.get(movingBoxIndex);

        Rectangle2D border = new Rectangle2D.Double(Tester.MAX_ERROR,Tester.MAX_ERROR,1 - Tester.MAX_ERROR,1 - Tester.MAX_ERROR);

        Point2D bottomLeft = movingBox.getPos();
        Point2D bottomLeftCorner = new Point2D.Double(bottomLeft.getX() - Tester.MAX_ERROR,
                bottomLeft.getY() - Tester.MAX_ERROR);
        Point2D topRightCorner = new Point2D.Double(bottomLeft.getX() + movingBox.getWidth() + Tester.MAX_ERROR,
                bottomLeft.getY() + movingBox.getWidth() + Tester.MAX_ERROR);

        Line2D robotLine = new Line2D.Double(tester.getPoint1(state.robotConfig), tester.getPoint2(state.robotConfig));

        if (!border.contains(bottomLeftCorner) || !border.contains(topRightCorner)) {
            return false;
        }


        for (int i=0; i < state.movingBoxes.size(); i++) {
            if (i != movingBoxIndex) {
                Box box = state.movingBoxes.get(i);
                if (movingBox.getRect().intersects(box.getRect())) {
                    return false;
                }
                if (robotLine.intersects(tester.grow(box.getRect(), -Tester.MAX_ERROR))) {
                    return false;
                }
            }
        }

        for (StaticObstacle box: staticObstacles) {
            if (movingBox.getRect().intersects(box.getRect())) {
                return false;
            }
        }
        return true;
    }

}
