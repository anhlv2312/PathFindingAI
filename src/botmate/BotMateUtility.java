package botmate;

import problem.Box;

import java.awt.geom.Point2D;

public class BotMateUtility {

    /**
     * Manhattan Distance between two points (x1, y1) and (x2, y2) is:
     * |x1 – x2| + |y1 – y2|
     */
    public double calculateDistance(Point2D initial, Point2D goal) {
        return (Math.abs(initial.getX() - goal.getX()) + Math.abs(initial.getY() - goal.getY()));
    }

    public Box createBoundaryBox(Box box, double margin) {
        return null;
    }

    public Box createCoverBox(Box box1, Box Box2) {
        return null;
    }

    public boolean checkCollisionBetweenBox(Box box1, Box Box2) {
        return false;
    }
}
