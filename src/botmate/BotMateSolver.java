package botmate;


import common.*;
import problem.*;
import tester.Tester;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

public class BotMateSolver {

    /** Maximum step size for a primitive step*/
    public static final double MAX_BASE_STEP = 0.001;
    public static final double MAX_ERROR = 0.0001;

    private static final int ROBOT_RANDOM_SAMPLES = 50;
    private static final int BOX_RANDOM_SAMPLES = 200;

    private static Line2D[] boundaryLines = new Line2D[]{
            new Line2D.Double(-MAX_ERROR, -MAX_ERROR, -MAX_ERROR, 1 + MAX_ERROR),
            new Line2D.Double(-MAX_ERROR, 1 + MAX_ERROR, 1 + MAX_ERROR, 1 + MAX_ERROR),
            new Line2D.Double(1 + MAX_ERROR, 1 + MAX_ERROR, 1 + MAX_ERROR, -MAX_ERROR),
            new Line2D.Double(1 + MAX_ERROR, -MAX_ERROR, -MAX_ERROR, -MAX_ERROR)
    };

    static ProblemSpec ps;
    static Tester tester;

    public static void main(String args[]) {

        try {
            ps = new ProblemSpec();
            ps.loadProblem("bot.input1.txt");
            tester = new Tester(ps);
        } catch (IOException ex) {
            System.out.println("IO Exception occurred");
        }

        BotMateState initialState, currentState, goalState;
        Point2D movingBoxGoal;
        List<StateCostPair> solution = new LinkedList<>();
        SearchAgent agent = new Astar();

        initialState = new BotMateState(0,
                ps.getInitialRobotConfig(),
                ps.getMovingBoxes(),
                ps.getMovingObstacles(),
                tester);

        currentState = initialState;

        // loop all the box
        for (int i=0; i < ps.getMovingBoxes().size(); i++) {
            currentState.setMovingBoxIndex(i);
            solution.add(new StateCostPair(currentState,0));
            movingBoxGoal = ps.getMovingBoxEndPositions().get(i);
            goalState = currentState.moveMovingBox(movingBoxGoal);
            solution.addAll(agent.search(currentState, goalState));
            currentState = goalState;
        }


        List<String> output = new LinkedList<>();

        currentState = initialState;
        BotMateState state;
        for (StateCostPair stateCostPair: solution) {

            state = (BotMateState)stateCostPair.state;
            System.out.println(state.outputString());
            output.addAll(generateBoxMove(currentState, state));
            currentState = state;

        }

        try {
            FileWriter fw = new FileWriter("bot.output.txt");
            BufferedWriter bw = new BufferedWriter(fw);

            System.out.println("Number of steps: " + output.size());
            bw.write(Integer.toString(output.size()));
            bw.write("\n");
            bw.flush();
            for (String string : output) {
                bw.write(string);
                bw.write("\n");
            }
            bw.close();

        } catch (IOException ex) {
            System.out.println("IO Exception occurred");
        }

    }


    private static List<String> generateBoxMove(BotMateState state1, BotMateState state2) {


        List<String> result = new LinkedList<>();

        if (state1.getMovingBoxIndex () != state2.getMovingBoxIndex ()) {
            return result;
        }

        BotMateState temp = state1;

        Box movingBox1 = state1.getMovingBox();
        Box movingBox2 = state2.getMovingBox();

        Double numberOfSteps = Math.ceil(movingBox1.getPos().distance(movingBox2.getPos()) / 0.001);

        double deltaX = (movingBox2.getPos().getX() - movingBox1.getPos().getX()) / numberOfSteps;
        double deltaY = (movingBox2.getPos().getY() - movingBox1.getPos().getY()) / numberOfSteps;
        for (int i = 0; i < numberOfSteps; i++) {
            Point2D position = new Point2D.Double(temp.getMovingBox().getPos().getX() + deltaX, temp.getMovingBox().getPos().getY() + deltaY);
            temp = temp.moveMovingBox(position);
            result.add(temp.outputString());
        }

        return result;
    }

}
