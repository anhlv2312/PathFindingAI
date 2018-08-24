package eightpuzzle;

import common.*;

import java.util.List;

/**
 * Main class for 8-Puzzle solver (Tutorial 2 Q1-3).
 *
 * Part of the solution code for COMP3702/7702 Tutorial 2.
 *
 * Created by Nicholas Collins on 8/08/2017.
 * Updated by Sergiy Dudnikov on 5/08/2018
 */
public class PuzzleSolver {

    /**
     * Main method - solve the 8 puzzle problems from tutorial 2 using BFS and
     * DFS.
     *
     * @param args not used
     */
    public static void main(String args[]) {

        boolean showSteps = false;
        boolean showNumberOfSteps = true;
        boolean showElapsedTime = true;
        boolean showTotNodes = true;


        System.out.println("\n### Puzzle 1 ###");
        System.out.println("# BFS:");
        solveEightPuzzle("BFS", "1348627_5", "1238_4765", showSteps, showNumberOfSteps, showElapsedTime, showTotNodes);
        System.out.println("# DFS:");
        solveEightPuzzle("DFS", "1348627_5", "1238_4765", showSteps, showNumberOfSteps, showElapsedTime, showTotNodes);

        System.out.println("\n### Puzzle 2 ###");
        System.out.println("# BFS:");
        solveEightPuzzle("BFS", "281_43765", "1238_4765", showSteps, showNumberOfSteps, showElapsedTime, showTotNodes);
        System.out.println("# DFS:");
        solveEightPuzzle("DFS", "281_43765", "1238_4765", showSteps, showNumberOfSteps, showElapsedTime, showTotNodes);

        System.out.println("\n### Puzzle 3 ###");
        System.out.println("# BFS:");
        solveEightPuzzle("BFS", "281463_75", "1238_4765", showSteps, showNumberOfSteps, showElapsedTime, showTotNodes);
        System.out.println("# DFS:");
        solveEightPuzzle("DFS", "281463_75", "1238_4765", showSteps, showNumberOfSteps, showElapsedTime, showTotNodes);

    }

    /**
     * Solve the given 8 puzzle using the given search strategy.
     * @param searchType search strategy to use - BFS or DFS
     * @param initStr initial 8 puzzle state
     * @param goalStr goal 8 puzzle state
     * @param showSteps set true to display the list of intermediate states
     * @param showNumSteps set true to display the number of steps
     * @param showElapsedTime set true to display the elapsed time
     */
    private static void solveEightPuzzle(String searchType, String initStr, String goalStr,
                                         boolean showSteps, boolean showNumSteps, boolean showElapsedTime, boolean showTotNodes) {
        SearchAgent agent;
        if(searchType.equals("BFS")) {
            agent = new BFS();
        } else if(searchType.equals("DFS")) {
            agent = new DFS();
        } else {
            throw new IllegalArgumentException("Invalid search type.");
        }

        State initial = new PuzzleState(initStr);
        State goal = new PuzzleState(goalStr);

        if(!parityMatch(initial, goal)) {
            System.out.println("No Solution - Parity Mismatch");
            return;
        }

        // measure time taken to find solution
        long startTime = System.nanoTime();

        List<StateCostPair> pathToGoal = agent.search(initial, goal);

        double elapsedTime = (System.nanoTime() - startTime) / 1000000.0;

        if(showSteps) {
            // print path to goal
            System.out.println("Solution:");
            System.out.println("    " + initial.outputString());
            for (int i = 0; i < pathToGoal.size(); i++) {
                System.out.println("    " + pathToGoal.get(i).state.outputString());
            }
        }

        if(showNumSteps) {
            System.out.println("No. of steps:");
            System.out.println("    " + pathToGoal.size());
        }

        if(showTotNodes) {
            System.out.println("Tot Nodes:");
            System.out.println("    " + agent.totNodes());
        }

        if(showElapsedTime) {
            // print elapsed time
            System.out.println("Time taken:");
            System.out.println("    " + elapsedTime + "ms");
        }
    }

    /**
     * Determine if states s1 and s2 have the same parity (indicating s2 is
     * reachable from s1)
     * @param s1 an 8 puzzle state
     * @param s2 a different 8 puzzle state
     * @return true if it is possible to reach s2 from s1
     */
    private static boolean parityMatch(State s1, State s2) {
        PuzzleState p1 = (PuzzleState)s1;
        PuzzleState p2 = (PuzzleState)s2;
        if(p1.parity() == p2.parity()) {
            return true;
        } else {
            return false;
        }
    }
}
