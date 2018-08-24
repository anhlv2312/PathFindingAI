package common;

import java.util.*;

/**
 *
 * A generic implementation of Depth First Search.
 *
 * Part of the solution code for COMP3702/7702 Tutorial 2.
 *
 * Created by Nicholas Collins on 8/08/2017.
 * Updated by Sergiy Dudnikov on 5/08/2018
 */
public class DFS implements SearchAgent {

    private Stack<SearchTreeNode> container;
    private HashSet<String> visited;
    private int totNodes;
    private int maxDepth = 100000000;

    /**
     * Create a DFS search agent instance.
     */
    public DFS() {
        container = new Stack<SearchTreeNode>();
        visited = new HashSet<String>();
    }

    public List<StateCostPair> search(State initial, State goal) {

        container.add(new SearchTreeNode(new StateCostPair(initial, 0)));

        while(container.size() > 0) {
            // select a tree node from the container
            SearchTreeNode currentNode = container.pop();
            totNodes--;
            State currentState = currentNode.stateCostPair.state;

            // mark this state as visited
            visited.add(currentState.outputString());

            // check if this state is the goal
            if(currentState.equals(goal)) {
                // goal found - return all steps from initial to goal
                List<StateCostPair> pathToGoal = new LinkedList<StateCostPair>();

                while(currentNode.parent != null) {
                    pathToGoal.add(currentNode.stateCostPair);
                    currentNode = currentNode.parent;
                }
                Collections.reverse(pathToGoal);

                // reset for next search
                reset();

                return pathToGoal;

            }


            // not the goal - add all (unvisited) successors to container

            // first check if the max depth has been reached
            if (currentNode.depth >= maxDepth) {continue;}
            List<StateCostPair> successors = currentState.getSuccessors();
            for(StateCostPair s : successors) {
                if(!visited.contains(s.state.outputString())) {

                    container.add(new SearchTreeNode(currentNode, s));
                    totNodes++;
                }
            }
        }

        // no solution
        reset();
        return null;

    }

    /**
     * Resets the search agent (clears instance variables to be ready for next
     * search request).
     */
    private void reset() {
        container.clear();
        visited.clear();
    }

    public int totNodes() {return totNodes;}

}
