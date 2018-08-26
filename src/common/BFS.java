package common;

import java.util.*;

/**
 * A generic implementation of Breadth First Search.
 *
 * Part of the solution code for COMP3702/7702 Tutorial 2.
 *
 * Created by Nicholas Collins on 8/08/2017.
 * Updated by Sergiy Dudnikov on 5/08/2018
 */
public class BFS implements SearchAgent {

    private Queue<SearchTreeNode> container;
    private HashSet<String> visited;
    private int totNodes = 0;
    private int temp = 0;

    /**
     * Create a BFS search agent instance.
     */
    public BFS() {
        container = new LinkedList<SearchTreeNode>();
        visited = new HashSet<String>();
    }

    /**
     * Search for a path between a given initial state and goal state.
     * @param initial the initial stage
     * @param goal the goal state
     * @return the list of states and costs representing the path from the
     * initial state to the goal state found by the BFS agent.
     */
    public List<StateCostPair> search(State initial, State goal) {

        container.add(new SearchTreeNode(new StateCostPair(initial, 0)));

        while(container.size() > 0) {
            // select a search tree node from the container
            SearchTreeNode currentNode = container.remove();
            temp--;
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
            List<StateCostPair> successors = currentState.getSuccessors();
            for(StateCostPair s : successors) {
                if(!visited.contains(s.state.outputString())) {
                    container.add(new SearchTreeNode(currentNode, s));
                    temp++;
                    if (temp > totNodes) {
                        totNodes = temp;
                    }
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
