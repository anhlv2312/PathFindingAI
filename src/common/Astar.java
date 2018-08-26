package common;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;

/**
 * A generic implementation of Uniform Cost Search.
 *
 * Part of the solution code for COMP3702/7702 Tutorial 2.
 *
 * Created by Nicholas Collins on 8/08/2017.
 * Updated by Sergiy Dudnikov on 5/08/2018
 * Updated by Yukai Qiao on 12/8/2018
  */
public class Astar implements SearchAgent {

    private PriorityQueue<SearchTreeNode> container;
    private int totNodes = 0;

    /**
     * Create a UCS search agent instance.
     */
    public Astar() {
        container = new PriorityQueue<SearchTreeNode>();
    }

    /**
     * Search for a path between a given initial state and goal state.
     * @param initial the initial stage
     * @param goal the goal state
     * @return the list of states and costs representing the path from the
     * initial state to the goal state found by the UCS agent.
     */
    public List<StateCostPair> search(State initial, State goal) {
        container.add(new SearchTreeNode(new StateCostPair(initial, 0)));

        while(container.size() > 0) {
            // select the search tree node with lowest total path cost
            SearchTreeNode currentNode = container.poll();
            totNodes--;
            State currentState = currentNode.stateCostPair.state;

            // check if this state is the goal
            if (currentState.equals(goal)) {
                // goal found - return all steps from initial to goal
                List<StateCostPair> pathToGoal = new LinkedList<StateCostPair>();

                while (currentNode.parent != null) {
                    pathToGoal.add(currentNode.stateCostPair);
                    currentNode = currentNode.parent;
                }
                Collections.reverse(pathToGoal);

                // reset for next search
                reset();

                return pathToGoal;

            }

            // not the goal - add all successors to container
            List<StateCostPair> successors = currentState.getSuccessors(goal);
            for (StateCostPair s : successors) {
                totNodes++;
                SearchTreeNode tempNode = currentNode;
                boolean visited = false;
                while(tempNode.parent != null) {
                    if(tempNode.stateCostPair.state.equals(s.state)) {
                        // this state has been visited on this path
                        visited = true;
                    }
                    tempNode = tempNode.parent;
                }
                if(initial.equals(s.state)) {
                    // this state is the initial state
                    visited = true;
                }

                if(!visited) {
                    container.add(new SearchTreeNode(currentNode, s));
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
    }

    public int totNodes() {return totNodes;}

}
