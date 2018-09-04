package botmate;

import java.util.*;

public class SearchAgent {

    PriorityQueue<SearchNode> container;
    static tester.Tester tester;

    public SearchAgent(tester.Tester tester) {
        this.container = new PriorityQueue<>();
    }

    private boolean isFound(State currentState, State goalState) {
        return false;
    }

    public List<State> search(State initialState, State goalState) {

        Set<String> visited = new HashSet<>();
        SearchNode initialNode = new SearchNode(initialState);

        initialNode.gScores = 0;
        container.add(initialNode);

        while (!container.isEmpty()) {

            //the node in having the lowest f_score value
            SearchNode currentNode = container.poll();
            State currentState = currentNode.state;
            visited.add(currentState.toString());

            //goal found
            if (isFound(currentState, goalState)) {
                List<State> pathToGoal = new LinkedList<>();
                while (currentNode.parent != null) {
                    pathToGoal.add(currentNode.state);
                    currentNode = currentNode.parent;
                }
                Collections.reverse(pathToGoal);

                // reset for next search
                container.clear();

                return pathToGoal;
            }

            //check every child of current node

            List<SearchNode> nodes = currentNode.state.getSuccessors();

            for (SearchNode node : nodes) {
                State child = node.state;
                double temp_g_scores = currentNode.gScores + node.gScores;
                double temp_f_scores = temp_g_scores + node.hScores;

                if ((visited.contains(child.toString())) &&
                        (temp_f_scores >= node.fScores)) {
                    continue;
                }
                else if ((!container.contains(child)) ||
                        (temp_f_scores < node.fScores)) {
                    node.parent = currentNode;
                    node.gScores = temp_g_scores;
                    node.fScores = temp_f_scores;
                    if (container.contains(child)) {
                        container.remove(child);
                    }
                    container.add(node);
                }
            }

        }
        return null;

    }
}
