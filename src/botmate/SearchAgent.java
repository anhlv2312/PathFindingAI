package botmate;

import problem.ProblemSpec;
import problem.StaticObstacle;
import tester.Tester;
import java.util.*;

public abstract class SearchAgent {

    PriorityQueue<SearchNode> container;
    List<StaticObstacle> staticObstacles;
    Tester tester;
    State initialState;
    double robotWidth;

    public SearchAgent(ProblemSpec ps, State initialState) {
        tester = new Tester(ps);
        robotWidth = ps.getRobotWidth();
        staticObstacles = ps.getStaticObstacles();
        container = new PriorityQueue<>();
        this.initialState = initialState;
    }

    public abstract boolean isFound(State currentState);
    public abstract List<SearchNode>  getSuccessors(State currentState);

    public List<State> search() {

        Set<String> visited = new HashSet<>();
        SearchNode initialNode = new SearchNode(initialState);

        container.clear();
        container.add(initialNode);

        while (!container.isEmpty()) {

            SearchNode currentNode = container.poll();
            State currentState = currentNode.state;
            visited.add(currentState.toString());

            if (isFound(currentState)) {
                return getPathToGoal(currentNode);
            }

            List<SearchNode> nextNodes = getSuccessors(currentState);
            for (SearchNode nextNode : nextNodes) {
                State nextState = nextNode.state;

                double nextTotalCost = currentNode.totalCost + nextNode.cost;
                double nextPriority = nextTotalCost + nextNode.heuristic;

                if (visited.contains(nextState.toString()) && nextPriority >= nextNode.totalCost) {
                    continue;
                } else if (!container.contains(nextState.toString()) || nextPriority < nextNode.totalCost) {
                    nextNode.parent = currentNode;
                    nextNode.totalCost = nextTotalCost;
                    nextNode.priority = nextPriority;
                    if (container.contains(nextState.toString())) {
                        container.remove(nextState.toString());
                    }
                    container.add(nextNode);
                }
            }
        }
        return null;
    }

    private List<State> getPathToGoal (SearchNode currentNode) {
        List<State> pathToGoal = new LinkedList<>();

        while (currentNode.parent != null) {
            pathToGoal.add(currentNode.state);
            currentNode = currentNode.parent;
        }

        Collections.reverse(pathToGoal);
        return pathToGoal;
    }
}
