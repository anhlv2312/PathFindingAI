package botmate;

import problem.ProblemSpec;
import problem.StaticObstacle;
import tester.Tester;
import java.util.*;

public abstract class SearchAgent {

    PriorityQueue<SearchNode> container;
    Tester tester;
    double robotWidth;
    State initialState;
    List<StaticObstacle> staticObstacles;

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

        container.add(initialNode);

        while (!container.isEmpty()) {

            //the node in having the lowest f_score value
            SearchNode currentNode = container.poll();

            State currentState = currentNode.state;

            visited.add(currentState.toString());
            //goal found

            if (isFound(currentState)) {
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

            List<SearchNode> nodes = getSuccessors(currentState);

            for (SearchNode node : nodes) {


                double newTotalCost = currentNode.totalCost + node.cost;
                double priority = newTotalCost + node.heuristic;

                if ((visited.contains(node.state.toString())) &&
                        (priority >= node.totalCost)) {
                    continue;
                }

                else if ((!container.contains(node.state.toString())) ||
                        (priority < node.totalCost)) {
                    node.parent = currentNode;
                    node.totalCost = newTotalCost;
                    node.priority = priority;
                    if (container.contains(node.state.toString())) {
                        container.remove(node.state.toString());
                    }
                    container.add(node);
                }

            }

        }
        return null;

    }




}
