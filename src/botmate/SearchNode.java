package botmate;

public class SearchNode implements Comparable<SearchNode> {

    public double priority;
    public double totalCost;
    public double heuristic;
    public double cost;
    public State state;
    public SearchNode parent;

    public SearchNode(State state) {
        parent = null;
        totalCost = 0;
        this.state = state;
        cost = 1;
        heuristic = 1;
    }

    public SearchNode(State state, double cost, double heuristic ) {
        parent = null;
        totalCost = 0;
        this.state = state;
        this.cost = cost;
        this.heuristic = heuristic;
    }

    @Override
    public int compareTo(SearchNode o) {
        return Double.compare(this.priority, o.priority);
    }
}

