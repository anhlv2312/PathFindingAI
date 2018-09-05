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
        heuristic = 100;
        this.state = state;
    }

    public SearchNode(State state, double cost, double heuristic ) {
        this.state = state;
        this.cost = cost;
        this.heuristic = heuristic;
    }

    @Override
    public int compareTo(SearchNode o) {
        return Double.compare(this.priority, o.priority);
    }
}

