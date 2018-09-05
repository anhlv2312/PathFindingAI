package botmate;

public class SearchNode implements Comparable<SearchNode> {

    public double gCost;
    public double hCost;
    public double fCost;
    public State state;
    public SearchNode parent;

    public SearchNode(State state) {
        parent = null;
        fCost = 0;
        gCost = 1;
        fCost = 1;
        this.state = state;
    }

    public SearchNode(State state, double gCost, double hCost ) {
        fCost = 0;
        this.state = state;
        this.gCost = gCost;
        this.hCost = hCost;
    }

    @Override
    public int compareTo(SearchNode o) {
        return Double.compare(this.gCost + hCost, o.gCost + hCost);
    }
}

