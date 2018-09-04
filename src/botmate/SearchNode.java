package botmate;

public class SearchNode implements Comparable<botmate.BotMateNode> {

    public double gScores;
    public double hScores;
    public double fScores;
    public State state;
    public SearchNode parent;

    public SearchNode(State state){
        this.parent = null;
        this.state = state;
    }

    public SearchNode(State state, double gScores, double hScores ){
        this.state = state;
        this.gScores = gScores;
        this.hScores = hScores;
    }

    @Override
    public int compareTo(botmate.BotMateNode o) {
        return Double.compare(this.fScores, o.fScores);
    }

}

