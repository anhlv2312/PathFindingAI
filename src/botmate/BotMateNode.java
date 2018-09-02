package botmate;

public class BotMateNode implements Comparable<BotMateNode> {

    public double cost;
    public double gScores;
    public double hScores;
    public double fScores;
    public BotMateState state;
    public BotMateNode parent;

    public BotMateNode(BotMateState state){
        this.parent = null;
        this.state = state;
    }


    public BotMateNode(BotMateNode parent, BotMateState state, double gScores, double hScores ){
        this.parent = parent;
        this.state = state;
        this.gScores = gScores;
        this.hScores = hScores;
    }


    @Override
    public int compareTo(BotMateNode o) {
        return Double.compare(this.gScores, o.gScores);
    }
}
