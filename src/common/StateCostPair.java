package common;

/**
 * A class storing a state and a cost to travel to it.
 *
 * Part of the solution code for COMP3702/7702 Tutorial 2.
 *
 * Created by Nicholas Collins on 8/08/2017.
 */
public class StateCostPair implements Comparable<StateCostPair> {

    public State state;
    public double cost;

    /**
     * Construct a state cost pair
     * @param state an agent problem state
     * @param cost cost to travel to the state
     */
    public StateCostPair(State state, double cost) {
        this.state = state;
        this.cost = cost;
    }

    /**
     * Required to allow StateCostPair to be used in a priority queue. Refer to
     * comment in SearchTreeNode.java
     * @param s StateCostPair to compare to
     * @return -1 if this node has a lower travel cost than pair s
     *          0 if this node has the same travel cost as pair s
     *          1 if this node has a greater travel cost than pair s
     */
    public int compareTo(StateCostPair s) {
        return Double.compare(this.cost, s.cost);
    }
}
