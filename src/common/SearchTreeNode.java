package common;

/**
 * A generic search tree node representing a path through a state graph. Stores
 * a state, the cost of the last edge followed, and the total cost of all edges
 * in the path associated with this node.
 *
 * Part of the solution code for COMP3702/7702 Tutorial 2.
 *
 * Created by Nicholas Collins on 8/08/2017.
 * Updated by Sergiy Dudnikov on 5/08/2018
 */
public class SearchTreeNode implements Comparable<SearchTreeNode> {

    public SearchTreeNode parent;
    public StateCostPair stateCostPair;
    public double pathCost;
    public int depth;

    /**
     * Construct a root node of a search tree.
     * @param stateCostPair the state (and last edge cost) associated with this
     *                      search tree node.
     */
    public SearchTreeNode(StateCostPair stateCostPair) {
        this.parent = null;
        this.stateCostPair = stateCostPair;
        this.pathCost = 0;
        this.depth = 0;
    }

    /**
     * Construct a non-root search tree node.
     * @param parent the parent of this node
     * @param stateCostPair the state (and last edge cost) associated with this
     *                      search tree node.
     */
    public SearchTreeNode(SearchTreeNode parent, StateCostPair stateCostPair) {
        this.parent = parent;
        this.stateCostPair = stateCostPair;
        this.pathCost = parent.pathCost + stateCostPair.cost;
        this.depth = parent.depth + 1;
    }

    /**
     * Compares the path cost of this node to the path cost of node s.
     *
     * Defining a "compareTo" method is necessary in order for searchTreeNodes
     * to be used in a priority queue.
     *
     * @param s node to compare path cost with
     * @return  -1 if this node has a lower total path cost than node s
     *          0 if this node has the same total path cost as node s
     *          1 if this node has a greater total path cost than node s
     */
    public int compareTo(SearchTreeNode s) {
        return Double.compare(this.pathCost, s.pathCost);
    }
}
