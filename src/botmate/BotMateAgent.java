package botmate;

import common.SearchTreeNode;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.Queue;

public class BotMateAgent  {

    private Queue<SearchTreeNode> container;
    private HashSet<String> visited;
    private int totNodes = 0;

    /**
     * Create a search agent instance.
     */
    //todo: Implement what ever method we want
    public BotMateAgent() {
        container = new LinkedList<SearchTreeNode>();
        visited = new HashSet<String>();
    }
}
