package botmate;

import java.util.*;

public class BotMateAgent {

    static PriorityQueue<BotMateNode> container = new PriorityQueue<>();

    public static List<BotMateState> search(BotMateState initialState, BotMateState goalState) {

        Set<String> explored = new HashSet<>();

        BotMateNode initialNode = new BotMateNode(initialState);
        initialNode.gScores = 0;

        container.add(initialNode);


        while (!container.isEmpty()) {

            //the node in having the lowest f_score value
            BotMateNode currentNode = container.poll();

            explored.add(currentNode.state.outputString());

            //goal found
            if (currentNode.state.outputString().equals(goalState.outputString())) {

                List<BotMateState> pathToGoal = new LinkedList<>();

                while (currentNode.parent != null) {
                    pathToGoal.add(currentNode.state);
                    currentNode = currentNode.parent;
                }
                Collections.reverse(pathToGoal);

                // reset for next search
                container.clear();

                return pathToGoal;

            }

            //check every child of current node

            for (BotMateNode node : currentNode.state.getSuccessors(goalState)) {

                BotMateState child = node.state;

                double temp_g_scores = currentNode.gScores + node.cost;
                double temp_f_scores = temp_g_scores + node.hScores;


                                /*if child node has been evaluated and
                                the newer f_score is higher, skip*/

                if ((explored.contains(child)) &&
                        (temp_f_scores >= node.fScores)) {
                    continue;
                }

                                /*else if child node is not in queue or
                                newer f_score is lower*/

                else if ((!container.contains(child)) ||
                        (temp_f_scores < node.fScores)) {

                    node.parent = currentNode;
                    node.gScores = temp_g_scores;
                    node.fScores = temp_f_scores;

                    if (container.contains(child)) {
                        container.remove(child);
                    }

                    container.add(node);

                }

            }


        }
        return null;

    }
}
