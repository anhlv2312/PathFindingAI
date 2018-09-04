package botmate;

import java.util.*;

public class BotMateAgent {

    static PriorityQueue<BotMateNode> container = new PriorityQueue<>();

    public static List<BotMateState> searchMovingBoxPath(BotMateState initialState, BotMateState goalState) {


        Set<String> explored = new HashSet<>();
        BotMateNode initialNode = new BotMateNode(initialState);
        initialNode.gScores = 0;

        container.add(initialNode);

        Boolean found = false;
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

            List<BotMateNode> nodes;

            nodes = currentNode.state.getSuccessors(goalState);

            for (BotMateNode node : nodes) {
                BotMateState child = node.state;
                double temp_g_scores = currentNode.gScores + node.gScores;
                double temp_f_scores = temp_g_scores + node.hScores;

                if ((explored.contains(child.outputString())) &&
                        (temp_f_scores >= node.fScores)) {
                    continue;
                }
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

    public static List<BotMateState> searchRobotPath(BotMateState initialState, BotMateState goalState) {

        Set<String> explored = new HashSet<>();
        BotMateNode initialNode = new BotMateNode(initialState);
        initialNode.gScores = 0;

        container.add(initialNode);

        while (!container.isEmpty()) {

            //the node in having the lowest f_score value
            BotMateNode currentNode = container.poll();

            explored.add(currentNode.state.outputString());

            //goal found

            if (currentNode.state.getRobotConfig().getPos().distance(goalState.getRobotConfig().getPos()) < 0.01) {
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

            List<BotMateNode> nodes;

            nodes = currentNode.state.getSuccessors();


            for (BotMateNode node : nodes) {
                BotMateState child = node.state;
                double temp_g_scores = currentNode.gScores + node.gScores;
                double temp_f_scores = temp_g_scores + node.hScores;

                if ((explored.contains(child.outputString())) &&
                        (temp_f_scores >= node.fScores)) {
                    continue;
                }
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