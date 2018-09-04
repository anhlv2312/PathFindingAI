package botmate;

import java.util.List;

public interface State {
    List<SearchNode> getSuccessors();
    List<SearchNode> getSuccessors(State goal);
    String toString();
}
