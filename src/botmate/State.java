package botmate;

import java.util.List;

public interface State {
    List<Successor> getSuccessors();
    List<Successor> getSuccessors(State goal);
}
