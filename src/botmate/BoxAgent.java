package botmate;

import problem.ProblemSpec;

public class BoxAgent extends SearchAgent {

    public BoxAgent(ProblemSpec ps, State initialState) {
        super(ps, initialState);
    }

    private boolean isFound(RobotState currentState, RobotState goalState) {
        return false;
    }

}
