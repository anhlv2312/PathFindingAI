package eightpuzzle;

import java.util.ArrayList;
import java.util.List;
import common.State;
import common.StateCostPair;

/**
 * A state of the 8-puzzle problem.
 *
 * Part of the solution code for COMP3702/7702 Tutorial 2.
 *
 * Created by Nicholas Collins on 8/08/2017.
 */
public class PuzzleState implements common.State {

    private ArrayList<Character> numbers;
    private int indexOfBlank;

    /**
     * Create an 8-puzzle state from a string.
     * @param str string representing tile positions
     */
    public PuzzleState(String str) {
        numbers = new ArrayList<Character>(9);

        // add each char to an array
        for(int i = 0; i < str.length(); i++) {
            numbers.add(str.charAt(i));
        }

        // find the position of the blank tile
        indexOfBlank = -1;
        for(int i = 0; i < numbers.size(); i++) {
            if(numbers.get(i) == '_') {
                if(indexOfBlank != -1) {
                    throw new IllegalArgumentException("Too many blank spaces.");
                }
                indexOfBlank = i;
            }
        }
        if(indexOfBlank == -1) {
            throw new IllegalArgumentException("Blank space missing.");
        }
    }

    /**
     * Create an 8-puzzle state from an array of chars.
     * @param numbers array of chars representing tile positions
     */
    public PuzzleState(List<Character> numbers) {
        this.numbers = new ArrayList<Character>(numbers);

        // find the position of the blank tile
        indexOfBlank = -1;
        for(int i = 0; i < numbers.size(); i++) {
            if(numbers.get(i) == '_') {
                if(indexOfBlank != -1) {
                    throw new IllegalArgumentException("Too many blank spaces.");
                }
                indexOfBlank = i;
            }
        }
        if(indexOfBlank == -1) {
            throw new IllegalArgumentException("Blank space missing.");
        }
    }

    /**
     * Create an 8-puzzle state from an array of chars with blank position as
     * specified.
     * @param numbers array of chars representing tile positions
     * @param indexOfBlank index in the array of the blank tile
     */
    public PuzzleState(List<Character> numbers, int indexOfBlank) {
        this.numbers = new ArrayList<Character>(numbers);
        this.indexOfBlank = indexOfBlank;
    }

    /**
     * Output a string representation of this puzzle state.
     * @return string representation
     */
    public String outputString() {
        StringBuilder sb = new StringBuilder(9);

        for(int i = 0; i < numbers.size(); i++) {
            sb.append(numbers.get(i));
        }

        return sb.toString();
    }

    /**
     * Return the number at position i (counting from 0, left to right then
     * top to bottom).
     * @param i index
     * @return number at index i
     */
    public char getNumberAt(int i) {
        return numbers.get(i);
    }

    /**
     * Return a list of all states reachable from this state. Each successor
     * has a cost of 1 (indicating it is reached in 1 move).
     * @return list of successors
     */
    public List<StateCostPair> getSuccessors() {
        List<StateCostPair> successors = new ArrayList<StateCostPair>(4);

        if((indexOfBlank % 3) != 0) {
            // blank is not in left column - left move is valid
            successors.add(new StateCostPair(swapPositions(indexOfBlank - 1), 1));
        }

        if((indexOfBlank % 3) != 2) {
            // blank is not in right column - right move is valid
            successors.add(new StateCostPair(swapPositions(indexOfBlank + 1), 1));
        }

        if((indexOfBlank / 3) != 0) {
            // blank is not in top row - up move is valid
            successors.add(new StateCostPair(swapPositions(indexOfBlank - 3), 1));
        }

        if((indexOfBlank / 3) != 2) {
            // blank is not in bottom row - down move is valid
            successors.add(new StateCostPair(swapPositions(indexOfBlank + 3), 1));
        }

        return successors;
    }

    /**
     * Clone the puzzle state and swap the blank tile with the tile at the
     * given index.
     * @param index position to swap blank with
     * @return puzzle state after swap
     */
    private PuzzleState swapPositions(int index) {
        List<Character> temp = new ArrayList<Character>(numbers);
        temp.set(indexOfBlank, numbers.get(index));
        temp.set(index, '_');
        return new PuzzleState(temp, index);
    }

    /**
     * Check if this puzzle state is equivalent to state s
     * @param s state to check equality with
     * @return true if all tiles are in the same position
     */
    public boolean equals(State s) {
        if(!(s instanceof PuzzleState)) {
            return false;
        }
        PuzzleState p = (PuzzleState) s;
        boolean match = true;
        for(int i = 0; i < numbers.size(); i++) {
            if(numbers.get(i) != p.getNumberAt(i)) {
                match = false;
            }
        }
        return match;
    }

    /**
     * Compute the parity of this state. Refer to the parity slides posted on the course website.
     * @return  0 if puzzle state has even parity
     *          1 if puzzle state has odd parity
     */
    public int parity() {
        int total = 0;

        for(int i = 0; i < numbers.size(); i++) {
            if(numbers.get(i) == '_') {
                continue;
            }
            for(int j = i+1; j < numbers.size(); j++) {
                if(numbers.get(i) > numbers.get(j)) {
                    total++;
                }
            }
        }
        return total % 2;
    }
}
