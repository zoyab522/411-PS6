package edu.ncsu.csc411.ps06.agent;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;
import java.util.PriorityQueue;

import edu.ncsu.csc411.ps06.environment.Action;
import edu.ncsu.csc411.ps06.environment.Environment;
import edu.ncsu.csc411.ps06.environment.Position;
import edu.ncsu.csc411.ps06.environment.TileStatus;

/**
 * 
 * @author zoyab
 *
 */
public class Robot {
	private Environment env;

	/** Initializes a Robot on a specific tile in the environment.
	 * @param env - The Environment
	 */
	public Robot (Environment env) { this.env = env; }

	/**	
	 ****************************************DOCSTRING HERE****************************************
	 * This uses a planning algorithm using priority queues to
	 * prioritize subgoals encountered during Chip's Challenge and
	 * an A* search algorithm to find an optimal path to the
	 * end goal. 
	 *
	 * First, this method gets the locations of all of the objects
	 * in the environment that the agent will encounter. This includes
	 * chips, doors, keys, the DOOR_GOAL, and the final GOAL. The priority queue
	 * uses a custom comparator which prioritizes the subgoals in the queue
	 * based on their calculated proximity to the agent. However, these priorities
	 * are still altered based on where the agent is on the map. For example,
	 * if the agent is closest to the DOOR_GOAL, but there are still chips left
	 * to collect on the map, then the agent will prioritize collecting those chips
	 * rather than the DOOR_GOAL as it cannot move past the DOOR_GOAL without collecting
	 * all the chips. 
	 * 
	 * The agent's inventory is also tracked along the way to keep track of and
	 * update which keys it has. This is so that, when a locked door is 
	 * encountered, it can check if it has the key for the corresponding door. 
	 * A helper method called getKeyForDoor is used to determine which key 
	 * goes to which door. It iterates through all of the subgoals, while the queue of 
	 * subgoals is not empty, to determine which subgoal (Position) to target next. 
	 * The first subgoal that is dequeued has the highest priority, but if this subgoal
	 * brings the agent to a locked door for which it has no corresponding key,
	 * then the agent is able to skip that subgoal and move to the next subgoal. This is
	 * where the helper method getKeyForDoor is used, to check if the agent has the corresponding
	 * key of color for the locked door it may be at. 
	 * 
	 * Essentially, with each iteration of the loop in which it is checking
	 * subgoals in the queue, the method will POLL the next subgoal from the queue. 
	 * It uses the A* search algorithm to determine an OPTIMAL path to the current subgoal. 
	 * If a path is found towards it, then the agent's position is updated (it takes an Action) and
	 * it moves towards the subgoal. 
	 * 
	 * If it goes through the whole queue of subgoals without finding an Action (e.g., there is 
	 * no valid path towards the current subgoal), then the method will return Action.DO_NOTHING and the
	 * agent doesn't move. 
	 *
	 * The method called by Environment to retrieve the agent's actions.        
	 * @return should return a single Action from the Action class.
    	- Action.DO_NOTHING
    	- Action.MOVE_UP
    	- Action.MOVE_DOWN
    	- Action.MOVE_LEFT
    	- Action.MOVE_RIGHT
	 */
	public Action getAction() {		

		// Get a Map of all of the locations of the objects in the environment
		Map<TileStatus, ArrayList<Position>> envPositions = env.getEnvironmentPositions();

		// Get an ArrayList of all the locations of all of the chips in the environment
		ArrayList<Position> chips = envPositions.get(TileStatus.CHIP);

		// Get an ArrayList of all the locations of all the doors in the environment
		ArrayList<Position> doors = new ArrayList<>();

		// There are 4 different types of doors in the environment the agent can open
		// Add all 4 types of doors (their locations) to the ArrayList
		doors.addAll(envPositions.get(TileStatus.DOOR_GREEN));
		doors.addAll(envPositions.get(TileStatus.DOOR_BLUE));
		doors.addAll(envPositions.get(TileStatus.DOOR_YELLOW));
		doors.addAll(envPositions.get(TileStatus.DOOR_RED));

		// Get the position of the DOOR_GOAL, which the agent must reach before reaching the GOAL
		ArrayList<Position> doorGoalPositions = envPositions.get(TileStatus.DOOR_GOAL);
		// Make sure that the list isn't empty so it doesn't throw an IndexOutOfBoundsError
		if(!doorGoalPositions.isEmpty()) {
			Position doorGoal = doorGoalPositions.get(0);
			doors.add(doorGoal); // Once the check is complete, add its position to the list of other doors
		}

		// Get the position of the GOAL on the map
		Position goal = envPositions.get(TileStatus.GOAL).get(0);

		// Get the list of the positions of all the keys on the map
		ArrayList<Position> keys = new ArrayList<>();
		keys.addAll(envPositions.get(TileStatus.KEY_GREEN));
		keys.addAll(envPositions.get(TileStatus.KEY_BLUE));
		keys.addAll(envPositions.get(TileStatus.KEY_YELLOW));
		keys.addAll(envPositions.get(TileStatus.KEY_RED));

		// Get the agent's current position
		Position selfPos = env.getRobotPosition(this);

		// Create a PriorityQueue to track all of the subgoals the agent will encounter
		PriorityQueue<Position> subgoals = new PriorityQueue<>(customComparator(selfPos));

		// Track the objects the agent has already picked up (chips, keys)
		ArrayList<String> robotInventoryList = env.getRobotHoldings(this);
		// Store those objects in a Map
		Map<TileStatus, Boolean> robotInventory = new HashMap<>();
		// For every item in the robot's inventory, add it to the map
		for(String inventoryItem : robotInventoryList) {
			TileStatus inventoryStatus = TileStatus.valueOf(inventoryItem);
			robotInventory.put(inventoryStatus, true);
		}

		// Have a boolean flag to check if all the chips on the map have been collected
		boolean allChipsCollected = true;
		// Add each chip to be collected to the list of subgoals
		for(Position chip : chips) {
			if(!robotInventoryList.contains(chip.toString())) {
				subgoals.add(chip);
			}
		}

		// Add all the door positions to the list of subgoals
		for(Position door : doors) {
			if(!door.equals(selfPos)) {
				subgoals.add(door);
			}
		}
		// Add all the key positions to the list of subgoals
		for(Position key : keys) {
			if(!key.equals(selfPos)) {
				subgoals.add(key);
			}
		}

		// Prioritize collecting all the chips by checking if there are chips remaining on the map
		// If there are, add it to the list of subgoals
		// In this case, if the agent still has chips left to collect on the map it will
		// prioritize this subgoal
		if(!allChipsCollected || env.getNumRemainingChips() > 0) {
			for(Position chip : chips) {
				if(!chip.equals(selfPos)) {
					subgoals.add(chip);
				}
			}
		}

		// Finally, add the GOAL portal to the list of subgoals (this is required for the agent
		// to pass the map)
		subgoals.add(goal);

		Action nextAction;

		// Go through the list of subgoals while it is not empty
		while(!subgoals.isEmpty()) {
			// Dequeue the first subgoal (this is the subgoal with the highest priority)
			Position nextSubgoal = subgoals.poll();

			// Retrieve the status of the Position of the subgoal - check if it's a door
			TileStatus doorStatus = env.getTiles().get(nextSubgoal).getStatus();
			boolean isDoor = false; // Use a boolean flag to check if it's a door
			// Check if it is one of the four types of doors
			if(doorStatus == TileStatus.DOOR_RED 
					|| doorStatus == TileStatus.DOOR_BLUE 
					|| doorStatus == TileStatus.DOOR_GREEN 
					|| doorStatus == TileStatus.DOOR_YELLOW) {
				isDoor = true;
			}
			// Get the type of key required to unlock that door, if that is the next subgoal
			TileStatus requiredKey = getKeyForDoor(nextSubgoal);
			// If the agent does NOT have the key it needs to unlock that door, skip this subgoal
			if(isDoor && !env.getRobotHoldings(this).contains(requiredKey.toString())) {
				continue; 
			}

			// This is a planning situation in which the agent reaches the DOOR_GOAL right
			// before the goal, but there are still chips left on the map
			// In this situation, if there are still chips left on the map BUT the 
			// agent is closer to the DOOR_GOAL than a chip, it will still prioritize
			// collecting a chip
			if(env.getTiles().get(nextSubgoal).getStatus().equals(TileStatus.DOOR_GOAL) && env.getNumRemainingChips() > 0) {

				// It will also ensure that all chips have been collected then, before going to the DOOR_GOAL
				for(Position chip : chips) {
					if(!chip.equals(selfPos)) {
						subgoals.add(chip);
					}
					// Check if the agent is closer to the DOOR_GOAL than the chips on the map
					if(calculateDistance(selfPos, nextSubgoal) < calculateDistance(selfPos, chip)) {
						nextSubgoal = subgoals.poll();
					}
				} 
				continue; 
			}

			// If the final GOAL is closer to the agent than another chip (and there are sitll chips remaining on the map)
			// then prioritize the chips
			if(env.getTiles().get(nextSubgoal).getStatus().equals(TileStatus.GOAL) && env.getNumRemainingChips() > 0) {
				for(Position chip : chips) {
					if(!chip.equals(selfPos)) {
						subgoals.add(chip);
					}
					if(calculateDistance(selfPos, nextSubgoal) < calculateDistance(selfPos, chip)) {
						nextSubgoal = subgoals.poll();
					}
				}
			}

			// Move towards the next subgoal using A* search algorithm
			nextAction = aStar(selfPos, nextSubgoal);

			// Update the agent's current position to be that of the next subgoal and keep going
			selfPos = nextSubgoal;

			// Make sure the final GOAL is in the list of subgoals and prioritize it
			if(!subgoals.contains(goal)) {
				subgoals.add(goal);
			} 

			// The method returns the next action from A* that the robot should take towards
			// a goal or subgoal
			return nextAction;
		}
		// Otherwise, there are no subgoals left, return no action
		return Action.DO_NOTHING;
	}

	/**
	 * This method creates a CUSTOM comparator that compares two positions based on the distance
	 * from a given position (the position of another subgoal to reach). 
	 * Essentially, this is what orders the subgoals in the priority queue. 
	 * Subgoals that the agent is closer to are higher in priority. 
	 * @param selfPos the agent's current position 
	 * @return the custom comparator for the Positions in the list of subgoals to help prioritize them
	 */
	private Comparator<Position> customComparator(Position selfPos) {
		return Comparator.comparingInt(p -> calculateDistance(selfPos, p)); 
	}

	/**
	 * This method is used to calculate the Manhattan distance (Manhattan heuristic) between two
	 * positions. 
	 * @param firstPos The first position
	 * @param secondPos The second position
	 * @return The Manhattan distance between both of the positions passed in
	 */
	private int calculateDistance(Position firstPos, Position secondPos) {
		return Math.abs(firstPos.getRow() - secondPos.getRow()) 
				+ Math.abs(firstPos.getCol() - secondPos.getCol());
	}

	/**
	 * This is a helper method to determine which key goes to which door.
	 * It checks the status of each tile at a given position and then
	 * it will give the matching key color for it (if it's a door).
	 * @param door the position to check 
	 * @return the TileStatus of the position
	 */
	private TileStatus getKeyForDoor(Position door) {
		if(env.getTiles().get(door).getStatus() == TileStatus.DOOR_RED) {
			return TileStatus.KEY_RED;
		} else if(env.getTiles().get(door).getStatus() == TileStatus.DOOR_BLUE)  {
			return TileStatus.KEY_BLUE;
		} else if(env.getTiles().get(door).getStatus() == TileStatus.DOOR_GREEN) {
			return TileStatus.KEY_GREEN;
		} else if(env.getTiles().get(door).getStatus() == TileStatus.DOOR_YELLOW) {
			return TileStatus.KEY_YELLOW;
		} else {
			return null;
		}
	}

	/**
	 * The method uses A* Search to find the shortest path to the target. 
	 * Robot.java also includes Nodes and a custom comparator to be used 
	 * in a Priority Queue to track and compare positions. 
	 * During the search, it calculates the cost associated with moving to each 
	 * new position, and it updates the frontier queue, cameFrom, and costSoFar 
	 * hash maps accordingly. 
	 * Once the algorithm finds the target, it reconstructs the path from the
	 * new current position of the Robot to the target. 
	 * The cameFrom map is used so that the robot can backtrack from the target position to
	 * the current position, and this is repeated until the target is found (unless
	 * it is unreachable). 
	 * After the path is reconstructed, the next action for the Robot to take is 
	 * determined using differences in the columns and rows between the current position 
	 * and the target. If the target is neighboring the current position 
	 * (the difference is 1 unit/action), then the agent moves in the appropriate 
	 * direction to reach the target. If not, then the robot does nothing. 
	 * @param start the starting position of the agent
	 * @param goal the goal position 
	 * @return an Action to take to get closer to the goal
	 */
	private Action aStar(Position start, Position goal) {

		// Priority Queue for A* search
		PriorityQueue<Node> frontier = new PriorityQueue<>();

		// Start exploring adjacent positions
		// cameFrom stores previous positions
		Map<Position, Position> cameFrom = new HashMap<>();

		// costSoFar gets updated with the cumulative cost associated with the 
		// new position reached
		Map<Position, Integer> costSoFar = new HashMap<>();

		// Add the current position to the queue with priority 0
		frontier.add(new Node(start, 0));

		// The starting position has no previous position. 
		// This is the initial position to put into the maps.
		cameFrom.put(start, null);
		costSoFar.put(start, 0);

		// This loop iterates through the positions in the queue until it's empty, or if the target is reached
		while(!frontier.isEmpty()) {

			// Get the position with the lowest priority from the frontier
			Node current = frontier.poll();

			// Check if the current position equals the target position
			if(current.position.equals(goal)) {
				Position currentPos = goal;
				Position previousPos = cameFrom.get(currentPos);
				// If the target position is unreachable
				if (previousPos == null) {
					return Action.DO_NOTHING;
				}

				// Reconstruct the path
				while(!previousPos.equals(start)) {
					// Update the current position to the previous position (move backwards)
					currentPos = previousPos;
					// Update the previous position to where the current position just was - reconstructing
					previousPos = cameFrom.get(currentPos);
				}

				// Determine which direction to go in
				return determinePath(start, currentPos);
			}

			// This map gets neighboring positions based on the CURRENT position
			Map<String, Position> neighbors = env.getNeighborPositions(current.position);

			// Iterate through all the neighboring positions
			for (Position next : neighbors.values()) {
				// For each position, get the tile and check if it is a wall/obstacle/barrier
				TileStatus nextTile = env.getTiles().get(next).getStatus();
				if(nextTile != null && nextTile != TileStatus.WALL && nextTile != TileStatus.WATER) {
					// Calculate the new cost to get to the next position
					int newCost = costSoFar.get(current.position) + 1;
					if (!costSoFar.containsKey(next) || newCost < costSoFar.get(next)) {
						// Update the cumulative cost
						costSoFar.put(next, newCost);
						// Use the Manhattan Distance Heuristic
						int manhattanDistance = Math.abs(next.getCol() - goal.getCol()) + Math.abs(next.getRow() - goal.getRow());
						int priority = newCost + manhattanDistance;
						// Add the next position to the queue with the calculated priority
						frontier.add(new Node(next, priority));
						// By recording the path taken to reach each position, the algo can
						// backtrack from the target to start
						cameFrom.put(next, current.position); 
					} 
				}
			}
		}
		return Action.DO_NOTHING;
	}

	/**
	 * This method reconstructs the path and works with A*, it returns the action for the A* method.
	 * It finds the difference between the current and target positions and decides the best
	 * action moving forward to make the difference smaller. 
	 * @param currentPos the agent's current position
	 * @param nextSubGoal the next subgoal (position) that the agent needs to get to 
	 * @return and Action to take
	 */
	private Action determinePath(Position currentPos, Position nextSubGoal) {
		// Determine where to go next based on the path to the next subgoal
		// Calculate the differences in column and row positions between the current position and the target
		int columnDifference = nextSubGoal.getCol() - currentPos.getCol();
		int rowDifference = nextSubGoal.getRow() - currentPos.getRow();

		// If the target is to the right of the current position, move right
		if (columnDifference == 1) {
			return Action.MOVE_RIGHT;
			// If the target is to the left of the current position, move left
		} else if (columnDifference == -1) {
			return Action.MOVE_LEFT;
			// If the target is below the current position, move down
		} else if (rowDifference == 1) {
			return Action.MOVE_DOWN;
			// If the target is above the current position, move up
		} else if (rowDifference == -1) {
			return Action.MOVE_UP;
		} else {
			return Action.DO_NOTHING;
		}
	}

	@Override
	public String toString() {
		return "Robot [pos=" + env.getRobotPosition(this) + "]";
	}

	/**
	 * Since Positions cannot be compared, an inner class 
	 * for a Node is used to make comparisons. 
	 * A Node contains a position and associated priority. 
	 * Nodes are comparable based on these priorities, so
	 * that they may be sorted in the frontier queue.
	 * @author zoyab
	 *
	 */
	private static class Node implements Comparable<Node> {
		/** The position associated with the Node */
		Position position;
		/** The priority of the Node */
		int priority;

		/**
		 * Constructor for a Node with a position and priority. 
		 * @param position the associated position
		 * @param priority the associated priority of it to use for comparisons
		 */
		Node(Position position, int priority) {
			this.position = position;
			this.priority = priority;
		}
		/**
		 * Compares the current node with a different node using the associated
		 * priorities.
		 * @param o the other node to compare 
		 * @return an integer representing the comparison value: 0 if equal,
		 * -1 if less than, and 1 if greater than
		 */
		@Override
		public int compareTo(Node o) {
			return Integer.compare(this.priority, o.priority);
		}

	}



}