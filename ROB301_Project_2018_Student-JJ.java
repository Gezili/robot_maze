package ROB301_Project_2018;

import java.util.ArrayList;
//import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.PriorityQueue;
import java.util.concurrent.TimeUnit;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class ROB301_Project_2018_Student {
	static int asci_count = 0; // ASCII counter
	static int[] coord = new int [2]; // Keep track of coordinates
	static Map<Character, int[]> char_to_position; // Hash map maps node with given name to coordinate on map
	static char[][] my_map; // Stores maze map
	static char[] listHead = ['U', 'R', 'D', 'L']; // List of 4 possible Headings

	public static void main(String[] args) {
		int sizeMapX = 11; int sizeMapY = 11;
		char curPos = 'A'; // Start position of robot (to be updated)
		char curHead = 'R'; // Start orientation of robot (either 'U', 'D', 'L', 'R') (to be updated)
		char goalPos = 'Y'; // Final position the robot needs to reach
		char goalHead = 'U'; // Final orientation the robot needs to reach (either 'U', 'D', 'L', 'R')
		double wall_distance = 10;// to be determined

		List<Character> optPath; // Optimal path

		initializeMap(); // Initialize map with no walls
		Graph g = getGraph(my_map, sizeMapX, sizeMapY, char_to_position); // Create graph out of initialized map
		optPath = g.getShortestPath(curPos, goalPos); // Get optimal path from current position to goal
		System.out.println("Optimal Path: " + optPath);
		printMap(my_map); // Print map to see structure of map (can choose to print for debugging purposes)

		while (ifGoal == false){
			//
			/*my_map[1][6] = '1'; // Add a wall to the map (for demo)
			g = getGraph(my_map, sizeMapX, sizeMapY, char_to_position); // Create graph out of initialized map
			optPath = g.getShortestPath(curPos, goalPos); // Get optimal path from current position to goal
			System.out.println("Optimal Path: " + optPath);
			printMap(my_map); // Print map to see structure of map (can choose to print for debugging purposes)

			//Insert your code here...
			*/
			updateMap(curPos, curHead, wall_distance, my_map);
			g = getGraph(my_map, sizeMapX, sizeMapY, char_to_position); // Create graph out of updated map
			optPath = g.getShortestPath(curPos, goalPos); // Get optimal path from current position to goal
			System.out.println("Optimal Path: " + optPath);
			nextPos = optPath[0];
			nextHead = turnHead(curHead, curPos, nextPos); // write function to update curHead ** Currenly this update already turns it
			// Move one grid forward along it (i.e. from currPos to nextPos)

			// Update curPos and curHead
			curHead = nextHead;
			curPos = nextPos;
		}
		printMap(my_map); // Print map to see structure of map (can choose to print for debugging purposes)
	}

	//public static char

	public static boolean ifGoal(char curPos,char curHead,char goalPos,char goalHead){
		/* return true and execute the turning if goal is reached
			 return false if not
		 */
		if(curPos != goalPos){
			return false;
		}
		else{
			System.out.println("Goal is reached!");
			curHeadIndex = listHead.indexOf(curHead);
			goalHeadIndex = listHead.indexOf(goalHead);
			direction = goalHeadIndex - curHeadIndex;
			switch (direction) {
				case 1: case -1: case 2: case -2: robot_reading.turn_90(direction);
					break;
				case 3:
					robot_reading.turn_90(-1);
					break;
				case -3: robot_reading.turn_90(1);
					break;
				default: break;
			}
			return true;
		}
	}

	public static char turnHead(char curHead char curPos, char nextPos){
		/* Use the difference between the current position and desired position (must be adjacent)
			 to determine the heading and turn it. Return nextHead
			 Examples:
			 * cur: (0,0) --> next: (0,1): nextHead = R
			 * cur: (0,0) --> next: (1,0): nextHead = D
			 * cur: (0,1) --> next: (0,0): nextHead = L
			 * cur: (1,0) --> next: (0,0): nextHead = U
		 */
		 int [] curCoord = char_to_postion.get(curPos);
		 int [] nextCoord = char_to_postion.get(nextPos);
		 int [] deltaCoord = [[ - ], [curCoord[1] - nextCoord[1]]];

		 // determine which direction it should be heading
		 if(curCoord[0] == nextCoord[0]){
			 if (curCoord[1] == nextCoord[1]+1){
				 nextHead = 'L';
			 }
			 else if (curCoord[1] == nextCoord[1]-1){
				 nextHead = 'R';
			 }
		 }
		 else if(curCoord[1] == nextCoord[1]){
			 if (curCoord[0] == nextCoord[0]+1){
				 nextHead = 'U';
			 }
			 else if (curCoord[0] == nextCoord[0]-1){
				 nextHead = 'D';
			 }
		 }

		 // determine how it should turn to that direction and execute the turn
		 curHeadIndex = listHead.indexOf(curHead);
		 nextHeadIndex = listHead.indexOf(nextHead);
		 direction = nextHeadIndex - curHeadIndex;
		 switch (direction) {
			 case 1: case -1: case 2: case -2: robot_reading.turn_90(direction);
				 break;
			 case 3:
				 robot_reading.turn_90(-1);
				 break;
			 case -3: robot_reading.turn_90(1);
				 break;
			 default: break;
		 }
		return nextHead;
	}

	public static void initializeMap(){
		/* Map should look like:
		 *  ZZZZZZZZZZZ
			ZA0B0C0D0EZ
			Z0Z0Z0Z0Z0Z
			ZF0G0H0I0JZ
			Z0Z0Z0Z0Z0Z
			ZK0L0M0N0OZ
			Z0Z0Z0Z0Z0Z
			ZP0Q0R0S0TZ
			Z0Z0Z0Z0Z0Z
			ZU0V0W0X0YZ
			ZZZZZZZZZZZ

			Hash map char_to_postion is like a dictionary relating characters (e.g. 'A') to coordinates (e.g. [1,1]) in my_map

			Note that positive X is right and positive Y is down
			Z character is a null entry of the map
			Alphabetical characters from A to Y are potential positions the robot can be in
			Numerical characters can hold either 0 or 1 (to signify empty space or wall respectively between its neighbouring positions)
		 */

		char_to_position = new HashMap<Character, int[]>(); // Create hash from character to position in map
		my_map = new char[11][11]; // Create map from position to character (i.e. regular map)
		char letter; // Holds character corresponding to a position in the map
		// Populate entire array with Z
		for(int i = 0; i < 11; i++){
			for(int j =0; j < 11; j ++){
				my_map[i][j] = 'Z';
			}
		}
		// Populate inner map area with 0's to signify free path between robot positions
		for(int i = 1; i < 10; i++){
			for(int j =1; j < 10; j ++){
				my_map[i][j] = '0';
			}
		}
		// Populate cells from A-Y where robot will go
		for(int i = 1; i < 10; i+=2){
			for(int j =1; j < 10; j +=2){
				int[] coord = new int [2]; // Must create new array object so since hash map points all keys to same
				letter = (char)(65+asci_count);
				my_map[i][j] = letter;
				coord [0] = i; coord[1] = j;
				char_to_position.put(letter, coord);
				asci_count++;
			}
		}

		//Rest of map is padded with Z character to make parsing the map easier to implement
		for(int i = 2; i < 10; i+=2){
			for(int j =2; j < 10; j +=2){
				my_map[i][j] = 'Z';
			}
		}
	}

	public static void updateMap_old(char curPos, char curHead, double wall_distance,char[][] map){
		/***
		 * Inputs: current Position, current Heading
		 * Outputs: None
		 * Function: Use current position and heading to correctly add a wall to the map my_map
		***/

		// Insert your code here...
		int [] curCoord = char_to_postion.get(curPos);
		double sonic_data = robot_reading.get_sonic_reading();
		if(sonic_data <= wall_distance){
			if(curHead == 'U'){
				int wall_x = curCoord[0];
				int wall_y = curCoord[1]+1;
			}
			else if(curHead == 'D'){
				int wall_x = curCoord[0];
				int wall_y = curCoord[1]-1;
			}
			else if(curHead == 'L'){
				int wall_x = curCoord[0]-1;
				int wall_y = curCoord[1];
			}
			else if(curHead == 'R'){
				int wall_x = curCoord[0]+1;
				int wall_y = curCoord[1];
			}
		}
		if(map[wall_x][wall_y] != 'Z'){
			map[wall_x][wall_y] = '1';
		}
	}

	public static void updateMap(char curPos, char curHead, char[][] map){
		/***
		 * Inputs: current Position, current Heading
		 * Outputs: None
		 * Function: Use current position and heading to correctly add a wall to the map my_map
		***/

		// Insert your code here...
		int [] curCoord = char_to_postion.get(curPos);

		if(curHead == 'U'){
			int wall_x = curCoord[0];
			int wall_y = curCoord[1]+1;
		}
		else if(curHead == 'D'){
			int wall_x = curCoord[0];
			int wall_y = curCoord[1]-1;
		}
		else if(curHead == 'L'){
			int wall_x = curCoord[0]-1;
			int wall_y = curCoord[1];
		}
		else if(curHead == 'R'){
			int wall_x = curCoord[0]+1;
			int wall_y = curCoord[1];
		}
		if(map[wall_x][wall_y] != 'Z'){
			map[wall_x][wall_y] = '1';
		}
	}

	public static Graph getGraph(char[][] map, int sizeX, int sizeY, Map<Character, int[]> char_to_position){
		// Iterate through each robot position of the map
		char[] neighbours;
		Graph g = new Graph();
		char letter;
		for(int i = 1; i < sizeX-1; i+=2){
			for(int j =1; j < sizeY-1; j +=2){
				letter = map[i][j]; // Get current letter we're on and create edges from this on the graph
				neighbours = getNeighbours(letter, map, char_to_position);
				ArrayList<Vertex> vertices = new ArrayList<Vertex>();
				for(int k=0; k < 4; k++){ // Iterate over all neighbours of current position in map
					if(neighbours[k] != 'Z'){
						vertices.add(new Vertex(neighbours[k],1));
					}else{
						break;
					}
				}
				g.addVertex(letter, vertices); // Add list of neighbouring vertices to graph
			}
		}
		return g;
	}

	public static char[] getNeighbours(char letter, char[][] map, Map<Character, int[]> char_to_position){
		/***
		 * Inputs: position (char identifier of position in map we want to get the neighbours of)
		 * 		   map (my_map variable above)
		 * 		   char_to_position (hash map, see explanation in initializaMap() )
		 * Outputs: character array size between 1 and 4 of the neighbours (e.g. if we query H, return char will be 'C','I','M','G')
		 * Function: Return neighbors to queried node
		***/

		char[] neighbours = {'Z','Z','Z','Z'}; // Initialize neighbours to null type
		int[] coord = new int[2];
		coord = char_to_position.get(letter);
		int n_index = 0;

		//Check if any of the four neighbouring positions are free for the robot to travel to
		if(map[coord[0]-1][coord[1]] == '0'){
			neighbours[n_index] = map[coord[0]-2][coord[1]];
			n_index++;
		}
		if(map[coord[0]+1][coord[1]] == '0'){
			neighbours[n_index] = map[coord[0]+2][coord[1]];
			n_index++;
		}
		if(map[coord[0]][coord[1]-1] == '0'){
			neighbours[n_index] = map[coord[0]][coord[1]-2];
			n_index++;
		}
		if(map[coord[0]][coord[1]+1] == '0'){
			neighbours[n_index] = map[coord[0]][coord[1]+2];
		}

		return neighbours;
	}

	public static void printMap(char[][] map){
		for(int i = 0; i < 11; i++){
			for(int j =0; j < 11; j ++){
				System.out.print(map[i][j]);
			}
			System.out.println("");
		}
	}
}

// DO NOT CHANGE FOLLOWING CODE. Path planning implementation
class Vertex implements Comparable<Vertex> {

	private Character id;
	private Integer distance;

	public Vertex(Character id, Integer distance) {
		super();
		this.id = id;
		this.distance = distance;
	}

	public Character getId() {
		return id;
	}

	public Integer getDistance() {
		return distance;
	}

	public void setId(Character id) {
		this.id = id;
	}

	public void setDistance(Integer distance) {
		this.distance = distance;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result
				+ ((distance == null) ? 0 : distance.hashCode());
		result = prime * result + ((id == null) ? 0 : id.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		Vertex other = (Vertex) obj;
		if (distance == null) {
			if (other.distance != null)
				return false;
		} else if (!distance.equals(other.distance))
			return false;
		if (id == null) {
			if (other.id != null)
				return false;
		} else if (!id.equals(other.id))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "Vertex [id=" + id + ", distance=" + distance + "]";
	}

	@Override
	public int compareTo(Vertex o) {
		if (this.distance < o.distance)
			return -1;
		else if (this.distance > o.distance)
			return 1;
		else
			return this.getId().compareTo(o.getId());
	}

}

class Graph {
	public final Map<Character, List<Vertex>> vertices;

	public Graph() {
		this.vertices = new HashMap<Character, List<Vertex>>();
	}

	public void addVertex(Character character, List<Vertex> vertex) {
		this.vertices.put(character, vertex);
	}

	public void createHashMap(){

	}

	public List<Character> getShortestPath(Character start, Character finish) {
		final Map<Character, Integer> distances = new HashMap<Character, Integer>();
		final Map<Character, Vertex> previous = new HashMap<Character, Vertex>();
		PriorityQueue<Vertex> nodes = new PriorityQueue<Vertex>();

		for(Character vertex : vertices.keySet()) {
			if (vertex == start) {
				distances.put(vertex, 0);
				nodes.add(new Vertex(vertex, 0));
			} else {
				distances.put(vertex, Integer.MAX_VALUE);
				nodes.add(new Vertex(vertex, Integer.MAX_VALUE));
			}
			previous.put(vertex, null);
		}

		while (!nodes.isEmpty()) {
			Vertex smallest = nodes.poll();
			if (smallest.getId() == finish) {
				final List<Character> path = new ArrayList<Character>();
				while (previous.get(smallest.getId()) != null) {
					path.add(smallest.getId());
					smallest = previous.get(smallest.getId());
				}
				return path;
			}

			if (distances.get(smallest.getId()) == Integer.MAX_VALUE) {
				break;
			}

			for (Vertex neighbor : vertices.get(smallest.getId())) {
				Integer alt = distances.get(smallest.getId()) + neighbor.getDistance();
				if (alt < distances.get(neighbor.getId())) {
					distances.put(neighbor.getId(), alt);
					previous.put(neighbor.getId(), smallest);

					forloop:
					for(Vertex n : nodes) {
						if (n.getId() == neighbor.getId()) {
							nodes.remove(n);
							n.setDistance(alt);
							nodes.add(n);
							break forloop;
						}
					}
				}
			}
		}

		return new ArrayList<Character>(distances.keySet());
	}
}
