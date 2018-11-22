//package ROB301_Project_2018;

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
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class FinalProjectMonday {
	static int asci_count = 0; // ASCII counter
	static int[] coord = new int [2]; // Keep track of coordinates
	static Map<Character, int[]> char_to_position; // Hash map maps node with given name to coordinate on map
	static char[][] my_map; // Stores maze map
	
	public static void main(String[] args) throws InterruptedException {  
		int sizeMapX = 11; int sizeMapY = 11;
		char curPos = 'A'; // Start position of robot (to be updated)
		char curHead = 'R'; // Start orientation of robot (either 'U', 'D', 'L', 'R') (to be updated)
		char goalPos = 'Y'; // Final position the robot needs to reach
		char goalHead = 'U'; // Final orientation the robot needs to reach (either 'U', 'D', 'L', 'R')
		
		List<Character> optPath; // Optimal path
		
		initializeMap(); // Initialize map with no walls
		Graph g = getGraph(my_map, sizeMapX, sizeMapY, char_to_position); // Create graph out of initialized map
		optPath = g.getShortestPath(curPos, goalPos); // Get optimal path from current position to goal
		System.out.println("Optimal Path: " + optPath);
		//printMap(my_map); // Print map to see structure of map (can choose to print for debugging purposes)
		
		my_map[10][6] = '1'; // Add a wall to the map (for demo)
		g = getGraph(my_map, sizeMapX, sizeMapY, char_to_position); // Create graph out of initialized map
		optPath = g.getShortestPath(curPos, goalPos); // Get optimal path from current position to goal
		System.out.println("Optimal Path: " + optPath);
		while(true){
			if(Button.ESCAPE.isDown()){
				break;
			} 
			
			//pi.run();
			printMap(my_map);
			Thread.sleep(5000);
	}
		 // Print map to see structure of map (can choose to print for debugging purposes)
		
		// Insert your code here...
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
	
	public static void updateMap(char curPos, char curHead){ 
		/***
		 * Inputs: current Position, current Heading
		 * Outputs: None
		 * Function: Use current position and heading to correctly add a wall to the map my_map 
		***/
		
		// Insert your code here...
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

class robot_control{
	
	double sensor_data; //this is the value returned by sensor	
	long time_start;
	long time_end;
	long d_t;
	double robot_distance = 0;
	boolean start_flag = false;
	double magic_number = 0.8;
	double grid_length = 17.5;
	pidcontroller pid = new pidcontroller();
	robot_reading robotreading = new robot_reading();
	double turn_sensitivity_thresh = 0.15;
	double speed = 0;
	double magic_number_sonic = 6.5;
	
	public void reset_flags(){
		
		robot_distance = 0;
		start_flag = true;
		
	}
	
	public double run(double speed) throws InterruptedException{
		time_start = System.nanoTime();
		//sensor_data = robot_reading.get_sonic_reading();
		System.out.println(robot_distance);
		//robot_reading.turn(motor_speed, motor_speed);
		time_end = System.nanoTime();
		d_t = time_end - time_start;
		if (start_flag == false){
			start_flag = true;
			d_t = 0;
		}
		
		robot_distance += ((speed)*0.0275)*d_t*2*magic_number/1000000000;
		
		return robot_distance;
			
	}
	
	public void turn_90() throws InterruptedException{
		turn_45(-1);
		while(robotreading.get_color_reading() >= turn_sensitivity_thresh){
			robot_control.turn_increment(-1);
		}
	}
	
	public void move_1_grid() throws InterruptedException{
		while (run(100) < grid_length){
		/*
			if(Button.ESCAPE.isDown()){
				break;
			} 
			*/
			speed = pid.run();
		}
	
		reset_flags();
		pid.resetpid();
		}	

	//We want this to output a distance (?)

	public static void turn_increment(int direction) throws InterruptedException{
		
		Motor.B.setSpeed(90);
		Motor.C.setSpeed(90);
		
		if (direction == -1){
			Motor.B.backward();
			Motor.C.forward();
		} else if (direction == 1){
			Motor.B.forward();
			Motor.C.backward();
		}
		Thread.sleep((long) (10));
	}
	
	public void turn_45(int direction) throws InterruptedException{
		Motor.B.setSpeed(90);
		Motor.C.setSpeed(90);
		Motor.B.rotate(direction*92, true);
		Motor.C.rotate(direction*(-92));
		
		Thread.sleep((long) (184.090909090*1000/90));
	}
	
	public void move_until_wall(){
		while(robotreading.get_sonic_reading() >= magic_number_sonic){
			/*
			if(Button.ESCAPE.isDown()){
				break;
			} 
			*/
			speed = pid.run();
			//control_run = robotcontrol.run(grid_length, speed);
		}
	}
	public static void turn(double lspeed, double rspeed) {
		
		int left_speed_round = (int) lspeed;
		int right_speed_round = (int) rspeed;
		
		Motor.B.setSpeed(Math.abs(left_speed_round));
		if(lspeed > 0){
			Motor.B.forward();
		} else if(lspeed < 0) {
			Motor.B.backward();
		} else {
			Motor.B.stop(true);
		}
		
		Motor.C.setSpeed(Math.abs(right_speed_round));
		if(rspeed > 0){
			Motor.C.forward();
		} else if(rspeed < 0) {
			Motor.C.backward();
		} else {
			Motor.C.stop(true);
		}		
	}
	
	
}



class pidcontroller{
	
	double k_p = 250;
	double k_i = 5;
	double k_d = 500;
	double derivative = 0;
	double lasterror = 0;
	double integral= 0;
	
	double floor = 0.35;
	double line = 0.09;
	double mid = 0.21;

	double target = 0.21; //set target value
	int direction; 
	double speed;
	double sensor_data; //this is the vlaue returnd by sensor
	
	public void resetpid(){
		integral = 0;
	}
	
	public double run(){
		sensor_data = robot_reading.get_color_reading();
		
		
		double error = target - sensor_data; //if positive - target is larger - turn left

		derivative = error - lasterror;
		lasterror = error;
		
		integral *= 0.98;
		integral += error;
		
		//if negative - target is smaller - turn right
		double rightspeed = 90 + k_p*error + k_d*derivative + k_i*integral;
		double leftspeed = 90 - k_p*error - k_d*derivative - k_i*integral;
	
		robot_control.turn(leftspeed, rightspeed);
		return (leftspeed + rightspeed)/2;
				
	}
}


class robot_reading{
	
	public static final EV3ColorSensor color = new EV3ColorSensor(SensorPort.S3);
	public static final EV3UltrasonicSensor sonic = new EV3UltrasonicSensor(SensorPort.S2);

	public static float get_color_reading() {
		
		int sampleSize = color.sampleSize();
		float[] idsample = new float[sampleSize];
		color.getRedMode().fetchSample(idsample, 0);
		LCD.clear();
		return idsample[0];
	}

	public float get_sonic_reading() {
	
		int sampleSize = sonic.sampleSize();
		float[] sonicsample = new float[sampleSize];
		sonic.fetchSample(sonicsample, 0);
		System.out.println(sonicsample[0]*100);
		LCD.clear();
		return sonicsample[0]*100;
	}
	

}
