import lejos.hardware.motor.*;
import lejos.hardware.Button;

public class Testing {

	public static void main(String[] args) throws Exception{
		
		robot_reading robotreading = new robot_reading();
		pidcontroller pid = new pidcontroller();
		robot_control robotcontrol = new robot_control();
		
		Button.waitForAnyPress();
		
		robotcontrol.move_1_grid();
		robotcontrol.move_1_grid();
		
		robotcontrol.turn_90(); //TURN LEFT 90
		
		for (int i = 1; i < 5; i++){
			robotcontrol.move_until_wall();
			robotcontrol.turn_90();
		}
	}
}
