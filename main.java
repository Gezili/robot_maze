import lejos.hardware.motor.*;
import lejos.hardware.Button;

public class Testing {

	public static void main(String[] args) throws Exception{
		double floor = 0.35;
		double line = 0.09;
		double mid = 0.21;
		double turn_sensitivity_thresh = 0.15;
		double grid_length = 30.48;
		boolean control_run = true;
		double speed = 0;
		double magic_number_sonic = 6.5;
		
		robot_reading robotreading = new robot_reading();
		pidcontroller pid = new pidcontroller();
		robot_control robotcontrol = new robot_control();
		
		
		Button.waitForAnyPress();
		for (int i = 1; i < 8; i++) {
			
			/*
			while(!Button.ENTER.isDown()){
				System.out.println("Ready");
			}
			*/
				
			while(robotreading.get_sonic_reading() >= magic_number_sonic){
				/*
				if(Button.ESCAPE.isDown()){
					break;
				} 
				*/
				speed = pid.run();
				control_run = robotcontrol.run(grid_length, speed);
			}
			
			robotcontrol.turn_45(-1);
			while(robotreading.get_color_reading() >= turn_sensitivity_thresh){
				robot_control.turn_increment(-1);
			}
			control_run = true;
		}
	}
}


