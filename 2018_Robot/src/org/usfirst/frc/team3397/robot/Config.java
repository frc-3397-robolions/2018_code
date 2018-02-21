package org.usfirst.frc.team3397.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class Config {
	
	public int NUM_FRONT_LEFT_DRIVE;
	public int NUM_FRONT_RIGHT_DRIVE;
	public int NUM_BACK_LEFT_DRIVE = 0;
	public int NUM_BACK_RIGHT_DRIVE = 0;
	
	public int NUM_INTAKE_LEFT = 0;
	public int NUM_INTAKE_RIGHT = 0;
	public int NUM_INTAKE_LIFT = 0;
	
	public int NUM_ELEVATOR_LIFT = 0;
	
	public int NUM_CLIMB_HOOK = 0;
	public int NUM_CLIMB_WINCH = 0;
	
	public DigitalInput configInput = new DigitalInput(9);
	
	public boolean finalBot = false;
	
	public void determineConfig() {
//		if (configInput.get()) {
			finalBot = true;
			NUM_FRONT_LEFT_DRIVE = 3;
			NUM_FRONT_RIGHT_DRIVE = 2;
			NUM_BACK_LEFT_DRIVE = 1;
			NUM_BACK_RIGHT_DRIVE = 0;
			
			NUM_ELEVATOR_LIFT = 4;
			
			NUM_CLIMB_WINCH = 5;
			
//		}
//		else
//		{
//			finalBot = false;
			
			
//		}
	}
	
}
