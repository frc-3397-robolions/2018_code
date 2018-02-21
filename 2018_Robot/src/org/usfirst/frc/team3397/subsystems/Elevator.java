package org.usfirst.frc.team3397.subsystems;

import org.usfirst.frc.team3397.robot.OI;
import org.usfirst.frc.team3397.robot.Robot;
import org.usfirst.frc.team3397.robot.Config;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.DigitalInput;

import java.util.logging.Logger;

public class Elevator {
	
//	RobotLayout rLayout;
	Config config = new Config();
	OI controlScheme;
	public Victor elevatorMotor;
	public DigitalInput elevatorTopStop;
	double elevatorSpeed;
	boolean elevatorUpCtrl;
	boolean elevatorDnCtrl;
	boolean ELEVATOR_TOP_LIMIT;
	
	private static final Logger logger = Logger.getLogger(Robot.class.getName());
	
	public Elevator() {
		config.determineConfig();
		controlScheme = new OI(0, 1);
		
		elevatorTopStop = new DigitalInput(0);
		
		elevatorMotor = new Victor(config.NUM_ELEVATOR_LIFT);
		elevatorSpeed = 0.5;
		
		elevatorUpCtrl = controlScheme.getElevatorUp();
		elevatorDnCtrl = controlScheme.getElevatorDown();
		
		ELEVATOR_TOP_LIMIT = elevatorTopStop.get();
	}
	
	public void useElevator() {
		if (controlScheme.getElevatorUp() == true) {
			if (ELEVATOR_TOP_LIMIT) {
				elevatorMotor.set(-elevatorSpeed);
				logger.info("Elevator going up");
			}
			else
			{
				elevatorMotor.set(0.0);
				logger.info("Elevator has stopped at the top");
			}
		}
		else if (controlScheme.getElevatorDown() == true) {
			elevatorMotor.set(elevatorSpeed);
			logger.info("Elevator going down");
		}
		else
		{
			elevatorMotor.set(0.0);
			logger.info("Elevator idle");
		}
	}
}
