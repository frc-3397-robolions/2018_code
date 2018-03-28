package org.usfirst.frc.team3397.subsystems;

import org.usfirst.frc.team3397.robot.OI;
import org.usfirst.frc.team3397.robot.Robot;
import org.usfirst.frc.team3397.robot.Config;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;

import java.util.logging.Logger;

public class Elevator {
	
//	RobotLayout rLayout;
	Config config = new Config();
	OI controlScheme;
	public Victor elevatorMotor;
	public DigitalInput elevatorTopStop;
	public AnalogInput elevatorPot;
	double elevatorPos;
	double elevatorPosDesired;
	double elevatorSpeed;
	double elevatorSpeedDown;
	boolean elevatorUpCtrl;
	boolean elevatorDnCtrl;
	boolean ELEVATOR_TOP_LIMIT;
	
	private static final Logger logger = Logger.getLogger(Robot.class.getName());
	
	public Elevator() {
		config.determineConfig();
		controlScheme = new OI(0, 1);
		
		elevatorTopStop = new DigitalInput(0);
		elevatorPot = new AnalogInput(1);
		
		elevatorMotor = new Victor(4);
		elevatorSpeed = 0.75;
		elevatorSpeedDown = 0.4;
		
		elevatorUpCtrl = controlScheme.getElevatorUp();
		elevatorDnCtrl = controlScheme.getElevatorDown();
		
		ELEVATOR_TOP_LIMIT = elevatorTopStop.get();
	}
	
	public void useElevator() {
		elevatorPos = elevatorPot.getVoltage();
		SmartDashboard.putNumber("Elevator Position", elevatorPos);
		if (controlScheme.getElevatorUp() == true) {
			if (ELEVATOR_TOP_LIMIT || elevatorPos < 0.3) {
				if (elevatorPos > 0.7) {
					elevatorMotor.set(-elevatorSpeed);
				}
				else if (elevatorPos < 0.7 && elevatorPos > 0.3) {
					elevatorMotor.setSpeed(-elevatorSpeed * 0.5);
				}
				else
				{
					elevatorMotor.setSpeed(-0.09);
				}
				logger.info("Elevator going up");
			}
			else
			{
				elevatorMotor.set(-0.09);
				logger.info("Elevator has stopped at the top");
			}
			
			
		}
		else if (controlScheme.getElevatorDown() == true) {
			if (elevatorPos >= 2.9) {
				elevatorMotor.set(0.2);
			}
			else
			{
				elevatorMotor.set(elevatorSpeedDown);
			}
			logger.info("Elevator going down");
		}
		else
		{
			elevatorMotor.set(-0.09);
			logger.info("Elevator idle");
		}
	}
	
	public void setPos(double position) {
		elevatorPos = elevatorPot.getVoltage();
		double error = position - elevatorPos;
		double cmd = 1 * error;
		elevatorMotor.set(cmd);
	}
}
