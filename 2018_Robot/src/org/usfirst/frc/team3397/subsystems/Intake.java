package org.usfirst.frc.team3397.subsystems;

import org.usfirst.frc.team3397.robot.OI;
import org.usfirst.frc.team3397.robot.Robot;
import org.usfirst.frc.team3397.robot.Config;

import edu.wpi.first.wpilibj.Victor;

import java.util.logging.Logger;

public class Intake {
	
	Config config = new Config();
	OI controlScheme;
//	Robot robot = new Robot();

	public Victor intakeLift;
	public Victor leftIntake;
	public Victor rightIntake;
	
	double intakeSpeed;
	double intakeLiftSpeed;
	double shootSpeed;
	
	double tiltdesired;
	
	boolean INTAKE_UP;
	
	enum IntakeEnum {PUSH, PULL};
	
	private static final Logger logger = Logger.getLogger(Robot.class.getName());
	
	public Intake() {
		
		config.determineConfig();
		controlScheme = new OI(0, 1);
		
		intakeLift = new Victor(6);
		leftIntake = new Victor(8);
		rightIntake = new Victor(9);
		
		intakeSpeed = 0.5;
		intakeLiftSpeed = 0.5;
		shootSpeed = 1.0;
		
		INTAKE_UP = true;
		
	}
	
	public void intakeLift() {
		
		double tilterr;
		double tiltcmd;
		
		if (controlScheme.getIntakeUp()) {
			intakeLift.set(-intakeLiftSpeed);
			logger.info("Intake up");
		}
		else if (controlScheme.getIntakeDown()) {
			intakeLift.set(intakeLiftSpeed);
			logger.info("Intake down");
		}
		else
		{
			intakeLift.set(0.0);
			logger.info("Intake at rest");
		}
	}
	
	public void runIntake() {
		if (controlScheme.getRightIntakeOut() && controlScheme.getLeftIntakeOut()) {
			if (controlScheme.getTurboShoot()) {
				rightIntake.set(shootSpeed);
			}
			else
			{
				rightIntake.set(intakeSpeed);
			}
		}
		else if (controlScheme.getRightIntakeIn()) {
			rightIntake.set(-intakeSpeed);
			logger.info("Right wheel in");
			
		}
		else if (controlScheme.getRightIntakeOut()) {
			rightIntake.set(intakeSpeed);
			logger.info("Right wheel out");
		}
		else
		{
			rightIntake.set(-0.05);
		}
		
		if (controlScheme.getRightIntakeOut() && controlScheme.getLeftIntakeOut()) {
			if (controlScheme.getTurboShoot()) {
				leftIntake.set(-shootSpeed);
			}
			else
			{
				leftIntake.set(-intakeSpeed);
				logger.info("Shooting cube");
			}
		}
		else if (controlScheme.getLeftIntakeIn()) {
			leftIntake.set(intakeSpeed);
			logger.info("Left wheel in");
		}
		else if (controlScheme.getLeftIntakeOut()) {
			leftIntake.set(-intakeSpeed);
			logger.info("Left wheel out");
		}
		else
		{
			leftIntake.set(0.05);
			logger.info("Intake not running");
		}
	}
	
	public void setIntake(boolean PUSH, double speed) {
		if (PUSH) {
			rightIntake.set(speed);
			leftIntake.set(-speed);
		}
		else if (!PUSH) {
			rightIntake.set(-speed);
			leftIntake.set(speed);
		}
	}
}
