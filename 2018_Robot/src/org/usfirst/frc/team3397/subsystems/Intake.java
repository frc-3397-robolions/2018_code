package org.usfirst.frc.team3397.subsystems;

import org.usfirst.frc.team3397.robot.OI;
import org.usfirst.frc.team3397.robot.Robot;

import edu.wpi.first.wpilibj.Victor;

import java.util.logging.Logger;

public class Intake {
	
	OI controlScheme;

	public Victor intakeLift;
	public Victor leftIntake;
	public Victor rightIntake;
	
	double intakeSpeed;
	double intakeLiftSpeed;
	double shootSpeed;
	
	boolean INTAKE_UP;
	
	private static final Logger logger = Logger.getLogger(Robot.class.getName());
	
	public Intake() {
		
		controlScheme = new OI(0, 1);
		
		intakeLift = new Victor(7);
		leftIntake = new Victor(9);
		rightIntake = new Victor(8);
		
		intakeSpeed = 0.5;
		intakeLiftSpeed = 0.5;
		shootSpeed = 1.0;
		
		INTAKE_UP = true;
		
	}
	
	public void intakeLift() {
		if (controlScheme.getIntakeUp()) {
			intakeLift.set(intakeLiftSpeed);
			logger.info("Intake up");
		}
		else if (controlScheme.getIntakeDown()) {
			intakeLift.set(-intakeLiftSpeed);
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
			rightIntake.set(shootSpeed);
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
			rightIntake.set(0.0);
		}
		
		if (controlScheme.getRightIntakeOut() && controlScheme.getLeftIntakeOut()) {
			leftIntake.set(-shootSpeed);
			logger.info("Shooting cube");
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
			leftIntake.set(0.0);
			logger.info("Intake not running");
		}
	}
}
