package org.usfirst.frc.team3397.subsystems;

import org.usfirst.frc.team3397.robot.OI;
import org.usfirst.frc.team3397.robot.Robot;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Victor;
import java.util.logging.Logger;

public class DriveTrain {
	
	MecanumDrive chassis;
	OI controlScheme = new OI(0, 1);
	private static final Logger logger = Logger.getLogger(Robot.class.getName());
	
	public DriveTrain(Victor frontLeft, Victor frontRight, Victor backLeft, Victor backRight) {
		chassis = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
	}
	
	public void setExpiration(double timeout)
	{
		chassis.setExpiration(timeout);
	}
	
	public void setSafetyEnabled(boolean enabled)
	{
		chassis.setSafetyEnabled(enabled);
	}
	
	public void MecanumDrive() {
		double forward = controlScheme.getForward();
		double strafe = controlScheme.getStrafe();
		double turn = controlScheme.getTurn();
		
		double nonTurboSpeed = 0.7;
		double turboSpeed = 1.0;
		double turnSpeed = 0.5;
		
		double speedMultiplier = 0.0;
		
		double deadzone = 0.2;
		
		if (Math.abs(forward) >= deadzone || Math.abs(strafe) >= deadzone) {
//			if (controlScheme.getTurbo()) {
//				speedMultiplier = turboSpeed;
//			}
//			else
//			{
//				speedMultiplier = nonTurboSpeed;
//			}
			speedMultiplier = nonTurboSpeed;
			
			forward *= speedMultiplier;
			strafe *= -speedMultiplier;
			turn *= -speedMultiplier;
			
			chassis.driveCartesian(forward, strafe, turn);
			logger.info("Outputting drive!");
		}
		else if (Math.abs(turn) >= deadzone) {
			speedMultiplier = turnSpeed;
			turn *= -speedMultiplier;
			
			chassis.driveCartesian(0.0,  0.0,  turn);
			logger.info("Robot is turning");
		}
		else
		{
			chassis.driveCartesian(0.0, 0.0, 0.0);
		}
		
	}
	
	public void Drive(double speedForward) {
		speedForward *= -1;
		chassis.driveCartesian(speedForward, 0, 0);
		
	}
	
	public void Strafe(double speed) {
		speed *= -1;
		chassis.driveCartesian(0, speed, 0);
	}
	
//	public void Turn(double speed, double gyroAngle) {
//		speed *= -1;
//		chassis.driveCartesian(0, 0, speed, gyroAngle);
//	}
	
	public void Turn(double speed, double gyroAngle) {
		speed *= -1;
		chassis.driveCartesian(0, 0, speed, gyroAngle);
		logger.info("FAL;SRUWEA;KFJSDKL;RUAWEKL;JFASDL;UTJR");
	}
}
