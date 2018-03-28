package org.usfirst.frc.team3397.subsystems;

import org.usfirst.frc.team3397.robot.OI;
import org.usfirst.frc.team3397.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Victor;

import java.util.logging.Logger;
import com.kauailabs.navx.frc.AHRS;

public class DriveTrain {
	
	MecanumDrive chassis;
	OI controlScheme = new OI(0, 1);
	private static final Logger logger = Logger.getLogger(Robot.class.getName());
	
	Victor frontLeftMotor;
	Victor frontRightMotor;
	Victor backLeftMotor;
	Victor backRightMotor;
	
	AHRS ahrs;
	
	public DriveTrain(Victor frontLeft, Victor frontRight, Victor backLeft, Victor backRight) {
		chassis = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
		frontLeftMotor = frontLeft;
		frontRightMotor = frontRight;
		backLeftMotor = backLeft;
		backRightMotor = backRight;
		
		try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
            ahrs = new AHRS(SPI.Port.kMXP); 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
		
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
		
		if (Math.abs(forward) <= deadzone) {
			forward = 0.0;
		}
		
		if (Math.abs(strafe) <= deadzone) {
			strafe = 0.0;
		}
		if (Math.abs(turn) <= deadzone) {
			turn = 0.0;
		}
		
		if (Math.abs(forward) >= deadzone || Math.abs(strafe) >= deadzone) 
		{
//			if (controlScheme.getTurbo()) {
//				speedMultiplier = turboSpeed;
//			}
//			else
//			{
//				speedMultiplier = nonTurboSpeed;0
//			}
			if (Math.abs(forward) <= deadzone) {
				forward = 0.0;
			}
			
			if (Math.abs(strafe) <= deadzone) {
				strafe = 0.0;
			}
			if (Math.abs(turn) <= deadzone) {
				turn = 0.0;
			}
			
			speedMultiplier = -nonTurboSpeed;
			
			forward *= speedMultiplier;
			strafe *= -speedMultiplier;
			turn *= -speedMultiplier;
			
//			chassis.driveCartesian(0.0, strafe, turn);
//			frontLeftMotor.set(forward);
//			frontRightMotor.set(-forward);
//			backLeftMotor.set(forward);
//			backRightMotor.set(-forward);
			chassis.driveCartesian(forward, strafe, turn);
			logger.info("Outputting drive!");
		}

		else if (Math.abs(turn) >= deadzone) {
			speedMultiplier = turnSpeed;
			turn *= speedMultiplier;
			
			chassis.driveCartesian(0.0,  0.0,  turn);
			logger.info("Robot is turning");
		}
		else
		{
			chassis.driveCartesian(0.0, 0.0, 0.0);
		}
		
		SmartDashboard.putNumber("Forward Speed", forward);
		SmartDashboard.putNumber("Right wheels speed", -forward);
		SmartDashboard.putNumber("Strafe Speed", strafe);
		SmartDashboard.putNumber("Turn Speed", turn);
		SmartDashboard.putNumber("FR Speed", frontRightMotor.get());
		SmartDashboard.putNumber("FL Speed", frontLeftMotor.get());
		SmartDashboard.putNumber("BR Speed", backRightMotor.get());
		SmartDashboard.putNumber("BL Speed", backLeftMotor.get());
		
	}
	
	public void angleDrive(double speed, double angle) {
//		speed *= -1;
		double angleDiff = angle - ahrs.getAngle();
		
		double turnInput = angleDiff * 0.05;
		if (turnInput > 0.3) {
			turnInput = 0.3;
		}
		else if (turnInput < -0.3) {
			turnInput = -0.3;
		}
		
		SmartDashboard.putNumber("Turn Input for straight driving", turnInput);
		
		chassis.driveCartesian(speed, 0, turnInput);
	}
	
	public void Drive(double speed) {
		speed *= -1;
		chassis.driveCartesian(speed, 0.0, 0.0);
	}
	
	public void stop() {
		chassis.driveCartesian(0.0, 0.0, 0.0);
	}
	
	public void Strafe(double speed) {
		speed *= -1;
		chassis.driveCartesian(0, speed, 0);
	}
	
	public void test(Victor frontLeft, Victor frontRight, Victor backLeft, Victor backRight, double speed) {
		frontLeft.set(speed);
		frontRight.set(-speed);
		backLeft.set(speed);
		backRight.set(-speed);
	}
	
	public void driveCart(double forward, double strafe, double turn) {
		chassis.driveCartesian(forward, strafe, turn);
	}
	
//	public void Turn(double speed, double gyroAngle) {
//		speed *= -1;
//		chassis.driveCartesian(0, 0, speed, gyroAngle);
//	}
	
	public void Turn(double speed, double gyroAngle) {
		speed *= 1;
		chassis.driveCartesian(0, 0, speed, gyroAngle);
		logger.info("FAL;SRUWEA;KFJSDKL;RUAWEKL;JFASDL;UTJR");
	}
}
