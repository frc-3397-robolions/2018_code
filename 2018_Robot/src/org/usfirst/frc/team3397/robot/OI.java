package org.usfirst.frc.team3397.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 *  This class assigns functions to each controller.
 */

public class OI {
	
	XboxController driveStick;
	Joystick operatorStick;
	
	public OI(int drivePort, int operatePort) {
		driveStick = new XboxController(drivePort);
		operatorStick = new Joystick(operatePort);
	}
	
	// Xbox controls //
	
	public double getForward() {
		return driveStick.getY(Hand.kLeft);
	}
	
	public double getStrafe() {
		return driveStick.getX(Hand.kLeft);
	}
	
	public double getTurn() {
		return driveStick.getX(Hand.kRight);
	}
	
	public boolean getRightIntakeIn() {
		return driveStick.getTriggerAxis(Hand.kRight) >= 0.75;
	}
	
	public boolean getLeftIntakeIn() {
		return driveStick.getTriggerAxis(Hand.kLeft) >= 0.75;
	}
	
	public boolean getRightIntakeOut() {
		return driveStick.getBumper(Hand.kRight);
	}
	
	public boolean getLeftIntakeOut() {
		return driveStick.getBumper(Hand.kLeft);
	}
	
	public boolean test() {
		return operatorStick.getRawButton(4);
	}
	
	public boolean getX() {
		return driveStick.getXButton();
	}
	
	public boolean getTurboShoot() {
		return driveStick.getBButton();
	}
	
	public void vibrate() {
		driveStick.setRumble(RumbleType.kLeftRumble, 1);
		driveStick.setRumble(RumbleType.kRightRumble, 1);
	}
	
	public void vibrateOff() {
		driveStick.setRumble(RumbleType.kLeftRumble, 0);
		driveStick.setRumble(RumbleType.kRightRumble, 0);
	}
	
	// Operator stick controls //
	
	public boolean getElevatorUp() {
		return operatorStick.getRawButton(2);
	}
	
	public boolean getElevatorDown() {
		return operatorStick.getRawButton(1);
	}
	
	public boolean getHookDeployUp() {
		return operatorStick.getRawButton(6);
	}
	
	public boolean getHookDeployDown() {
		return operatorStick.getRawButton(5);
	}
	
	public boolean getClimbWinchUp() {
		return operatorStick.getRawAxis(1) > 0.0;
	}
	
	public boolean getClimbWinchDown() {
		return operatorStick.getRawAxis(1) < 0.0;
	}
	
	public boolean getIntakeUp() {
		return operatorStick.getRawAxis(2) >= 0.5;
	}
	
	public boolean getIntakeDown() {
		return operatorStick.getRawAxis(3) >= 0.5;
	}
}
