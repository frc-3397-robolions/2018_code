package org.usfirst.frc.team3397.subsystems;

import org.usfirst.frc.team3397.robot.OI;

import edu.wpi.first.wpilibj.Victor;

public class Climber {
	
//	RobotLayout rLayout;
	OI controlScheme;
	
	public Victor hookDeployMotor;
	public Victor climberWinchMotor;
	
	double hookDeploySpeed;
	double climberWinchSpeed;
	
	boolean hookDeployUp;
	boolean hookDeployDn;
	boolean climberWinchUp;
	boolean climberWinchDn;
	
	public Climber() {
//		rLayout = new RobotLayout();
		controlScheme = new OI(0, 1);
		
		hookDeployMotor = new Victor(5);
		climberWinchMotor = new Victor(6);
		
		hookDeploySpeed = 0.5;
		climberWinchSpeed = 0.5;
		
		hookDeployUp = controlScheme.getHookDeployUp();
		hookDeployDn = controlScheme.getHookDeployDown();
		climberWinchUp = controlScheme.getClimbWinchUp();
		climberWinchDn = controlScheme.getClimbWinchDown();
	}
	
	public void hookDeploy() {
		if (controlScheme.getHookDeployUp()) {
			hookDeployMotor.set(-hookDeploySpeed);
		}
		else if (controlScheme.getHookDeployDown()) {
			hookDeployMotor.set(hookDeploySpeed);
		}
		else
		{
			hookDeployMotor.set(0.0);
		}
	}
	
	public void useClimberWinch() {
		if (controlScheme.getClimbWinchUp()) {
			climberWinchMotor.set(climberWinchSpeed);
		}
		else if (controlScheme.getClimbWinchDown()) {
			climberWinchMotor.set(-climberWinchSpeed);
		}
		else
		{
			climberWinchMotor.set(0.0);
		}
	}
}
