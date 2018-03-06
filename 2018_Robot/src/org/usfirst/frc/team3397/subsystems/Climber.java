package org.usfirst.frc.team3397.subsystems;

import org.usfirst.frc.team3397.robot.OI;
import org.usfirst.frc.team3397.robot.Config;

import edu.wpi.first.wpilibj.Victor;

public class Climber {
	
//	RobotLayout rLayout;
	OI controlScheme;
	Config config = new Config();
	
	public Victor hookDeployMotor;
	public Victor climberWinchMotor;
	
	double hookDeploySpeed;
	double climberWinchSpeed;
	
	boolean hookDeployUp;
	boolean hookDeployDn;
	boolean climberWinchUp;
	boolean climberWinchDn;
	boolean climberWinchMoving;
	
	public Climber() {
		config.determineConfig();
		
		controlScheme = new OI(0, 1);
		
		hookDeployMotor = new Victor(7);
		climberWinchMotor = new Victor(5);
		
		hookDeploySpeed = 0.7;
		climberWinchSpeed = 0.95;
		
		hookDeployUp = controlScheme.getHookDeployUp();
		hookDeployDn = controlScheme.getHookDeployDown();
		climberWinchUp = controlScheme.getClimbWinchUp();
		climberWinchDn = controlScheme.getClimbWinchDown();
		
		climberWinchMoving = false;
	}
	
	public void hookDeploy() {
		if (controlScheme.getHookDeployUp()) {
			hookDeployMotor.set(-hookDeploySpeed);
			climberWinchMotor.set(-0.44);
			climberWinchMoving = true;
		}
		else if (controlScheme.getHookDeployDown()) {
			hookDeployMotor.set(hookDeploySpeed);
			climberWinchMotor.set(0.4);
			climberWinchMoving = true;
		}
		else
		{
			hookDeployMotor.set(0.0);
			climberWinchMotor.set(0.0);
			climberWinchMoving = false;
		}
	}
	
	public void useClimberWinch() {
		if (climberWinchMoving == false) {
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
}
