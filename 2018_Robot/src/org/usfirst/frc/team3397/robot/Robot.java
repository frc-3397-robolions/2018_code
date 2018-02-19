/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3397.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import org.opencv.core.Mat;
import edu.wpi.first.wpilibj.CameraServer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;

import java.util.logging.Logger;
import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc.team3397.subsystems.DriveTrain;
import org.usfirst.frc.team3397.subsystems.Elevator;
import org.usfirst.frc.team3397.subsystems.Climber;
import org.usfirst.frc.team3397.subsystems.Intake;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot implements PIDOutput {
	private static final String kMiddleAuto = "Middle Auto";
	private static final String kLeftAuto = "Left Auto";
	private static final String kRightAuto = "Right Auto";
	private String     m_autoSelected;
	private SendableChooser<String> auto_chooser = new SendableChooser<>();
	
	static final double kP = 0.03;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    
    static final double kToleranceDegrees = 2.0f;
    
	public final double TIME_A = 0.5;
	public final double TIME_B_L = 0.96;
	public final double TIME_B_R = 0.975;
	public final double TIME_C = 5.5;
	public final double TIME_SPIT = 0.75;
	public final double TIME_TURN = 3.35; 	// This variable represents the time it takes to complete
											// the first two parts of the program
	
	public final double RIGHT_ANGLE = 29.93716257;
	public final double LEFT_ANGLE = -26.1778169;
//	public final double ANGLE_ERROR = 0.271778169;
	public final double ANGLE_ERROR = 1;
	
	public double AUTO_COUNTER = 0;
	
	public double AUTO_ROTATION_RATE = 0.3;
	public double ROTATION_MULTIPLIER = 0.95;
	
	public boolean TURNING = false;
	public boolean SET_TURNING = true;
	public boolean STAGE_3 = false;
	public boolean STAGE_4 = false;
	public boolean DIRECTION = false; // false = left; true = right
    
    double rotateToAngleRate;
	
	public String autoSide;
	
//	private static final String kAutoValueKey = "kAutoValue";
//	private static final Sendable kAutoValue = 0;
	
	AHRS ahrs;
	DriveTrain robotDrive;
	Elevator elevator = new Elevator();
	Climber climber = new Climber();
	Intake intake = new Intake();
	OI controlScheme = new OI(0, 1);
	
	PIDController turnController;
	
//	RobotLayout rLayout = new RobotLayout();
	
//	Victor frontLeftMotor = rLayout.frontLeftMotor;
//	Victor frontRightMotor = rLayout.frontRightMotor;
//	Victor backLeftMotor = rLayout.backLeftMotor;
//	Victor backRightMotor = rLayout.backRightMotor;
	
	//Drive motors
	
	Victor frontLeftMotor = new Victor(0);
	Victor frontRightMotor = new Victor(1);
	Victor backLeftMotor = new Victor(2);
	Victor backRightMotor = new Victor(3);
	
	// Operation motors
	
	Victor elevatorMotor = elevator.elevatorMotor;
	Victor hookDeploy = climber.hookDeployMotor;
	Victor climberWinch = climber.climberWinchMotor;
	
	// Note: the following motors are not on the robot yet, there are here solely for placeholders
	
	Victor intakeDeploy = intake.intakeLift;
	Victor leftIntake = intake.leftIntake;
	Victor rightIntake = intake.rightIntake;
	
	// Vision //
	
	UsbCamera cam0;
	CameraServer camServer;
	
	DigitalInput elevatorStopTop = elevator.elevatorTopStop;
	
	Timer time;
	
	private static final Logger logger = Logger.getLogger(Robot.class.getName());
	

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		auto_chooser.addDefault("Middle Auto", kMiddleAuto);
		auto_chooser.addObject("Right Auto", kRightAuto);
		auto_chooser.addObject("Left Auto", kLeftAuto);
		SmartDashboard.putData("Auto Programs", auto_chooser);			//		SmartDashboard.putData(kAutoValueKey, null);
		
		robotDrive = new DriveTrain(frontLeftMotor, 
									frontRightMotor, 
									backLeftMotor, 
									backRightMotor
									);
		robotDrive.setExpiration(0.1);
//		elevator = new Elevator();
//		climber = new Climber();
		
		camServer = CameraServer.getInstance();
		cam0 = camServer.startAutomaticCapture(0);
		
		time = new Timer();
		logger.info("Robot initiated");
		
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
        turnController = new PIDController(kP, kI, kD, kF, ahrs, (PIDOutput) this);
        turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-1.0, 1.0);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
		
		
//		SmartDashboard.getData(kAutoValue);
		// Input for autonomous
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = auto_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
		
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		// Autonomous constants (time for each step, etc.)
		
		
		if (gameData.charAt(0) == 'L') {
			autoSide = "left";
		}
		else if (gameData.charAt(0) == 'R') {
			autoSide = "right";
		}
		
		ahrs.reset();
		AUTO_ROTATION_RATE = 0.3;
		AUTO_COUNTER = 0;
		
		cam0.setResolution(320, 240);
		cam0.setFPS(15);
		Mat image = new Mat();
		
		CvSink cvSink0 = camServer.getVideo(cam0);
		CvSource outputStream = camServer.putVideo("Main", 320, 240);
		
		cvSink0.setEnabled(true);
		cvSink0.grabFrame(image);
		outputStream.putFrame(image);
		
		time.stop(); time.start(); time.reset();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		double angleDiffLeft = Math.abs(LEFT_ANGLE) - Math.abs(ahrs.getAngle());
		double angleDiffRight = Math.abs(RIGHT_ANGLE) - Math.abs(ahrs.getAngle());
		
		SmartDashboard.putNumber("Current Angle", ahrs.getAngle());
				
		switch (m_autoSelected) {
			case kMiddleAuto:
			default:
				logger.info("Running middle auto");
				logger.info(autoSide);
				if (autoSide == "left") {
					logger.info("Turning left");
					if (time.get() < TIME_A) {
						robotDrive.Drive(0.5);
						intakeDeploy.set(0.6);
					}
					else {
						robotDrive.Drive(0.0);
						if (SET_TURNING) {
							TURNING = true;
						}
						intakeDeploy.set(0.0);
					}
					
					if (TURNING) {
						logger.info("Entering stage 2");
						turnController.enable();
						turnController.setSetpoint(LEFT_ANGLE);
						if (time.get() >= TIME_TURN) {
							SET_TURNING = false;
							TURNING = false;
							STAGE_3 = true;
						}
						
						else if (Math.abs(angleDiffLeft) <= ANGLE_ERROR) {
							robotDrive.Turn(0, ahrs.getAngle());
							logger.info("7676874534537534");
							TURNING = false;
							if (AUTO_COUNTER >= 30) {
								SET_TURNING = false;
								STAGE_3 = true;
							}

							logger.info("Turning has ended!");
							AUTO_COUNTER++;
						}
						else if (Math.abs(ahrs.getAngle()) < Math.abs(LEFT_ANGLE)) {
							robotDrive.Turn(-AUTO_ROTATION_RATE, ahrs.getAngle());
//							robotDrive.Turn(0.3);
							logger.info("TURNING TO THE LEFT!");
							if (DIRECTION) {
								DIRECTION = false;
								AUTO_ROTATION_RATE *= ROTATION_MULTIPLIER;
							}
						}
						else if (Math.abs(ahrs.getAngle()) > Math.abs(LEFT_ANGLE)) {
							robotDrive.Turn(AUTO_ROTATION_RATE,  ahrs.getAngle());
							if (!DIRECTION) {
								DIRECTION = true;
								AUTO_ROTATION_RATE *= ROTATION_MULTIPLIER;
							}
						}
						logger.info("Turning is still true");
					}
					else if (STAGE_3) {
						logger.info("STAGE 3 HAS BEGUN");
						turnController.disable();
						if (time.get() <= (TIME_TURN + TIME_B_L)) {
							robotDrive.Drive(0.8);
							if (elevatorStopTop.get()) {
								elevatorMotor.set(0.7);
								logger.info("ELEVATOR GOING UP");
							}
							else {
								elevatorMotor.set(0.0);
								logger.info("ELEVATOR NOT MOVING");
							}
						}
						else
						{
							robotDrive.Drive(0.0);
							STAGE_4 = true;
							STAGE_3 = false;
						}
						
					}
					else if (STAGE_4) {
						if (time.get() > TIME_C && time.get() <= (TIME_C + TIME_SPIT)) {
							rightIntake.set(1.0);
							leftIntake.set(-1.0);
						}
						else
						{
							rightIntake.set(0.0);
							leftIntake.set(0.0);
						}
					}
					
				}
				else if (autoSide == "right") {
					logger.info("Turning right");
					if (time.get() < TIME_A) {
						robotDrive.Drive(0.5);
						intakeDeploy.set(0.6);
					}
					else {
						robotDrive.Drive(0.0);
						intakeDeploy.set(0.0);
						if (SET_TURNING) {
							TURNING = true;
						}
					}
					
					if (TURNING) {
						logger.info("Entering stage 2");
						turnController.enable();
						turnController.setSetpoint(RIGHT_ANGLE);
						
						if (time.get() >= TIME_TURN) {
							SET_TURNING = false;
							TURNING = false;
							STAGE_3 = true;
						}
						else if (Math.abs(angleDiffRight) <= ANGLE_ERROR) {
							robotDrive.Turn(0, ahrs.getAngle());
							logger.info("7676874534537534");
							TURNING = false;
							if (AUTO_COUNTER >= 50) {
								SET_TURNING = false;
								STAGE_3 = true;
							}

							logger.info("Turning has ended!");
							AUTO_COUNTER++;
						}
						else if (Math.abs(ahrs.getAngle()) < Math.abs(RIGHT_ANGLE)) {
							robotDrive.Turn(AUTO_ROTATION_RATE, ahrs.getAngle());
//							robotDrive.Turn(0.3);
							logger.info("TURNING TO THE LEFT!");
							if (DIRECTION) {
								DIRECTION = false;
								AUTO_ROTATION_RATE *= ROTATION_MULTIPLIER;
							}
						}
						else if (Math.abs(ahrs.getAngle()) > Math.abs(RIGHT_ANGLE)) {
							robotDrive.Turn(-AUTO_ROTATION_RATE,  ahrs.getAngle());
							if (!DIRECTION) {
								DIRECTION = true;
								AUTO_ROTATION_RATE *= ROTATION_MULTIPLIER;
							}
						}
						logger.info("Turning is still true");
					}
					else if (STAGE_3) {
						logger.info("STAGE 3 HAS BEGUN");
						turnController.disable();
						if (time.get() <= (TIME_TURN + TIME_B_R)) {
							robotDrive.Drive(0.8);
							if (elevatorStopTop.get()) {
								elevatorMotor.set(0.7);
								logger.info("ELEVATOR GOING UP");
							}
							else {
								elevatorMotor.set(0.0);
								logger.info("ELEVATOR NOT MOVING");
							}
						}
						else
						{
							robotDrive.Drive(0.0);
							STAGE_3 = false;
							STAGE_4 = true;
						}
					}
					else if (STAGE_4) {
						if (time.get() > TIME_C && time.get() <= (TIME_C + TIME_SPIT)) {
							rightIntake.set(1.0);
							leftIntake.set(-1.0);
						}
						else
						{
							rightIntake.set(0.0);
							leftIntake.set(0.0);
						}
					}
				}
				break;
		case kLeftAuto:
			// Put custom auto code here
			logger.info("Running left auto");
			break;
		case kRightAuto:
			// Put default auto code here
			logger.info("Running right auto");
			
			break;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	
	public void teleopInit() {
		cam0.setResolution(320, 240);
		cam0.setFPS(15);
		Mat image = new Mat();
		
		CvSink cvSink0 = camServer.getVideo(cam0);
		CvSource outputStream = camServer.putVideo("Main", 320, 240);
		
		cvSink0.setEnabled(true);
		cvSink0.grabFrame(image);
		outputStream.putFrame(image);
		
		time.stop(); time.reset(); time.start();
	}
	@Override
	public void teleopPeriodic() {
		
		SmartDashboard.putNumber("Current Angle", ahrs.getAngle());
		
		elevator.useElevator();
		
		robotDrive.setSafetyEnabled(true);
		robotDrive.MecanumDrive();
		
		climber.hookDeploy();
		climber.useClimberWinch();
		
		intake.intakeLift();
		intake.runIntake();
		
		if (time.get() >= 105 && time.get() <= 110) {
			controlScheme.vibrate();
		}
		else
		{
			controlScheme.vibrateOff();
		}
		
		if (!elevatorStopTop.get()) {
			logger.info("ELEVATOR AT THE TOP");
		}
		else
		{
			logger.info("ELEVATOR NOT AT THE TOP");
		}
		
		logger.info("Teleop enabled");
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		
	}

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
	}
}

