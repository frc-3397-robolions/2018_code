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
import edu.wpi.first.wpilibj.AnalogInput;

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
    
    public final double INTAKE_DEPLOY_TIME = 0.5;
	public final double TIME_A = 0.5;
	public final double TIME_B_L = 2.2;
	public final double TIME_B_R = 2.4;
	public final double TIME_C = 5.8;
	public final double TIME_SPIT = 0.75;
	public final double TIME_TURN = 3.45; 	// This variable represents the time it takes to complete
											// the first two parts of the program
	public final double TIME_D = 7.6;
	
	public final double RIGHT_ANGLE = 29.93716257;
	public final double LEFT_ANGLE = -34.1778169;
//	public final double ANGLE_ERROR = 0.271778169;
	public final double ANGLE_ERROR = 1;
	
	public final double SIDE_TIME_A = 2.5;
	public final double SIDE_TIME_WAIT = 2.5;
	public final double RIGHT_SIDE_ANGLE = -90.0;
	public final double LEFT_SIDE_ANGLE = 90.0;
	public final double SIDE_TURN_TIME = 3.0;
	public final double SIDE_TIME_C = 3.0;
	
	public AnalogInput elevatorPot;
	double elevatorPos;
	
	public double AUTO_COUNTER = 0;
	public double AUTO_COUNTER_2 = 0;
	
	public double AUTO_ROTATION_RATE = 0.3;
	public double AUTO_ROTATE_RATE = 0.3;
	public double ROTATION_MULTIPLIER = 0.95;
	
	public boolean STAGE_1 = true;
	public boolean STAGE_2 = false;
	public boolean SET_STAGE_2 = true;
	public boolean STAGE_3 = false;
	public boolean STAGE_4 = false;
	public boolean STAGE_5 = false;
	public boolean DIRECTION = false; // false = left; true = right
	public boolean DIRECTION_2 = true;
	
	enum IntakeEnum {PUSH, PULL};
    
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
	Config config = new Config();
	
	PIDController turnController;
	
//	RobotLayout rLayout = new RobotLayout();
	
//	Victor frontLeftMotor = rLayout.frontLeftMotor;
//	Victor frontRightMotor = rLayout.frontRightMotor;
//	Victor backLeftMotor = rLayout.backLeftMotor;
//	Victor backRightMotor = rLayout.backRightMotor;
	
	//Drive motors
	
	public Victor frontLeftMotor;
	public Victor frontRightMotor;
	public Victor backLeftMotor;
	public Victor backRightMotor;
	
	// Operation motors
	
	Victor elevatorMotor;
	Victor hookDeploy;
	Victor climberWinch;
	
	// Note: the following motors are not on the robot yet, there are here solely for placeholders
	
	Victor intakeDeploy;
	Victor leftIntake;
	Victor rightIntake;
	
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
		
		config.determineConfig();
		
		frontLeftMotor = new Victor(3);
		frontRightMotor = new Victor(2);
		backLeftMotor = new Victor(1);
		backRightMotor = new Victor(0);
		
		elevatorMotor = elevator.elevatorMotor;
		hookDeploy = climber.hookDeployMotor;
		climberWinch = climber.climberWinchMotor;
		
		intakeDeploy = intake.intakeLift;
		leftIntake = intake.leftIntake;
		rightIntake = intake.rightIntake;
		
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
		
		config.determineConfig();
		
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
		AUTO_ROTATE_RATE = 0.3;
		AUTO_COUNTER = 0;
		AUTO_COUNTER_2 = 0;
		
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
		
		double angleDiffLeftAbs = Math.abs(LEFT_ANGLE) - Math.abs(ahrs.getAngle());
		double angleDiffRightAbs = Math.abs(RIGHT_ANGLE) - Math.abs(ahrs.getAngle());
		
		double angleDiff2 = 0 - ahrs.getAngle();
		
		double angleDiffLeft = LEFT_ANGLE - ahrs.getAngle();
		double angleDiffRight = RIGHT_ANGLE - ahrs.getAngle();
		
		SmartDashboard.putNumber("Current Angle", ahrs.getAngle());
		SmartDashboard.putNumber("Left Angle Diff", angleDiffLeft);
				
		switch (m_autoSelected) {
			case kMiddleAuto:
			default:
				logger.info("Running middle auto");
				logger.info(autoSide);
				if (autoSide == "left") {
					logger.info("Turning left");
					if (time.get() < TIME_A) {
						robotDrive.angleDrive(0.5, 0.0);
						intakeDeploy.set(-0.65);
					}
					else {
						robotDrive.stop();
						if (SET_STAGE_2) {
							STAGE_2 = true;
						}
						intakeDeploy.set(0.05);
					}
					
					if (STAGE_2) {
						logger.info("Entering stage 2");
						turnController.enable();
						turnController.setSetpoint(LEFT_ANGLE);
						if (time.get() >= TIME_TURN) {
							SET_STAGE_2 = false;
							STAGE_2 = false;
							STAGE_3 = true;
						}
						
						else if (Math.abs(angleDiffLeftAbs) <= ANGLE_ERROR) {
							robotDrive.Turn(0, ahrs.getAngle());
							logger.info("7676874534537534");
							STAGE_2 = false;
							if (AUTO_COUNTER >= 30) {
								SET_STAGE_2 = false;
								STAGE_3 = true;
							}

							logger.info("Turning has ended!");
							AUTO_COUNTER++;
						}
						else if (ahrs.getAngle() < LEFT_ANGLE) {
							robotDrive.Turn(AUTO_ROTATION_RATE, ahrs.getAngle());
//							robotDrive.Turn(0.3);
							logger.info("STAGE_2 TO THE LEFT!");
							if (DIRECTION) {
								DIRECTION = false;
								AUTO_ROTATION_RATE *= ROTATION_MULTIPLIER;
							}
						}
						else if (ahrs.getAngle() > LEFT_ANGLE) {
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
						if (time.get() <= (TIME_TURN + TIME_B_L)) {
							elevator.setPos(0.8);
							double turnInput = angleDiffLeft * 0.05;
							if (turnInput > 0.3) {
								turnInput = 0.3;
							}
							else if (turnInput < -0.3) {
								turnInput = -0.3;
							}
							
							SmartDashboard.putNumber("Turn Input in auto", turnInput);
							
							robotDrive.angleDrive(0.8, LEFT_ANGLE);
						}
						else
						{
							robotDrive.Drive(0.0);
							STAGE_4 = true;
							STAGE_3 = false;
							elevatorMotor.set(-0.09);
						}
						
					}
					else if (STAGE_4) {
						logger.info("Entering stage 2");
						turnController.enable();
						turnController.setSetpoint(LEFT_ANGLE);
						if (time.get() >= TIME_D) {
							STAGE_4 = false;
							STAGE_5 = true;
						}
						
						else if (Math.abs(angleDiff2) <= ANGLE_ERROR) {
							robotDrive.Turn(0, ahrs.getAngle());
							logger.info("7676874534537534");
							if (AUTO_COUNTER_2 >= 30) {
								STAGE_4 = false;
								STAGE_5 = true;
							}

							logger.info("Turning has ended!");
							AUTO_COUNTER_2++;
						}
						else if (ahrs.getAngle() > 0) {
							robotDrive.Turn(-AUTO_ROTATION_RATE, ahrs.getAngle());;
//							robotDrive.Turn(0.3);
							logger.info("STAGE_2 TO THE LEFT!");
							if (!DIRECTION) {
								DIRECTION = false;
								AUTO_ROTATE_RATE *= ROTATION_MULTIPLIER;
							}
						}
						else if (ahrs.getAngle() < 0) {
							robotDrive.Turn(AUTO_ROTATION_RATE, ahrs.getAngle());
							if (DIRECTION) {
								DIRECTION = true;
								AUTO_ROTATE_RATE *= ROTATION_MULTIPLIER;
							}
						}
						logger.info("Turning is still true");
					}
					else if (STAGE_5) {
						if (time.get() > TIME_D && time.get() <= (TIME_D + TIME_SPIT)) {
							intake.setIntake(true, 0.5);
							robotDrive.Drive(0.15);
						}
						else
						{
							intake.setIntake(true, 0.0);
							robotDrive.Drive(0.0);
						}
					}
					
				}
				else if (autoSide == "right") {
					logger.info("Turning right");
					if (time.get() < TIME_A) {
						robotDrive.Drive(0.5);
						intakeDeploy.set(-0.65);
					}
					else {
						robotDrive.Drive(0.0);
						intakeDeploy.set(0.05);
						if (SET_STAGE_2) {
							STAGE_2 = true;
						}
					}
					
					if (STAGE_2) {
						logger.info("Entering stage 2");
						turnController.enable();
						turnController.setSetpoint(RIGHT_ANGLE);
						
						if (time.get() >= TIME_TURN) {
							SET_STAGE_2 = false;
							STAGE_2 = false;
							STAGE_3 = true;
						}
						else if (Math.abs(angleDiffRightAbs) <= ANGLE_ERROR) {
							robotDrive.Turn(0, ahrs.getAngle());
							logger.info("7676874534537534");
							STAGE_2 = false;
							if (AUTO_COUNTER >= 50) {
								SET_STAGE_2 = false;
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
							elevator.setPos(0.8);
							double turnInput = angleDiffRight * 0.05;
							if (turnInput > 0.3) {
								turnInput = 0.3;
							}
							else if (turnInput < -0.3) {
								turnInput = -0.3;
							}
							
							SmartDashboard.putNumber("Turn Input in auto", turnInput);
							
							robotDrive.driveCart(0.5, 0.0, turnInput);

						}
						else
						{
							robotDrive.Drive(0.0);
							STAGE_3 = false;
							STAGE_4 = true;
						}
					}
					else if (STAGE_4) {
						logger.info("Entering stage 4");
						turnController.enable();
						turnController.setSetpoint(RIGHT_ANGLE);
						if (time.get() >= TIME_D) {
							STAGE_4 = false;
							STAGE_5 = true;
						}
						
						else if (angleDiff2 <= ANGLE_ERROR) {
							robotDrive.Turn(0, ahrs.getAngle());
							logger.info("7676874534537534");
							if (AUTO_COUNTER >= 30) {
								STAGE_4 = false;
								STAGE_5 = true;
							}

							logger.info("Turning has ended!");
							AUTO_COUNTER++;
						}
						else if (ahrs.getAngle() > 0) {
							robotDrive.Turn(-AUTO_ROTATION_RATE, ahrs.getAngle());
//							robotDrive.Turn(0.3);
							logger.info("STAGE_2 TO THE LEFT!");
							if (!DIRECTION) {
								DIRECTION = false;
								AUTO_ROTATE_RATE *= ROTATION_MULTIPLIER;
							}
						}
						else if (ahrs.getAngle() < 0) {
							robotDrive.Turn(AUTO_ROTATION_RATE,  ahrs.getAngle());
							if (DIRECTION) {
								DIRECTION = true;
								AUTO_ROTATE_RATE *= ROTATION_MULTIPLIER;
							}
						}
						logger.info("Turning is still true");
					}
					else if (STAGE_5) {
						if (time.get() > TIME_D && time.get() <= (TIME_D + TIME_SPIT)) {
							intake.setIntake(true, 0.5);
							robotDrive.Drive(0.15);
						}
						else
						{
							intake.setIntake(true, 0.0);
							robotDrive.Drive(0.0);
						}
					}
				}
				break;
		case kLeftAuto:
			double angleDiff_leftSide = Math.abs(LEFT_SIDE_ANGLE) - Math.abs(ahrs.getAngle());
			if (STAGE_1) {
				if (time.get() <= SIDE_TIME_A) {
					double turnInput = -ahrs.getAngle() * 0.05;
					if (turnInput > 0.3) {
						turnInput = 0.3;                                      
					}
					else if (turnInput < -0.3) {
						turnInput = -0.3;
					}
					
					SmartDashboard.putNumber("Turn Input in auto", turnInput);
					
					robotDrive.driveCart(0.8, 0.0, turnInput);
					
//					if (autoSide == "left") {
						elevator.setPos(1.0);
						
						if (time.get() <= INTAKE_DEPLOY_TIME) {
							intakeDeploy.set(-0.6);
						}
						else {
							intakeDeploy.set(0.0);
						}
//					}
				}
				else
				{
					robotDrive.Drive(0.0);
					if (autoSide == "left") {
						STAGE_1 = false;
						STAGE_4 = true;
					}
				}
			}
			else if (STAGE_2) {
				if (autoSide == "left") {
					if (time.get() <= SIDE_TURN_TIME) {
						if (Math.abs(angleDiff_leftSide) <= ANGLE_ERROR) {
							robotDrive.Turn(0, ahrs.getAngle());
							logger.info("7676874534537534");
							if (AUTO_COUNTER >= 30) {
								STAGE_2 = false;
								STAGE_3 = true;
							}
	
							logger.info("Turning has ended!");
							AUTO_COUNTER++;
						}
						else if (ahrs.getAngle() < LEFT_SIDE_ANGLE) {
							robotDrive.Turn(AUTO_ROTATION_RATE, ahrs.getAngle());
							if (DIRECTION) {
								DIRECTION = false;
								AUTO_ROTATION_RATE *= ROTATION_MULTIPLIER;
							}
						}
						else if (ahrs.getAngle() > LEFT_SIDE_ANGLE) {
							robotDrive.Turn(-AUTO_ROTATION_RATE,  ahrs.getAngle());
							if (!DIRECTION) {
								DIRECTION = true;
								AUTO_ROTATION_RATE *= ROTATION_MULTIPLIER;
							}
						}
						logger.info("Turning is still true");
					}
					else
					{
						STAGE_2 = false;
						STAGE_3 = true;
					}
					
				}
			}
			else if (STAGE_3) {
				if (autoSide == "left") {
					if (time.get() > SIDE_TURN_TIME && time.get() < SIDE_TIME_C) {
						double turnInput = -ahrs.getAngle() * 0.05;
						if (turnInput > 0.3) {
							turnInput = 0.3;                                      
						}
						else if (turnInput < -0.3) {
							turnInput = -0.3;
						}
						
						SmartDashboard.putNumber("Turn Input in auto", turnInput);
						
						robotDrive.driveCart(0.8, 0.0, turnInput);
					}
					else
					{
						STAGE_3 = false;
						STAGE_4 = true;
					}
				}
			}
			else if (STAGE_4) {
				if (autoSide == "left") {
					if (time.get() > SIDE_TIME_C && time.get() < SIDE_TIME_C + TIME_SPIT) {
						intake.setIntake(true, 1.0);
					}
					else {
						intake.setIntake(true, 0.0);
					}
				}
			}
			logger.info("Running left auto");
			break;
		case kRightAuto:
			double angleDiff_rightSide = Math.abs(LEFT_SIDE_ANGLE) - Math.abs(ahrs.getAngle());
			if (STAGE_1) {
				if (time.get() <= SIDE_TIME_A) {
					double turnInput = -ahrs.getAngle() * 0.05;
					if (turnInput > 0.3) {
						turnInput = 0.3;                                      
					}
					else if (turnInput < -0.3) {
						turnInput = -0.3;
					}
					
					SmartDashboard.putNumber("Turn Input in auto", turnInput);
					
					robotDrive.driveCart(0.4, 0.0, turnInput);
					
//					if (autoSide == "right") {
						elevator.setPos(1.0);
						
						if (time.get() <= INTAKE_DEPLOY_TIME) {
							intakeDeploy.set(-0.6);
						}
						else {
							intakeDeploy.set(0.05);
//						}
					}
				}
				else
				{
					robotDrive.Drive(0.0);
					if (autoSide == "right") {
						STAGE_1 = false;
						STAGE_4 = true;
					}
					elevatorMotor.set(-0.09);
				}
			}
			else if (STAGE_2) {
				if (autoSide == "right") {
					if (time.get() <= SIDE_TURN_TIME) {
						if (Math.abs(angleDiff_rightSide) <= ANGLE_ERROR) {
							robotDrive.Turn(0, ahrs.getAngle());
							logger.info("7676874534537534");
							if (AUTO_COUNTER >= 30) {
								STAGE_2 = false;
								STAGE_3 = true;
							}
	
							logger.info("Turning has ended!");
							AUTO_COUNTER++;
						}
						else if (ahrs.getAngle() < RIGHT_SIDE_ANGLE) {
							robotDrive.Turn(AUTO_ROTATION_RATE, ahrs.getAngle());
							if (DIRECTION) {
								DIRECTION = false;
								AUTO_ROTATION_RATE *= ROTATION_MULTIPLIER;
							}
						}
						else if (ahrs.getAngle() > RIGHT_SIDE_ANGLE) {
							robotDrive.Turn(-AUTO_ROTATION_RATE,  ahrs.getAngle());
							if (!DIRECTION) {
								DIRECTION = true;
								AUTO_ROTATION_RATE *= ROTATION_MULTIPLIER;
							}
						}
						logger.info("Turning is still true");
					}
					else
					{
						STAGE_2 = false;
						STAGE_3 = true;
					}
				}
			}
			else if (STAGE_3) {
				if (autoSide == "right") {
					if (time.get() > SIDE_TURN_TIME && time.get() < SIDE_TIME_C) {
						double turnInput = -ahrs.getAngle() * 0.05;
						if (turnInput > 0.3) {
							turnInput = 0.3;                                      
						}
						else if (turnInput < -0.3) {
							turnInput = -0.3;
						}
						
						SmartDashboard.putNumber("Turn Input in auto", turnInput);
						
						robotDrive.driveCart(0.8, 0.0, turnInput);
					}
					else
					{
						STAGE_3 = false;
						STAGE_4 = true;
					}
				}
			}
			else if (STAGE_4) {
				if (autoSide == "right") {
					if (time.get() > SIDE_TURN_TIME && time.get() < SIDE_TURN_TIME + TIME_SPIT) {
						intake.setIntake(true, 1.0);
					}
					else {
						intake.setIntake(true, 0.0);
					}
				}
			}
			logger.info("Running left auto");
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
		
		ahrs.reset();
		
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
		
		double angleDiffLeft = LEFT_ANGLE - ahrs.getAngle();
		
		SmartDashboard.putNumber("Left Angle Difference", angleDiffLeft);
		
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
		
		if (controlScheme.test()) {
			elevator.setPos(1.0);
		}
		
		if (controlScheme.getX()) {
			double turnInput = angleDiffLeft * 0.1;
			if (turnInput > 0.3) {
				turnInput = 0.3;
			}
			else if (turnInput < -0.3) {
				turnInput = -0.3;
			}
			
			SmartDashboard.putNumber("Turn Input in auto", turnInput);
			
			robotDrive.driveCart(0.5, 0.0, turnInput);
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

