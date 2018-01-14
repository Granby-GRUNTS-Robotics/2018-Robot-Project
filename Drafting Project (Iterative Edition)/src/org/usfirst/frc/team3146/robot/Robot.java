package org.usfirst.frc.team3146.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	//Define channels for drive motors 
	final int kLeftChannel = 0;
	final int kRightChannel = 1;
	
	//Define channels for the joy sticks
	final int JoystickChannel = 0;
	
	//Define the actual joy sticks 
	Joystick stick1 = new Joystick(JoystickChannel);
	
	//Define the robots drive train as "robotDrive"
	RobotDrive robotDrive;
	
	//Define the timer variable
	Timer timer = new Timer();
	
	//Starting the "Robot" function in order to define drive train and motor inversions
	public Robot() {
		//Robot drive
		robotDrive = new RobotDrive(kLeftChannel, kRightChannel);
		
		//Motor inversions (only uncomment if necessary)
		//robotDrive.setInvertedMotor(kLeftChannel, true);
		//robotDrive.setInvertedMotor(kRightChannel, true);
		
	}
	
	// Define strings that are associated with different autonomous modes
	final String defaultAuto = "Cross Baseline";
	final String customAuto = "Single Switch Placement";
	final String customAuto2 = "Double Switch Placement";
	String autoSelected;
	
	//Define "chooser" object
	SendableChooser<String> chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// add auto options
		chooser.addDefault("Cross Baseline", defaultAuto);
		chooser.addObject("Single Switch Placement", customAuto);
		chooser.addObject("Double Switch Placement", customAuto2);
		
		//Publishes auto selector to smartdashboard
		SmartDashboard.putData("Auto choices", chooser);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		
		//Reset and start the timer
		timer.reset();
		timer.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case customAuto: // Only attempts to place a single power cube
			if(timer.get() < 2.0) {
				robotDrive.drive(.5, 0);// A sample of how you should time out functions during auto
			} else {
				robotDrive.drive(0, 0); // Stops the robot by setting motor speed to zero
			}
			break;
			
		case customAuto2: // Places one power cube and attempts to place another
			if(timer.get() < 2.0) {
				robotDrive.drive(.2, 0); // A sample of how you should time out functions during auto
			} else {
				robotDrive.drive(0, 0); // Stops the robot by setting motor speed to zero
			}  
			break;
			
		case defaultAuto:
		default: // Simply crosses the baseline
			if(timer.get() < 2.0) {
				robotDrive.drive(.5, 0); // A sample of how you should time out functions during auto
			}else {
				robotDrive.drive(0, 0); // Stops the robot by setting motor speed to zero
			}
			break;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		robotDrive.setSafetyEnabled(true); 
		while (isOperatorControl() && isEnabled()) { // Ensures that robot is enabled and in Teleoperated mode
			
			
			
			Timer.delay(.005); //Delays Cycles in order to avoid undue CPU usage
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}

