package org.usfirst.frc.team3146.robot;

import com.ctre.CANTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	//Initialize the string for storing field data
	String gameData;
	
	//Define channels for drive motors 
	final int kLeftChannel = 1;
	final int kRightChannel = 2;
	
	//Define channels for slave motors
	final int left_slave_channel = 0;
	final int right_slave_channel = 3;
	
	//Intitialize the Victor SPX motor controls
	WPI_VictorSPX motor1 = new WPI_VictorSPX(kLeftChannel);
	WPI_VictorSPX motor2 = new WPI_VictorSPX(kRightChannel);
	
	//Initialize slave motors
	WPI_VictorSPX left_slave_motor = new WPI_VictorSPX(left_slave_channel);
	WPI_VictorSPX right_slave_motor = new WPI_VictorSPX(right_slave_channel);
	
	//Initialize spark motor controllers for grabber wheel motors
	Spark grasping_motor_left = new Spark(0);
	Spark grasping_motor_right = new Spark(1);
	
	//Initialize pnuematic gripper
	Solenoid grip0 = new Solenoid(1, 0);
	Solenoid grip1 = new Solenoid(1, 1);
	
	//Create empty storage values
	double initial_value;
	
	
	//Initialize the talons used for testing with the breadboard, leave this commented out
	//CANTalon talon1 = new CANTalon(2);
	//CANTalon talon2 = new CANTalon(3);
	
	//Initialize the Gyro/magnetometer "pigeon"
	PigeonIMU pigeon = new PigeonIMU(0); 
	
	//Define channels for the joy sticks
	final int JoystickChannel = 0;
	
	//Define the actual joy sticks 
	Joystick stick1 = new Joystick(0);
	Joystick stick2 = new Joystick(1);
	//Define the robots drive train as "robotDrive"
	RobotDrive robotDrive;
	
	//Define the timer variable
	Timer timer = new Timer();
	

	//Modify variables for the smart dashboard
	//Preferences pref; //sets preference 
	//double auto_delay_value;
	
	
	//Custom functions 
	
	//Function that allows the robot to turn to a very specific degree
	public void auto_degree_turn( double turn_degree){
			if((initial_value - pigeon.getFusedHeading()) < (turn_degree - 1)){
				robotDrive.drive(( 1 / (initial_value - pigeon.getFusedHeading())) + .25, 1);
			}else if((initial_value - pigeon.getFusedHeading()) > (turn_degree + 1)){
				robotDrive.drive(( 1 / (initial_value - pigeon.getFusedHeading())) + .25, -1);
			}else{
				robotDrive.drive(0, 0);
			}
	}
		
	//Allows the robot to drive with complete directional compensation
	//Takes 3 arguments, (initial_value, speed, and curve).
	//Initial value = the magnetometers zero value
	//Speed = The motor output
	//Curve = The Gradual turn of the robot (1 is a zero point turn)
	
	public void drive_drift_compensation( double initial_value, double speed, double curve, double start_angle) {
		if((pigeon.getFusedHeading() - (initial_value - start_angle)) > 1) { //Will fix the robots orientation in the case that it drifts, or is hit.
			robotDrive.drive(speed, curve); //Turn at .3 speed in order to compensate
		}else if((initial_value - start_angle) - pigeon.getFusedHeading() > 1) { //Will fix the robots orientation in the case that it drifts, or is hit.
			robotDrive.drive(speed, -curve); ////Turn at .3 speed in order to compensate
		}else{
			robotDrive.drive(speed, 0);//Keep driving if nothing is thrown off.
		}
	}
	//Starting the "Robot" function in order to define drive train and motor inversions
	public Robot() {
		//Robot drive
		robotDrive = new RobotDrive(motor1, motor2);
		
		//Motor inversions (only uncomment if necessary)
		//robotDrive.setInvertedMotor(motor1, true);
		//robotDrive.setInvertedMotor(motor2, true);
		
		//motor2.setInverted(true);
		//motor1.setInverted(true);
		
		//Swap the axis channels
		
	}
	
	// Define strings that are associated with different autonomous modes
	final String defaultAuto = "Cross Baseline";
	final String Single_Placement = "Single Switch Placement";
	final String Double_Placement = "Double Switch Placement";
	String autoSelected;
	

	//define string associated with different stations
	final String left = "Left Station";
	final String middle = "Middle Station";
	final String right = "Right Station";
	String stationSelected;
	
	//Define "chooser" object for auto selector

	//Define "chooser" object for the smartdashboard

	SendableChooser<String> chooser = new SendableChooser<>();

	//Define "station_chooser" object for station selector
	SendableChooser<String> station_chooser = new SendableChooser<>();
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		//Retrieve the initial Magnetometer value for teleop purposes
		initial_value = pigeon.getFusedHeading();

		// add auto options
		chooser.addDefault("Cross Baseline", defaultAuto);
		chooser.addObject("Single Switch Placement", Single_Placement);
		chooser.addObject("Double Switch Placement", Double_Placement);
	
		//add station options
		station_chooser.addDefault("Left Station", left);
		station_chooser.addObject("Middle Station", middle);
		station_chooser.addObject("Right Station", right);
	
		//Publishes auto selector to smartdashboard
		SmartDashboard.putData("Auto choices", chooser);
	
		//Publishes station selector to SmartDashboard
		SmartDashboard.putData("Driver station", station_chooser);
		
		//Retrieve information from the control system
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		//Set the other two Victor motor controllers as followers to the main two
		right_slave_motor.follow(motor2);
		left_slave_motor.follow(motor1);
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
	//Retrieve the initial magnetometer value for autonomous purposes
	initial_value = pigeon.getFusedHeading();	
	
	autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		
	stationSelected = station_chooser.getSelected();
		//Reset and start the timer
		timer.reset();
		timer.start(); 
		
		//Retreive information from the control system
		gameData = DriverStation.getInstance().getGameSpecificMessage();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		//Check and execute the delay variable for competition
		//Timer.delay(auto_delay_value);
		switch (autoSelected) {
		case Single_Placement: // Only attempts to place a single power cube
			switch (stationSelected) {
				case left:
					if(gameData.charAt(0) == 'L') {
					    if(timer.get() < 3){
					    	drive_drift_compensation(initial_value, .3, .3, 0);
						}else if(timer.get() < 5){
							auto_degree_turn(90);
						}else if(timer.get() < 7){
							drive_drift_compensation(initial_value, .3, .3, 90);
						}else if (timer.get() < 7.25){
							drive_drift_compensation(initial_value, -0.1, .3,  90);
						}else {
							drive_drift_compensation(initial_value, 0, 0, 0);
						}
					}else {
						if(timer.get() < 2) {
							drive_drift_compensation(initial_value, .3, .3, 0);
						}else if(timer.get() < 4){
							auto_degree_turn(90);
						}else if(timer.get() < 9){
							drive_drift_compensation(initial_value, .3, .3, 90);
						}else if(timer.get() < 11){
							auto_degree_turn(0);
						}else if(timer.get() < 12){
							drive_drift_compensation(initial_value, .3, .3, 0);
						}else if(timer.get() < 14){
							auto_degree_turn(-90);
						}else if(timer.get() < 15) {
							drive_drift_compensation(initial_value, .3, .3, -90);
						}else if(timer.get() < 16) {
							drive_drift_compensation(initial_value, -.3, .3, -90);
						}else {
							drive_drift_compensation(initial_value, 0, 0, 0);
						}
					}
				case middle:
					if(gameData.charAt(0) == 'L') {
						if(timer.get() < 2) {
							drive_drift_compensation(initial_value, .3, .3, 0);
						}else if (timer.get() < 4) {
							auto_degree_turn(-90);
						}else if(timer.get() < 7) {
							drive_drift_compensation(initial_value, .3, .3, -90);
						}else if(timer.get() < 9) {
							auto_degree_turn(0);
						}else if(timer.get() < 11) {
							drive_drift_compensation(initial_value, .3, .3, 0);
						}else if(timer.get() < 13) {
							auto_degree_turn(90);
						}
					}else {
						
					}
				case right:
					if(gameData.charAt(0) == 'L') {
						
					}else {
						
					}
			}
			break;
		case Double_Placement: // Places one power cube and attempts to place another
				if(timer.get() < 2.0) {
					robotDrive.drive(.2, 0); // A sample of how you should time out functions during auto
				} else {
					robotDrive.drive(0, 0); // Stops the robot by setting motor speed to zero
				}  
				break;
			
		case defaultAuto:
			default: // Simply crosses the baseline
				switch (stationSelected) {
				case left:
					if(timer.get() < 5) {
						drive_drift_compensation(initial_value, .2, .3, 0);
					}else{
						robotDrive.drive(0, 0); // Stops the robot by setting motor speed to zero
					}
					
				case right:
					if(timer.get() < 5) {
						drive_drift_compensation(initial_value, .2, .3, 0);
					}else{
						robotDrive.drive(0, 0); // Stops the robot by setting motor speed to zero
					}
				case middle:
					
				}
				break;
		}
	//publish the base magnetometer value to the dashboard
		SmartDashboard.putNumber("Compass Variance", (initial_value - pigeon.getFusedHeading()));

		//publish the base magnetometer value to the dashboard
		SmartDashboard.putNumber("Compass Variance", (initial_value - pigeon.getFusedHeading()));
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		robotDrive.setSafetyEnabled(true); 
		while (isOperatorControl() && isEnabled()) { // Ensures that robot is enabled and in Teleoperated mode
			//Smooth drive code (with tangent)
			double slow_val_z = (stick1.getZ() / -1.5); //inverts and reduces x value
			double slow_val_y = (Math.tan(stick1.getY()) * -.5); //reduces y value on a tangent curve
			
			//Drive functions
			if(stick1.getRawButton(1)) {
				robotDrive.arcadeDrive(slow_val_y, slow_val_z); //smooth drive when top trigger is pressed
			}else{
				robotDrive.arcadeDrive((stick1.getY() * -1), (stick1.getZ() * -1)); //normal drive
			}
			
			//Map buttons to proper gripper functions
			if(stick2.getRawButton(10)) { 
				//opens the pnuematic arms on the robot
				grip0.set(true);
				grip1.set(false);
			}
			if(stick2.getRawButton(14)) { 
				//Closes the pnuematic arms on the robot
				grip0.set(false);
				grip1.set(true);
			}
			if(stick2.getRawButton(7)) { 
				//opens the pnuematic arms on the robot
				 grasping_motor_left.set(1);
				 grasping_motor_right.set(1);
			}
			if(stick2.getRawButton(8)) { 
				//Closes the pnuematic arms on the robot
				grasping_motor_left.set(-1);
				grasping_motor_right.set(-1);
			}
			if(stick2.getRawButton(8) == false && stick2.getRawButton(7) == false){ //If nothing is pressed
				grasping_motor_left.set(0);
				grasping_motor_right.set(0);
			}
			//Publish SmartDashboard values
			SmartDashboard.putBoolean("Smooth Drive", stick1.getRawButton(1));
			SmartDashboard.putNumber("Compass Variance", (initial_value - pigeon.getFusedHeading()));
			
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
