package org.usfirst.frc.team3146.robot;

import com.ctre.CANTalon;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;


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
	final int kLiftMotorChannel = 5;
	
	//Define channels for slave motors
	final int left_slave_channel = 4;
	final int right_slave_channel = 3;
	
	//Initialize the encoders
	AnalogInput lift_measure = new AnalogInput(2);
	Encoder wheel_counter1 = new Encoder(1, 0, false, EncodingType.k4X);
	
	//Intitialize the Victor SPX motor controls
	WPI_VictorSPX motor1 = new WPI_VictorSPX(kLeftChannel);
	WPI_VictorSPX motor2 = new WPI_VictorSPX(kRightChannel);
	WPI_TalonSRX lift_screw_motor = new WPI_TalonSRX(kLiftMotorChannel);
	
	//Initialize slave motors
	WPI_VictorSPX left_slave_motor = new WPI_VictorSPX(left_slave_channel);
	WPI_VictorSPX right_slave_motor = new WPI_VictorSPX(right_slave_channel);
	
	//Initialize spark motor controllers for grabber wheel motors
	Spark grasping_motor_left = new Spark(0);
	Spark grasping_motor_right = new Spark(1);

	//Initialize motors for climbing
	VictorSP climber = new VictorSP(3);
	
	WPI_TalonSRX hook = new WPI_TalonSRX(3146); //this is a placeholder value until we can see the CAN bus on real bot
	
	//Initialize pnuematic gripper and wrist
	Solenoid grip0 = new Solenoid(1, 0);
	Solenoid grip1 = new Solenoid(1, 1);
	Solenoid wrist1 = new Solenoid(1, 2);
	Solenoid wrist2 = new Solenoid(1, 3);
	
	//Create empty storage values
	double initial_value;
	double compass_value;
	double lift_value;
	double initial_lift_value;
	
	boolean first_run_lift = true;

	//Initialize the pid loop
	double p = -1.7; 
	double i = 0;
	double d = 0;
	
	PIDController PID = new PIDController(p, i, d, lift_measure, lift_screw_motor);
	
	
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
	
	//Define the timer variables
	Timer timer = new Timer();
	Timer cube_shoot_timer = new Timer();
	
	//interger will act like a 3-way boolean for cube placement function
	int first_time;
	
	//Define variables for auto testing purposes 
	boolean auto_running;
	
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
	SendableChooser<String> chooser = new SendableChooser<>();
	//Define "station_chooser" object for station selector
	SendableChooser<String> station_chooser = new SendableChooser<>();
	
	
	//Custom functions 
	
	//Function that allows the robot to turn to a very specific degree
	public void auto_degree_turn( double turn_degree){
			if((initial_value - pigeon.getFusedHeading()) < (turn_degree - 2)){
				robotDrive.drive(( 1 / (initial_value - pigeon.getFusedHeading() + 10) + .3), 1);
			}else if((initial_value - pigeon.getFusedHeading()) > (turn_degree + 2)){
				robotDrive.drive(( 1 / (initial_value - Math.abs(pigeon.getFusedHeading()) + 10) + .3), -1);
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
	public void cube_shoot() {//right before this is run, set first time to 1. 
		if(first_run_lift) {
			if (cube_shoot_timer.get() == 0 || cube_shoot_timer.get() > 4) {
				cube_shoot_timer.reset();
				cube_shoot_timer.start();
				first_run_lift = false;
			}
		}
		if(first_run_lift == false) {
			if(cube_shoot_timer.get() < 1 && cube_shoot_timer.get() > 0) {
				wrist1.set(true);
				wrist2.set(true);
				if(lift_value > 29 && lift_value < 31) {
					lift_screw_motor.set(0);
				}else if(lift_value > 30) {
					lift_screw_motor.set((((lift_value - 30) * -1) * 0.045) - .3);
				}else if(lift_value < 30) {
					lift_screw_motor.set(((30 - lift_value) * .045) + .4);
				}
			}else if(cube_shoot_timer.get() < 2 && cube_shoot_timer.get() > 1) {
				grasping_motor_left.set(1);
				grasping_motor_right.set(1);
			}else if(cube_shoot_timer.get() < 2.5 && cube_shoot_timer.get() > 2) {
				wrist1.set(false);
				wrist2.set(false);
			}else if(cube_shoot_timer.get() == 2.5) {
				first_run_lift = true;
				
			}
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
	}
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		//Retrieve the initial Magnetometer value for teleop purposes
		initial_value = pigeon.getFusedHeading();
		initial_lift_value = lift_measure.getAverageValue();
		
		
		//Reset Encoder values
		wheel_counter1.reset();
		
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
		
		//Camera initialization
		UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture();
		camera1.setResolution(640, 480);
		camera1.setFPS(20);
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
		
		//Retrieve the initial lift encoder value for auto
		initial_lift_value = lift_measure.getAverageValue();
		
		//Retrieve values from the auto selector and publish values
		autoSelected = chooser.getSelected();
		System.out.println("Auto selected: " + autoSelected);
		
		//Create chooser for alliance stations
		stationSelected = station_chooser.getSelected();
		
		//Reset and start the timer
		timer.reset();
		timer.start(); 
			
		//Retreive information from the control system
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		//Reset encoder values
		wheel_counter1.reset();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		//Declare updated values for compass and lift encoder
		compass_value = (initial_value - pigeon.getFusedHeading());
		lift_value = ((lift_measure.getAverageValue() - initial_lift_value) * -1);
		
		//Check and execute the delay variable for competition (commented out)
		//Timer.delay(auto_delay_value);
		
		
		if(autoSelected == Single_Placement){
			//All of the auto code below is made to place a single power cube onto the switch
			if(stationSelected == left){
				if(gameData.charAt(0) == 'L') {
					//Places cube in the left switch when the robot starts at the left station
					
					//Executes between 0 and 4 seconds
					//Travels roughly 7000 encoder pulses
					if(timer.get() < 4 && timer.get() > 0) {
						if(wheel_counter1.getDistance()  < 7000){
						    drive_drift_compensation(initial_value, .4, .3, 0);
						}else if(wheel_counter1.getDistance() > 7300){
							robotDrive.drive(-.15, 0);
						}
					}
					//Executes between 4 and 6 seconds into auto
					//Does a perfect 90 degree turn to face the switch
					if(timer.get() < 6 && timer.get() > 4) {
						auto_degree_turn(90);
						wheel_counter1.reset();
					}
					//Lifts arm up to the switch
					//Drives up to the switch
					if(timer.get() < 8 && timer.get() > 6) {
						if(lift_value > 29 && lift_value < 31) {
							lift_screw_motor.set(0);
						}else if(lift_value > 30) {
							lift_screw_motor.set((((lift_value - 30) * -1) * 0.045) - .3);
						}else if(lift_value < 30) {
							lift_screw_motor.set(((30 - lift_value) * .045) + .4);
						}
						if(wheel_counter1.getDistance()  < 400){
						    drive_drift_compensation(initial_value, .4, .3, 90);
						}else if(wheel_counter1.getDistance() > 700){
							robotDrive.drive(-.15, 0);
						}  
						first_time = 1;
					}
					if(timer.get() < 9.5 && timer.get() > 8) {
						cube_shoot();
					}
				}else{
					
					//Crosses baseline when robot starts on the left and the switch is on the right
					if(wheel_counter1.getDistance()  < 7300){
				    	drive_drift_compensation(initial_value, .4, .3, 0);
					}else if(wheel_counter1.getDistance() > 7600){
						robotDrive.drive(-.15, 0);
					}
				}
			}else if(stationSelected == right){
				//Executes when the robot is placed in front of the right station
				//Crosses baseline when robot starts on the right and the switch is on the left
				if(gameData.charAt(0) == 'L') {
					
					if(wheel_counter1.getDistance()  < 7300){
				    	drive_drift_compensation(initial_value, .4, .3, 0);
					}else if(wheel_counter1.getDistance() > 7600){
						robotDrive.drive(-.15, 0);
					}
				}else{	//Places cube in the right switch when the robot starts at the right station	
					//Drives forward past the baseline for the first four seconds
					if(timer.get() < 4 && timer.get() > 0) {
						if(wheel_counter1.getDistance()  < 7000){
						    drive_drift_compensation(initial_value, .4, .3, 0);
						}else if(wheel_counter1.getDistance() > 7300){
							robotDrive.drive(-.15, 0);
						}
					}
					//Executes between 4 and 6 seconds into auto
					//Does a perfect 90 degree turn to face the switch
					if(timer.get() < 6 && timer.get() > 4) {
						auto_degree_turn(-90);
						wheel_counter1.reset();
					}
					//Lifts arm to switch
					//drives up to switch
					if(timer.get() < 8 && timer.get() > 6) {
						if(lift_value > 29 && lift_value < 31) {
							lift_screw_motor.set(0);
						}else if(lift_value > 30) {
							lift_screw_motor.set((((lift_value - 30) * -1) * 0.045) - .3);
						}else if(lift_value < 30) {
							lift_screw_motor.set(((30 - lift_value) * .045) + .4);
						}
						if(wheel_counter1.getDistance()  < 400){
						    drive_drift_compensation(initial_value, .4, .3, -90);
						}else if(wheel_counter1.getDistance() > 700){
							robotDrive.drive(-.15, 0);
						}  
						first_time = 1;
					}
					if(timer.get() < 9.5 && timer.get() > 8) {
						cube_shoot();
					}
				}	
			}else{
				//Executes when the robot is placed in front of the middle station
				//Goes to the left from middle if the switch is on the left
				if(gameData.charAt(0) == 'L'){
					//go forward
					if(timer.get() < 2 && timer.get() > 0){
						if(wheel_counter1.getDistance()  < 1700){
						    drive_drift_compensation(initial_value, .4, .3, 0);
						}else if(wheel_counter1.getDistance() > 2000){
							robotDrive.drive(-.15, 0);
						}
					}
					//turn left
					if(timer.get() < 3.5 && timer.get() > 2) {
						auto_degree_turn(-90);
						wheel_counter1.reset();
					}
					if(timer.get() < 5.5 && timer.get() > 3.5){
						if(wheel_counter1.getDistance()  < 2400){
						    drive_drift_compensation(initial_value, .4, .3, -90);
						}else if(wheel_counter1.getDistance() > 2600){
							robotDrive.drive(-.15, 0);
						}
					}
					//turn to face switch
					if(timer.get() < 7 && timer.get() > 5.5) {
						auto_degree_turn(0);
						wheel_counter1.reset();
					}
					//Lift arm to switch
					//Drive forward
					if(timer.get() < 8 && timer.get() > 7) {
						if(lift_value > 29 && lift_value < 31) {
							lift_screw_motor.set(0);
						}else if(lift_value > 30) {
							lift_screw_motor.set((((lift_value - 30) * -1) * 0.045) - .3);
						}else if(lift_value < 30) {
							lift_screw_motor.set(((30 - lift_value) * .045) + .4);
						}
						if(wheel_counter1.getDistance()  < 1200){
						    drive_drift_compensation(initial_value, .4, .3, 0);
						}else if(wheel_counter1.getDistance() > 1500){
							robotDrive.drive(-.15, 0);
						}
						first_time = 1;
					}
					//Release cube
					if(timer.get() < 9.5 && timer.get() > 8) {
						lift_screw_motor.set(0);
						cube_shoot();
					}
					//Turn left
					if(timer.get() < 11 && timer.get() > 9.5) {
						lift_screw_motor.set(0);
						auto_degree_turn(-90);
						wheel_counter1.reset();
					}
					if(timer.get() < 13 && timer.get() > 11) {
						lift_screw_motor.set(0);
						if(wheel_counter1.getDistance()  < 2700){
						    drive_drift_compensation(initial_value, .4, .3, -90);
						}else if(wheel_counter1.getDistance() > 3000){
							robotDrive.drive(-.15, 0);
						}
					}
					if(timer.get() < 14.5 && timer.get() > 13) {
						lift_screw_motor.set(0);
						auto_degree_turn(0);
						wheel_counter1.reset();
					}//Cross baseline
					if(timer.get() < 15 && timer.get() > 14.5) {
						lift_screw_motor.set(0);
						if(wheel_counter1.getDistance()  < 2000){
						    drive_drift_compensation(initial_value, .7, .3, 0);
						}else if(wheel_counter1.getDistance() > 2300){
							robotDrive.drive(-.15, 0);
						}
					}
				}
				else{//goes to the right from middle if switch is on the right
					//Drive forward
					if(timer.get() < 2 && timer.get() > 0){
						if(wheel_counter1.getDistance()  < 1700){
						    drive_drift_compensation(initial_value, .4, .3, 0);
						}else if(wheel_counter1.getDistance() > 2000){
							robotDrive.drive(-.15, 0);
						}
					}//Turn right
					if(timer.get() < 3.5 && timer.get() > 2) {
						auto_degree_turn(90);
						wheel_counter1.reset();
					}
					if(timer.get() < 5.5 && timer.get() > 3.5){
						if(wheel_counter1.getDistance()  < 2400){
						    drive_drift_compensation(initial_value, .4, .3, 90);
						}else if(wheel_counter1.getDistance() > 2600){
							robotDrive.drive(-.15, 0);
						}
					}//turn to face switch
					if(timer.get() < 7 && timer.get() > 5.5) {
						auto_degree_turn(0);
						wheel_counter1.reset();
					}//lift arm to switch
					//Drive up to switch
					if(timer.get() < 8 && timer.get() > 7) {
						if(lift_value > 29 && lift_value < 31) {
							lift_screw_motor.set(0);
						}else if(lift_value > 30) {
							lift_screw_motor.set((((lift_value - 30) * -1) * 0.045) - .3);
						}else if(lift_value < 30) {
							lift_screw_motor.set(((30 - lift_value) * .045) + .4);
						}
						if(wheel_counter1.getDistance()  < 1700){
						    drive_drift_compensation(initial_value, .4, .3, 0);
						}else if(wheel_counter1.getDistance() > 2000){
							robotDrive.drive(-.15, 0);
						}
						first_time = 1;
					}//Release cube
					if(timer.get() < 9.5 && timer.get() > 8) {
						lift_screw_motor.set(0);
						cube_shoot();
					}
					if(timer.get() < 11 && timer.get() > 9.5) {
						lift_screw_motor.set(0);
						auto_degree_turn(90);
						wheel_counter1.reset();
					}
					if(timer.get() < 13 && timer.get() > 11) {
						lift_screw_motor.set(0);
						if(wheel_counter1.getDistance()  < 2700){
						    drive_drift_compensation(initial_value, .4, .3, 90);
						}else if(wheel_counter1.getDistance() > 3000){
							robotDrive.drive(-.15, 0);
						}
					}
					if(timer.get() < 14.5 && timer.get() > 13) {
						lift_screw_motor.set(0);
						auto_degree_turn(0);
						wheel_counter1.reset();
					}//Cross baseline
					if(timer.get() < 15 && timer.get() > 14.5) {
						lift_screw_motor.set(0);
						if(wheel_counter1.getDistance()  < 2000){
						    drive_drift_compensation(initial_value, .7, .3, 0);
						}else if(wheel_counter1.getDistance() > 2300){
							robotDrive.drive(-.15, 0);
						}
					}
				}
			}
		}else if(autoSelected == Double_Placement){
			//Places a single power cube on the switch, and attempts to retrieve and place a second
			if(stationSelected == left){
				//Executes when the robot is placed in front of the left station
			}else if(stationSelected == right){
				//Executes when the robot is placed in front of the right station
			}else{
				//Executes when the robot is placed in front of the middle station
			}
		}else{
			//Only used in emergencies, simply crosses the baseline
			if(stationSelected == left){
				//Executes when the robot is placed in front of the left station
				//Simply drives forward to cross baseline
				if(wheel_counter1.getDistance()  < 7300){
			    	drive_drift_compensation(initial_value, .4, .3, 0);
				}else if(wheel_counter1.getDistance() > 7600){
					robotDrive.drive(-.15, 0);
				}
			}else if(stationSelected == right){
				//Executes when the robot is placed in front of the right station
				//Simply drives forward to cross baseline
				if(wheel_counter1.getDistance()  < 7300){
			    	drive_drift_compensation(initial_value, .4, .3, 0);
				}else if(wheel_counter1.getDistance() > 7600){
					robotDrive.drive(-.15, 0);
				}
			}else if(stationSelected == middle){
				if(gameData.charAt(0) == 'L'){//Executes when the robot is placed in front of the middle station
					//Goes to the left switch
					if(timer.get() < 2 && timer.get() > 0){
						if(wheel_counter1.getDistance()  < 1700){
						    drive_drift_compensation(initial_value, .4, .3, 0);
						}else if(wheel_counter1.getDistance() > 2000){
							robotDrive.drive(-.15, 0);
						}
					}
					if(timer.get() < 3.5 && timer.get() > 2) {
						auto_degree_turn(-90);
						wheel_counter1.reset();
					}
					if(timer.get() < 6.5 && timer.get() > 3.5){
						if(wheel_counter1.getDistance()  < 5400){
						    drive_drift_compensation(initial_value, .4, .3, -90);
						}else if(wheel_counter1.getDistance() > 5700){
							robotDrive.drive(-.15, 0);
						}
					}
					if(timer.get() < 8 && timer.get() > 6.5) {
						auto_degree_turn(0);
						wheel_counter1.reset();
					}
					if(timer.get() < 11.5 && timer.get() > 8) {
						if(wheel_counter1.getDistance()  < 5500){
						    drive_drift_compensation(initial_value, .4, .3, 0);
						}else if(wheel_counter1.getDistance() > 5800){
							robotDrive.drive(-.15, 0);
						}
					}
					if(timer.get() < 13 && timer.get() > 11.5) {
						auto_degree_turn(90);
						wheel_counter1.reset();
					}//Lifts arm to switch
					//Drives up to switch
					if(timer.get() < 15 && timer.get() > 13) {
						if(lift_value > 29 && lift_value < 31) {
							lift_screw_motor.set(0);
						}else if(lift_value > 30) {
							lift_screw_motor.set((((lift_value - 30) * -1) * 0.045) - .3);
						}else if(lift_value < 30) {
							lift_screw_motor.set(((30 - lift_value) * .045) + .4);
						}
						if(wheel_counter1.getDistance()  < 400){
						    drive_drift_compensation(initial_value, .4, .3, 90);
						}else if(wheel_counter1.getDistance() > 700){
							robotDrive.drive(-.15, 0);
						}
					}
				}else{
					if(timer.get() < 2 && timer.get() > 0){
						if(wheel_counter1.getDistance()  < 1700){
						    drive_drift_compensation(initial_value, .4, .3, 0);
						}else if(wheel_counter1.getDistance() > 2000){
							robotDrive.drive(-.15, 0);
						}
					}
					if(timer.get() < 3.5 && timer.get() > 2) {
						auto_degree_turn(90);
						wheel_counter1.reset();
					}
					if(timer.get() < 6.5 && timer.get() > 3.5){
						if(wheel_counter1.getDistance()  < 5400){
						    drive_drift_compensation(initial_value, .4, .3, 90);
						}else if(wheel_counter1.getDistance() > 5700){
							robotDrive.drive(-.15, 0);
						}
					}
					if(timer.get() < 8 && timer.get() > 6.5) {
						auto_degree_turn(0);
						wheel_counter1.reset();
					}
					if(timer.get() < 11.5 && timer.get() > 8) {
						if(wheel_counter1.getDistance()  < 5500){
						    drive_drift_compensation(initial_value, .4, .3, 0);
						}else if(wheel_counter1.getDistance() > 5800){
							robotDrive.drive(-.15, 0);
						}
					}
					if(timer.get() < 13 && timer.get() > 11.5) {
						auto_degree_turn(-90);
						wheel_counter1.reset();
					}//Lifts arm to switch
					//Drives up to switch
					if(timer.get() < 15 && timer.get() > 13) {
						if(lift_value > 29 && lift_value < 31) {
							lift_screw_motor.set(0);
						}else if(lift_value > 30) {
							lift_screw_motor.set((((lift_value - 30) * -1) * 0.045) - .3);
						}else if(lift_value < 30) {
							lift_screw_motor.set(((30 - lift_value) * .045) + .4);
						}
						if(wheel_counter1.getDistance()  < 400){
						    drive_drift_compensation(initial_value, .4, .3, -90);
						}else if(wheel_counter1.getDistance() > 700){
							robotDrive.drive(-.15, 0);
						}
					}
				}
			}
		}
		
		//publish the base magnetometer value to the dashboard
		SmartDashboard.putNumber("Compass Variance", compass_value);
		SmartDashboard.putNumber("Encoder Value", wheel_counter1.getDistance());
		
		//Reset Pigeon IMU value when it exceeds a value of 360
		if(pigeon.getFusedHeading() > 360) {
			compass_value = 0;
		}
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
			
			//Assign compass value and lift value to their respective mathematical functions
			compass_value = (initial_value - pigeon.getFusedHeading());
			lift_value = ((lift_measure.getAverageValue() - initial_lift_value) * -1);
			
			//The code below maps the joystick axis to their proper drive functions
			if(stick1.getRawButton(1)) {
				robotDrive.arcadeDrive(slow_val_y, slow_val_z); //smooth drive when top trigger is pressed
			}else{
				robotDrive.arcadeDrive((stick1.getY() * -1), (stick1.getZ() * -1)); //normal drive
			}
			
			
			//The code below maps all buttons to their proper functions
			if(stick2.getRawButton(9)) { 
				//opens the pnuematic arms on the robot
				grip0.set(true);
				grip1.set(true);
			}
			else if(stick2.getRawButton(11)) { 
				//Closes the pnuematic arms on the robot - soft
				grip0.set(true);
				grip1.set(false);
			}
			else if(stick2.getRawButton(13)) { 
				//Closes the pnuematic arms on the robot - hard
				grip0.set(false);
				grip1.set(false);
			}
			
			if (stick2.getRawButton(3)) {
				wrist1.set(true);
				wrist2.set(true);
			}
			else if (stick2.getRawButton(4)) {
				wrist1.set(false);
				wrist2.set(false);
			}
			if(stick2.getRawButton(7)) { 
				//Moves the grasping wheels forward in order to spit out a cube
				 grasping_motor_left.set(1);
				 grasping_motor_right.set(1);
			}
			if(stick2.getRawButton(8)) { 
				//Moves the grasping wheels backward in order to pull in a cube
				grasping_motor_left.set(-1);
				grasping_motor_right.set(-1);
			}
			if(stick2.getRawButton(7) == false && stick2.getRawButton(8) == false){ 
				//Sets the grasping motor speed to zero in the event that no buttons are bieng pressed
				grasping_motor_left.set(0);
				grasping_motor_right.set(0);
			}
			if(stick1.getRawButton(12)){ 
				//code for manually operation of the arm (mostly used for testing purposes, and won't be used during real gameplay)
				lift_screw_motor.set(-.5);
			}else if(stick1.getRawButton(11)){
				//code for manually operation of the arm (mostly used for testing purposes, and won't be used during real gameplay)
				lift_screw_motor.set(.5);
			}else if(stick2.getRawButton(10)){
				//Moves arm to lowest position (starting position)
				//This position will be acheived by pressing button 10 on joystick 2
				if(lift_value > -1 && lift_value < 1) {
					lift_screw_motor.set(0);
				}else if(lift_value > 0) {
					lift_screw_motor.set((((lift_value) * -1) * 0.045) - .3);
				}else if(lift_value < 0) {
					lift_screw_motor.set(((lift_value) * .045) + .4);
				}
			}else if(stick2.getRawButton(12)){
				//Moves arm to middle position (switch height)
				//This position will be acheived by pressing button 12 on joystick 2
				if(lift_value > 29 && lift_value < 31) {
					lift_screw_motor.set(0);
				}else if(lift_value > 30) {
					lift_screw_motor.set((((lift_value - 30) * -1) * 0.045) - .3);
				}else if(lift_value < 30) {
					lift_screw_motor.set(((30 - lift_value) * .045) + .4);
				}
			}else if(stick2.getRawButton(14)){
				//Moves arm to highest position
				//This position will be acheived by pressing button 14 on joystick 2
				if(lift_value > 44 && lift_value < 46) {
					lift_screw_motor.set(0);
				}else if(lift_value > 45) {
					lift_screw_motor.set((((lift_value - 45) * -1) * 0.045) - .3);
				}else if(lift_value < 45) {
					lift_screw_motor.set(((45 - lift_value) * .035) + .4);
				}
			}else {
				//Code that sets the lift_screw_motor's speed to 0 in the event that no button is pressed at all
				lift_screw_motor.set(0);
			}
			if (stick2.getRawButton(1)) {
				cube_shoot();
			}
			//Publish SmartDashboard values
			SmartDashboard.putBoolean("Smooth Drive", stick1.getRawButton(1)); //Indicates whether or not smooth drive is active
			SmartDashboard.putNumber("Compass Variance", compass_value); //Displays the variance between the IMU's starting direction and it's current angle 
			SmartDashboard.putNumber("Lift Angle", lift_value); //outputs the value of the lift (displays the average voltage output)
			SmartDashboard.putNumber("Encoder Value", wheel_counter1.getDistance());//Displays the current encoder count
			
			
			//Delays Cycles in order to avoid undue CPU usage
			Timer.delay(.005);
			
			//Reset Pigeon IMU value
			if(pigeon.getFusedHeading() > 360) {
				compass_value = 0;
			}
			
			
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		
	}
}
