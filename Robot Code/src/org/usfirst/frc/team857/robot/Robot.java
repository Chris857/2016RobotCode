package org.usfirst.frc.team857.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends SampleRobot {
	
	//private PIDController shooterSpeedControl;
	private getGamePadButton gamepad;
	private Joystick leftJoystick;
	private Joystick rightJoystick;
	
	private RobotDrive	driveTrain;
	private AngleController angle;
	private VelocityController leftShooterSpeed, rightShooterSpeed;
	
	private Camera shooterCamera;
	private Camera driveCamera;
	
	private double g = 9.86;    //Grabity
	private double m = 4.98952; //Shooter mass
	private double d = 0.1143;  //Shooter center of gravity from axle
	private double r = 0.5;     //Angle motor resistance
	
	//private int lidarDelay = 10;
	
	public void robotInit() {
		
		//Setup robot IO map
		RobotMap.init();
		
		//Driver inputs
		gamepad = new getGamePadButton(0, .1, .1, .1);
		leftJoystick = new Joystick(1);
		rightJoystick = new Joystick(2);
		
		//Drive train
		driveTrain = new RobotDrive(RobotMap.leftDriveMotor, RobotMap.rightDriveMotor);		
		
		//Shooter Related
		leftShooterSpeed = new VelocityController("LeftShooter", 0.0005, 0, 0, 0.0005, RobotMap.leftShooterSpeed, RobotMap.leftShooterMotor);
		rightShooterSpeed = new VelocityController("RightShooter", 0.0005, 0, 0, 0.0005, RobotMap.rightShooterSpeed, RobotMap.rightShooterMotor);
		angle = new AngleController(-2, -.2, 0, m * g * d * r, RobotMap.anglePotentiometer, RobotMap.angleMotor);

		//Cameras
		shooterCamera = new Camera(RobotMap.shooterCamera);
		driveCamera = new Camera(RobotMap.driveCamera);
        //CameraServer.getInstance().startAutomaticCapture(RobotMap.shooterCamera);
        //CameraServer.getInstance().startAutomaticCapture(RobotMap.driveCamera);
		
		//angle.setSetpoint(2.65);
		
	}

	public void autonomous() {
		while (isAutonomous() && isEnabled()) {
			Timer.delay(0.005);
		}
	}

	public void operatorControl() {

        while (isOperatorControl() && isEnabled()) {


    		//Wedge Control
    		
    		//Left trigger moves wedge up, right trigger moves wedge down, otherwise turn off wedge
        	if (leftJoystick.getButton(ButtonType.kTrigger) && !rightJoystick.getButton(ButtonType.kTrigger)) {
        		RobotMap.leftWedgeMotor.set(0.8);
        		RobotMap.rightWedgeMotor.set(0.8);
        	} else if (!leftJoystick.getButton(ButtonType.kTrigger) && rightJoystick.getButton(ButtonType.kTrigger)) {
        		RobotMap.leftWedgeMotor.set(-0.8);
        		RobotMap.rightWedgeMotor.set(-0.8);
        	} else {
        		RobotMap.leftWedgeMotor.set(0);
        		RobotMap.rightWedgeMotor.set(0);
        	}
        	
        	
        	//Drive Train Control
        	
         	//Drive slowly if both triggers are pulled
        	if (leftJoystick.getButton(ButtonType.kTrigger) && rightJoystick.getButton(ButtonType.kTrigger)) {
        		driveTrain.tankDrive(leftJoystick.getAxis(AxisType.kY)*0.75, rightJoystick.getAxis(AxisType.kY)*0.75);
        	} else {
        		driveTrain.tankDrive(leftJoystick.getAxis(AxisType.kY), rightJoystick.getAxis(AxisType.kY));
        	}
        	

        	//Get distance to use with shooter
        	double distance = RobotMap.lidar.getDistance();
        	SmartDashboard.putNumber("Distance", distance);
        	
        	
        	//Check if buttons pressed down on the game pad
        	//correct wheel speeds not in place
        	
        	if (gamepad.getLeftBumper()){ //If left bumper is pressed down
        		angle.setSetpoint(setpoint); //Adjust shooter to the correct angle, setpoint not set
        	}
        	if (gamepad.getRightTrigger() > 500){ //If right trigger down, start motorsto shoot
        		RobotMap.leftShooterMotor.set(.8); //Set left shooter motor
        		RobotMap.rightShooterMotor.set(.8); //Set Right shooter motor
        	} else if (gamepad.getLeftTrigger() > 500){ //If left trigger down, start motors so suck in ball
        		RobotMap.leftShooterMotor.set(-.3); //Set left shooter motor
        		RobotMap.rightShooterMotor.set(-.3); //Set Right shooter motor
        	} else {
        		RobotMap.leftShooterMotor.set(0); //Set left shooter motor to zero
        		RobotMap.rightShooterMotor.set(0); //Set Right shooter motor to zero
        	}
        	if (gamepad.getButtonX()) { //if X buton is down
        		angle.setSetpoint(setpoint); //Angle down shooter plate for ball take-in
        	} 
        	if (gamepad.getButtonB()) { //if B button is down
        		RobotMap.kickerMotor.set(-.4); //kick ball into shooting motors
        	} else {
        		RobotMap.kickerMotor.set(0); //Turn off motor
        	}
        	if (gamepad.getRightAxisY() > 500) {
        		double THT = angle.getSetpoint();
        		THT++;
        		angle.setSetpoint(THT);
        	} else {
        		double THT = angle.getSetpoint();
        		THT--;
        		angle.setSetpoint(THT);
        	}
       
/*
            if (SmartDashboard.getBoolean("Drive Camera")) {
            	driveCamera.getFrame();
            } else {
            	shooterCamera.getFrame();
            }
            //*/
        	//if (gamepad.getButtonB()) {
        		/*
            	SmartDashboard.putNumber("Speed ERROR", shooterSpeed.getError());
            	SmartDashboard.putNumber("Speed", RobotMap.leftShooterSpeed.getRate());
            	SmartDashboard.putNumber("PID Ref", RobotMap.leftShooterSpeed.pidGet());
        		shooterSpeed.setSetpoint(SmartDashboard.getNumber("Shooter Motor Speed"));
        		*/
        		/*
        		double target = SmartDashboard.getNumber("Shooter Motor Speed");
        		double reference = 60.0 / RobotMap.leftShooterSpeed.getPeriod();
        		double k = SmartDashboard.getNumber("K");

            	SmartDashboard.putNumber("Error", (target - reference));
            	
        		double control = k * (target - reference) + 0.0005 * target;
            	RobotMap.leftShooterMotor.set(-1.0*control);
            	*/
           // 	RobotMap.leftShooterMotor.set(-1.0 * SmartDashboard.getNumber("Shooter Motor Speed"));
           // 	RobotMap.rightShooterMotor.set(SmartDashboard.getNumber("Shooter Motor Speed"));
            	//RobotMap.rightShooterMotor.set(-0.4);
        		//shooterSpeed.setSetpoint(SmartDashboard.getNumber("Shooter Motor Speed"));
            	//SmartDashboard.putNumber("Error", shooterSpeed.getPIDController().getError());
        		//shooterSpeed.enable();
       /* 	} else if (gamepad.getLeftTrigger()>0.2) {
        		//shooterSpeed.disable();
            	RobotMap.leftShooterMotor.set(0.4);
            	RobotMap.rightShooterMotor.set(-0.4);
        	} else {
        		//shooterSpeed.disable();
            	RobotMap.leftShooterMotor.set(0);
            	RobotMap.rightShooterMotor.set(0);
        	}
        	SmartDashboard.putNumber("RPM right", 60.0 / RobotMap.rightShooterSpeed.getPeriod());
        	SmartDashboard.putNumber("RPM left", 60.0 / RobotMap.leftShooterSpeed.getPeriod());

        	if (gamepad.getRightTrigger()>0.2) {
            	RobotMap.kickerMotor.set(-0.35);
        	} else {
            	RobotMap.kickerMotor.set(0);
        	}      	

        	*/
/*
        	if (gamepad.getButtonA()) {
            	angle.enable();
            	//pidController.enable();
        	} else {
            	angle.disable();
        		//pidController.disable();
        	}
*/
   //     	angle.setSetpoint(2.65);
     //   	angle.enable();
        	
        	//angle.setSetpoint(SmartDashboard.getNumber("Target Angle"));
        	//angle.setSetpoint(gamepad.getRightTrigger())
        	/*
        	if (gamepad.getButtonA()) {
            	RobotMap.angleMotor.set(0.2);
        	} else {
            	RobotMap.angleMotor.set(0);
        	}

        	*/
/*
           // RobotMap.leftWedgeMotor.set(gamepad.getLeftAxisY());
            //RobotMap.rightWedgeMotor.set(-1.0 * gamepad.getLeftAxisY());
        	if (gamepad.getButtonB()) {
            	RobotMap.leftShooterMotor.set(-1.0 * SmartDashboard.getNumber("Shooter Motor Speed"));
            	RobotMap.rightShooterMotor.set(SmartDashboard.getNumber("Shooter Motor Speed"));
        	} else if (gamepad.getLeftTrigger()>0.2) {

            	RobotMap.leftShooterMotor.set(0.4);
            	RobotMap.rightShooterMotor.set(-0.4);
        	}else{
            	RobotMap.leftShooterMotor.set(0);
            	RobotMap.rightShooterMotor.set(0);
        	}
        	

        	SmartDashboard.putNumber("PID ERROR", angle.getPIDAngleController().getError());
        	SmartDashboard.putNumber("Left Motor Speed", RobotMap.leftShooterSpeed.get());
        	
        	SmartDashboard.putNumber("Lidar Distance", RobotMap.lidar.getDistance());
        	SmartDashboard.putNumber("Encoder Angle", angle.getPosition());
        	SmartDashboard.putNumber("Encoder Potentiometer", RobotMap.anglePotentiometer.getVoltage());

        	SmartDashboard.putBoolean("Angle Top Switch", RobotMap.angleLowerLimitSwitch.get());
        	SmartDashboard.putBoolean("Angle Bottom Switch", RobotMap.angleUpperLimitSwitch.get());
*/
            //Timer.delay(0.005);
        }
	}
}
