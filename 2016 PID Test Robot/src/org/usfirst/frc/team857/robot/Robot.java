package org.usfirst.frc.team857.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends SampleRobot {

	private getGamePadButton gamepad;
	private Joystick leftDrive;
	private Joystick rightDrive;
	
	private RobotDrive	driveTrain;
	private AngleController angle;
	private VelocityController leftShooterSpeed;
	private VelocityController rightShooterSpeed;
	
	private Computer_Vision shooterCamera;
	
	//Used to keep track of PID setpoints
	private double angleSetpoint;
	private double speedSetpoint;
	private double prevAngleSetpoint=-1000;
	
	//Calibrated shooter velocity in ft/s for the chosen setpoint. Eventually a function of RPM?
	private double shooterVelocity = 30;
	
	//Drive multiplier for "slow mode"
	private double slowDriveSpeed = 0.75;
	private double mediumDriveSpeed = .85;
	
	//Kicker Speed, 0.0 to -1.0
	private double kickSpeed = -0.5;
	
	//Ball grab speed, 0.0 to -1.0
	private double grabBallSpeed = -0.4;
	
	//Ball shoot speed, 0.0 to 1.0
	private double shootSpeed = 0.8;
	
	//Shooter angle updates this amount when nudged
	private double nudgeShooterAmount = 0.01;
	
	private double rangeOffset = 0.0/12.0; //Offset range in feet
	
	private Timer time = new Timer();
	private Joystick auton;
	
	public void robotInit() {
		
		//Setup robot IO map
		RobotMap.init();
		
		//Driver inputs
		gamepad = new getGamePadButton(0, .1, .1, .1);
		leftDrive = new Joystick(1);
		rightDrive = new Joystick(2);
		auton = new Joystick(3);
		//PID Setpoints
		angleSetpoint = 45.0;
		speedSetpoint = 0;
		
		//Drive train
		driveTrain = new RobotDrive(RobotMap.leftDriveMotor, RobotMap.rightDriveMotor);		
		
		//Increase default drivetrain timeout
		//If the drivetrain times out in the dashboard increase this number (time in seconds before timeout)
		driveTrain.setExpiration(0.4); 
		
		//Cameras
        //Add axis camera via the smart dashboard! - IP should be 10.8.57.11. Check in roborio web interface!
		//Smartdashboard cameras: 
		//	Found under View -> Add...
		//	Simple Camera Viewer for axis camera. Set IP by right clicking the Simple Camera Viewer and selecting Properties.
		//	USB Webcam Viewer for shooter camera. Should be drawn to automatically by robot code
	
		//shooterCamera = new Computer_Vision(RobotMap.shooterCamera);
		CameraServer.getInstance().startAutomaticCapture(RobotMap.shooterCamera);
		
		//Shooter Related
		/*
		leftShooterSpeed = new VelocityController("LeftShooter", 0, 1.0/12000.0, 0, 0.0000525, RobotMap.leftShooterSpeed, RobotMap.leftShooterMotor, 0.5);
		rightShooterSpeed = new VelocityController("RightShooter", 0, 1.6/12000.0, 0, 10.0/12000.0, RobotMap.rightShooterSpeed, RobotMap.rightShooterMotor, 0.5);
		leftShooterSpeed.disable();
		rightShooterSpeed.disable();
		//*/
		//Voltage for now!
		//Needs fine tuning
		
		//Angle control, either one may work
		//angle = new AngleController(0.016,0,0,.016, RobotMap.anglePotentiometer, RobotMap.angleMotor);
		//angle = new AngleController(0.012,0.0002,0.002,.016, RobotMap.anglePotentiometer, RobotMap.angleMotor);
		angle = new AngleController(0.010,0.00035,0.002,.012, RobotMap.anglePotentiometer, RobotMap.angleMotor);
		
		//angle = new AngleController(0.0,0.000,0.00,0, RobotMap.anglePotentiometer, RobotMap.angleMotor);
		angle.disable();
		
		SmartDashboard.putNumber("IN", 0.70);
		SmartDashboard.putNumber("OUT", 0.80);
		SmartDashboard.putNumber("P",10);
		SmartDashboard.putNumber("I",0.35);
		SmartDashboard.putNumber("D",2);
		SmartDashboard.putNumber("F", 12);
		SmartDashboard.putNumber("Angle Setpoint", angleSetpoint);
	}

	public void autonomous() {
		
		time.stop();
		time.reset();
		time.start();
		//TODO: Auto controller button 5 is not functioning!
		while (isAutonomous() && isEnabled()) {

			//Auto 1
			if (auton.getRawButton(1)) {
				if (time.get() < 1) {
	    			angleSetpoint = 90;
	    			angle.setSetpoint(angleSetpoint);
	        		angle.enable();
	        		
	        		driveTrain.tankDrive(0, 0);
	    		} else if (time.get() < 2) {
	    			angleSetpoint = 60;
	    			angle.setSetpoint(angleSetpoint);
	        		angle.enable();
	        		

	            	//RobotMap.leftWedgeMotor.set(0.3);
	            	//RobotMap.rightWedgeMotor.set(0.3);
	        		
					//driveTrain.tankDrive(-slowDriveSpeed, -slowDriveSpeed);
	    		} else if (time.get() < 3) {
	    			angleSetpoint = 30;
	    			angle.setSetpoint(angleSetpoint);
	        		angle.enable();
	        		

	            	//RobotMap.leftWedgeMotor.set(0.0);
	            	//RobotMap.rightWedgeMotor.set(0.0);
	    		} else if (time.get() < 4) {
	    			angleSetpoint = 0;
	    			angle.setSetpoint(angleSetpoint);
	        		angle.enable();
	        		
	    			
	    			//driveTrain.tankDrive(-slowDriveSpeed, -slowDriveSpeed);
	    		} else if (time.get() < 4.3) {
	    			driveTrain.tankDrive(-slowDriveSpeed, -slowDriveSpeed);
	    		} else if (time.get() < 4.6) {

	            	RobotMap.leftWedgeMotor.set(-0.3);
	            	RobotMap.rightWedgeMotor.set(-0.3);
	            	driveTrain.tankDrive(-slowDriveSpeed, -slowDriveSpeed);
				} else if (time.get() < 9.5) {

	            	RobotMap.leftWedgeMotor.set(0.0);
	            	RobotMap.rightWedgeMotor.set(0.0);
	    			
					
					driveTrain.tankDrive(-slowDriveSpeed,-slowDriveSpeed);
					
				} else {
					driveTrain.tankDrive(0, 0);
				}
			} else if (auton.getRawButton(2)) { 
				//Auto 2
				//do nothing, shooter angle up
				  angleSetpoint = 90;
				  angle.setSetpoint(angleSetpoint);
				  angle.enable();
				  driveTrain.tankDrive(0,0);
/*			} else if (auton.getRawButton(3)) { 
				//Auto 3
				//drive under low bar, slow speed
				angleSetpoint = 0; //goes to angle zero
				angle.setSetpoint(angleSetpoint);
	    		angle.enable();
	    		
				if (time.get() < 5.0) {
					driveTrain.tankDrive(-slowDriveSpeed, -slowDriveSpeed);
	            	
				} else {
					driveTrain.tankDrive(0,0);
				}*/
			} else if (auton.getRawButton(4)) { 
				//Auto 4
				//drive over a defense, medium speed
				angleSetpoint = 90;
				angle.setSetpoint(angleSetpoint);
	    		angle.enable();
				if (time.get() < 4.5) { 
					driveTrain.tankDrive(-mediumDriveSpeed, -mediumDriveSpeed);
				} else {
					driveTrain.tankDrive(0,0);
				}
			} else if (auton.getRawButton(3)) { //portcullis, robot will move in direction of wedge
				
	    		if (time.get() < .3) {
	    			angleSetpoint = 90;
					angle.setSetpoint(angleSetpoint);
		    		angle.enable();
		    		
		    		RobotMap.leftWedgeMotor.set(-0.3);
		    		RobotMap.rightWedgeMotor.set(-0.3);
		    		
	    			
	    		} else if (time.get() < 1) {
	    			driveTrain.tankDrive(slowDriveSpeed, slowDriveSpeed);
	    			
	    			RobotMap.leftWedgeMotor.set(0);
	            	RobotMap.rightWedgeMotor.set(0);
	    			
	    		} else if (time.get() < 2) {
	    			driveTrain.tankDrive(0, 0);
	    			
	    			angleSetpoint = 60;
					angle.setSetpoint(angleSetpoint);
		    		angle.enable();
		    		
	    		} else if (time.get() < 3) {
	    			angleSetpoint = 30;
					angle.setSetpoint(angleSetpoint);
		    		angle.enable();
		    		
	    		} else if (time.get() < 4) {
	    			angleSetpoint = 0;
					angle.setSetpoint(angleSetpoint);
		    		angle.enable();
	    		} else if (time.get() < 8) {
	    			driveTrain.tankDrive(mediumDriveSpeed, mediumDriveSpeed);
	    			
	    		} else {
	    			driveTrain.tankDrive(0, 0);
	    		}
			}
			/*
			//drive forward for 5 seconds at medium speed
			angleSetpoint = 90;
			angle.setSetpoint(angleSetpoint);
    		angle.enable();
			if (time.get() < 4.5) { 
				driveTrain.tankDrive(-mediumDriveSpeed, -mediumDriveSpeed);
			} else {
				driveTrain.tankDrive(0,0);
			}
			*/
		}
	}
    
	public void operatorControl() {
	    //PID Setpoints
		angleSetpoint = 45.0;
		speedSetpoint = 0;
        while (isOperatorControl() && isEnabled()) {
			angleSetpoint = SmartDashboard.getNumber("Angle Setpoint");
        	angle.getPIDAngleController().setPID(
					SmartDashboard.getNumber("P")/1000.0,
					SmartDashboard.getNumber("I")/1000.0,
					SmartDashboard.getNumber("D")/1000.0,
					SmartDashboard.getNumber("F")/1000.0);
			//leftShooterSpeed.setSetpoint(SmartDashboard.getNumber("RPM"));
			//leftShooterSpeed.enable();
        	/*
        	rightShooterSpeed.getPIDController().setPID(
					SmartDashboard.getNumber("P")/12000.0,
					SmartDashboard.getNumber("I")/12000.0,
					SmartDashboard.getNumber("D")/12000.0,
					SmartDashboard.getNumber("F")/12000.0);
			rightShooterSpeed.setSetpoint(SmartDashboard.getNumber("RPM"));
			rightShooterSpeed.enable();
*/
        	//Display shooter camera on dashboard
        	//shooterCamera.update();
           // CameraServer.getInstance().setImage(shooterCamera.frame);
            
        	//Control Shooter Angle
        	if (gamepad.getButtonA()) {
        		angleSetpoint = 0.0;
        	}
        	
        	if (gamepad.getButtonX()) {
        		angleSetpoint = 45.0;
        	}
        	
        	if (gamepad.getButtonY()) {
        		angleSetpoint = 90.0;
        	}
        	
        	//Control Kicker
        	if (gamepad.getButtonB()) {
        		RobotMap.kickerMotor.set(kickSpeed);
        	} else {
        		RobotMap.kickerMotor.set(0);
        	}
        	
        	/*
        	//UNUSED BUTTONS
        	if (gamepad.getRightBumper()) {
        	} else {
        	

        	if (gamepad.getRightTrigger() > 0.0) {
        	} else {
        	}
        	
        	if (gamepad.getRight()) {
        	} else {
        	}
        	
        	if (gamepad.getLeft()) {
        	} else {
        	}
        	
    	 	*/

        	//Update Computer Vision for calculating target center location
        	//shooterCamera.update();
        	//SmartDashboard.putNumber("Target offset Center", shooterCamera.center());
        	
        	//Nudge Shooter Speed
        	if (gamepad.getUp()) {
        		//speedSetpoint += 100; //RPM
        		speedSetpoint += 0.01; //Volts
        	} else if (gamepad.getDown()) {
        		//speedSetpoint += 100; //RPM
        		speedSetpoint -= 0.01; //Volts
        	}
        	
    		//Wedge Control
        	RobotMap.leftWedgeMotor.set(gamepad.getLeftAxisY());
        	RobotMap.rightWedgeMotor.set(gamepad.getLeftAxisY());

        	//Drive Train Control
        	if (leftDrive.getButton(ButtonType.kTrigger) && rightDrive.getButton(ButtonType.kTrigger)) {

            	//Drive slowly if both triggers are pulled
        		driveTrain.tankDrive(leftDrive.getAxis(AxisType.kY)*slowDriveSpeed, rightDrive.getAxis(AxisType.kY)*slowDriveSpeed);
        	} else if (!leftDrive.getButton(ButtonType.kTrigger) && rightDrive.getButton(ButtonType.kTrigger)){ //if right trigger pulled, go in reverse
        		
        		//drive in reverse if right trigger is held down
        		driveTrain.tankDrive(-rightDrive.getAxis(AxisType.kY), -leftDrive.getAxis(AxisType.kY));
        	} else {
        		
        		//Drive normally
        		driveTrain.tankDrive(leftDrive.getAxis(AxisType.kY), rightDrive.getAxis(AxisType.kY));
        	}
        	
        	//Nudge shooter angle
        	if (Math.abs(gamepad.getRightAxisY()) > 0.1) {
        		angleSetpoint += (gamepad.getRightAxisY() > 0.1) ? nudgeShooterAmount : -1.0 * nudgeShooterAmount;
        		angleSetpoint = Math.min(Math.max(angleSetpoint, -20), 110.0);
        	}

        	//Update shooter angle
        	if (gamepad.getLeftBumper()) {
        		
            	//Get distance to use with shooter
            	double distance = (RobotMap.lidar.getDistance() / 2.54 / 12.0) - rangeOffset; //feet

            	//TODO: add velocity control
            	angleSetpoint = findAngle(distance, shooterVelocity);
            	
            	//Debug info
            	SmartDashboard.putNumber("Distance", distance);
            	SmartDashboard.putNumber("Velocity", shooterVelocity);
            	SmartDashboard.putNumber("Calculated Shooting Angle", angleSetpoint);
        	}
        	
        	
        	//SmartDashboard.putNumber("Current Shooting Angle", angleSetpoint);

        	//Control shooter
        	if (gamepad.getLeftTrigger() > 0.1) {
        		
        		//Move the shooter down and turn on motors in reverse to grab the boulder

            	angleSetpoint = -20; 
            	
        		speedSetpoint = grabBallSpeed;
        		//leftShooterSpeed.disable();
        		//rightShooterSpeed.disable();
            	//RobotMap.leftShooterMotor.set(-speedSetpoint);
            	//RobotMap.rightShooterMotor.set(speedSetpoint);
        		RobotMap.leftShooterMotor.setInverted(false);
        		RobotMap.rightShooterMotor.setInverted(true);
            	RobotMap.leftShooterMotor.set(SmartDashboard.getNumber("IN"));
            	RobotMap.rightShooterMotor.set(SmartDashboard.getNumber("IN"));
        		
        	} else if (gamepad.getRightTrigger() > 0.1) {
        		RobotMap.leftShooterMotor.setInverted(true);
        		RobotMap.rightShooterMotor.setInverted(false);
        	
        		//Spin up motors to fire the boulder
        		
        		//Use this code if PID is not tuned
        		speedSetpoint = shootSpeed;
        		//leftShooterSpeed.disable();
        		//rightShooterSpeed.disable();
            	//RobotMap.leftShooterMotor.set(-speedSetpoint);
            	//RobotMap.rightShooterMotor.set(speedSetpoint);
        		
            	RobotMap.leftShooterMotor.set(SmartDashboard.getNumber("OUT"));
            	RobotMap.rightShooterMotor.set(SmartDashboard.getNumber("OUT"));
            	
        		//TODO: Use this code if PID is tuned properly
            	//leftShooterSpeed.setSetpoint(SmartDashboard.getNumber("RPM"));
            	//rightShooterSpeed.setSetpoint(SmartDashboard.getNumber("RPM"));
            	//SmartDashboard.putNumber("Left Error", leftShooterSpeed.getPIDController().getError());
            	//SmartDashboard.putNumber("Right Error", rightShooterSpeed.getPIDController().getError());
        		//leftShooterSpeed.enable();
        		//rightShooterSpeed.enable();
        		
        	} else {
        		//Turn off shoot motors
        		//leftShooterSpeed.disable();
        		//rightShooterSpeed.disable();
        		speedSetpoint = 0.0;
            	RobotMap.leftShooterMotor.set(speedSetpoint);
            	RobotMap.rightShooterMotor.set(speedSetpoint);
        	}

        	//Move shooter to setpoint angle
        	if (angleSetpoint != prevAngleSetpoint) {
    			SmartDashboard.putNumber("Angle Setpoint", angleSetpoint);
        		prevAngleSetpoint = angleSetpoint;
        		System.out.println("Updating Shooter Angle");
        		angle.setSetpoint(angleSetpoint);
            	angle.enable();
        	}
        	
        	//Shooter speed update
        	
        }
        
        //Disable the PID loops to prevent errors on robot enable
        angle.disable();
        prevAngleSetpoint = -1000;
        //leftShooterSpeed.disable();
       // rightShooterSpeed.disable();
	}
	
	//Range in feet
	//Velocity in feet/s
	//Calculates shooter angle given a range and velocity
	public double findAngle(double r, double Vi) { // range and initial velocity, REFER TO TOBIN'S EQUATION
		
    	if(r == 0 || Vi == 0.0){
    		//Meltdown
    		//Print error message, or fudge the values a bit
    		return 45.0;
    	}
    	
    	double angle; // final result angle
    	double haxl = 0.5; //Axel height in feet
    	double laxl = 1.0; //Axel to shooter exit in feet
    	double tht0 = 35.0; //Avg angle in degrees
    	
    	double yT = 8.083; //Avg goal height in feet
    	
    	double yTm = (yT - haxl - laxl * Math.sin(tht0 / 180.0 * Math.PI));
    	double xTm = (r - laxl * Math.cos(tht0 / 180.0 * Math.PI));
    	
    	double k = 16.1 * Math.pow(xTm,  2) / Math.pow(Vi,  2);
    	double tmp1 = Math.pow((xTm/k), 2)- 4*((yTm/k) +1);
    	if (tmp1 > 0){
	    	//double tmp = Math.sqrt(Math.pow((xTm/k), 2)- 4*((yTm/k) +1));
	    	double tmp = Math.sqrt(tmp1);
	    	
			double y12_1 = ((xTm/k) + tmp)/2.0; // tobins formula sheet 3, quadratic formula
			double y12_2 = ((xTm/k) - tmp)/2.0;
			double ySmall = 0;
			
			if(y12_1 < y12_2){ // find smaller angle 
				ySmall = y12_1;
			} else {
				ySmall = y12_2; 
			}
			
			double radians = Math.atan(ySmall); //inverse tan, find radian
			angle = radians * (180/Math.PI); //convert to degrees
			//Clamp output range to 0-80 degrees
    	}
    	else {
    		angle = 45.0;
    	}
    	return Math.min(Math.max(angle, 0.0), 80.0);
    	
    }
	
	private synchronized double calculateSetpoint(double oldsetpoint, double setpoint, double deltime) {
		double zeta = 0.9;
		double freq = 0.125;
		
		double wn = freq * 2.0 * Math.PI;
		
		double deltht = oldsetpoint - setpoint; 	//Old target - new target
		
		double tmp0 = Math.sqrt(1.0 - Math.pow(zeta, 2));
		double tmp1 = Math.exp(-1.0 * zeta * wn * deltime);
		double tmp2 = wn * tmp0 * deltime;
		SmartDashboard.putNumber("DEBUG oldsetpoint", oldsetpoint);
		SmartDashboard.putNumber("DEBUG setpoint", setpoint);
		SmartDashboard.putNumber("DEBUG time", deltime);

		SmartDashboard.putNumber("Calculated Setpoint", setpoint + deltht * (1 - tmp1 * Math.cos(tmp2) - zeta / tmp0 * tmp1 * Math.sin(tmp2)));
		return setpoint + deltht * (1 - tmp1 * Math.cos(tmp2) - zeta / tmp0 * tmp1 * Math.sin(tmp2));
	}
}
