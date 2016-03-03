
package org.usfirst.frc.team857.robot;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	
	
	
	private driveTrain drive; //driveTrain
	private Joystick stickL; //left joystick
	private Joystick stickR; //right joystick 
	
	private Auto1 auto;
	
	private LidarSubsystem laser; //distance laser
	
	
    private getGamePadButton gamePad; //gamePad
    private SpeedController leftDrive;	// left drive train motor controlled with a joystick 
    private SpeedController rightDrive; //right drive chain motor controlled with a joystick
	
    private Counter leftDriveCounter; //encoder for the left drive train
    private Counter rightDriveCounter; //encoder for the right drive train
    
    private SpeedController leftShooter; //left shooter motor
    private SpeedController rightShooter; //right shooter motor
    private double motorSpeed; //desired motor speed sent into shoot.fire(desiredSpeed)
    private SpeedController angleMotor; //angle motor
    
    private double shooterAngleError = 2; //error leaneancy calculated before shooting
   // private Counter leftShooterEncoder; //encoder for the left shooting motor
   // private Counter rightShooterEncoder; //encoder for the right shooting motor
    private Encoder angleMotorEncoder; //encoder for the angle adjusting motor
    private double shooterAngleSet; //angle that is sent into the shooter angle encoder
    private SpeedController ballPusher; //motor on the shooter that pushes ball into the firing wheels
    
    private SpeedController wedgeL;//left wedge motor
    private SpeedController wedgeR;//right wedge motor
    
    private wedgeControl wedge; //wedge controller
    private Shooter shoot; //shoot!
    
    private boolean shootOverride = false; //override using y and b and a as shooting angles, default to false
    
   private Camera _mainCamera; //vision camera
   private Computer_Vision _magicCamera; //smart camera
    
   private Timer time;

    
    public void robotInit() {

    /*	laser = new LidarSubsystem(edu.wpi.first.wpilibj.I2C.Port.kMXP);
    	
    	_mainCamera = new Camera("cam0");
    	_magicCamera = new Computer_Vision("cam1");
    	*/
		leftDrive = new VictorSP(0);		// initialize the motor as a victor on channel 0
		rightDrive = new VictorSP(1);	 // initialize the motor as a victor on channel 1
		leftDriveCounter = new Counter(2);
		rightDriveCounter = new Counter(3);
    	drive = new driveTrain(leftDrive,rightDrive,leftDriveCounter,rightDriveCounter); 
    	
    	
    	
    	stickL = new Joystick(0);	// initialize the left joystick on port 2
    	stickR = new Joystick(1);	// initialize  right joystick to port 3
       
    	
       // gamePad = new getGamePadButton(0,.1,.1,.1); // port and deadband values
        
        
      //  leftShooter = new Victor(2); //initialize the left shooter motor as a victor on channel 2
    	//rightShooter = new Victor(3); //initialize the right shooter motor as a victor on channel 3
    	//angleMotor = new Victor(5);		//initialize the motor that sets the shooting plate angle on channel 4
    	/*
    	ballPusher = new Victor(4); //pushes ball into shooting motors
    	
    	wedgeL = new Victor(0);
    	wedgeR = new Victor(0);
    	
    	wedge = new wedgeControl(wedgeL,wedgeR);
    
    	shoot = new Shooter(leftShooter,rightShooter,angleMotor,angleMotorEncoder, ballPusher);
    	*/
        
        
    }
    

    /**
     * This function is called periodically during autonomous
     */
    
    public void autonomousPeriodic() {

    
    	
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	
    	//Shooter.enable(); //enable shooter (safety protocol)
    	/*
    	
    	_mainCamera.getFrame();
    	_magicCamera.update();
    	
    	
    	//Get laser range and estimated camera range
    	int laserDistance = laser.getDistance();
    	double magicCameraDistance = _magicCamera.distance();
    	
    	/*
    	 * 
    	 */
    	if (stickL.getRawButton(1) && stickR.getRawButton(1)) { //buttons "0's" may not be the correct button, see if triggers are both down
    		drive.drive(stickL.getY()/2.0, stickR.getY()/2.0); // 1/2 speed
    	} else {
    		drive.drive(stickL.getY(), stickR.getY()); // full speed
    	}
    	
    	
    	
    	/*
    	if (gamePad.getLeftBumper()){//if left bumper is pressed down
    		motorSpeed = .01;
    		shooterAngleSet = - 5;
    		shoot.getBall(motorSpeed);
    	}
    	else if (gamePad.getRightBumper()){ //if right bumper is pressed down 
    		motorSpeed = .4;
    		shooterAngleSet = shoot.findAngle(laserDistance , motorSpeed);
        	
    	} else{
    		motorSpeed = 0; // sets motor speed to zero
    		
    		
    		
    		
    	}
    	
    	
    	
    	if(gamePad.getButtonX()){ //switch shootOverride between true false
	    		
    		shootOverride = !shootOverride;
    	} 
    	
    	if(shootOverride){
    		
    			if (gamePad.getButtonY()){ //OVERRIDE, finds button currently pressed down to determine shooter angle
	    		    shooterAngleSet = 10;
	    		} else if(gamePad.getButtonB()){
	    			shooterAngleSet = 5;
	    		} else if(gamePad.getButtonA()) {
	    			shooterAngleSet = 0;
	    		}
    	
    	} 
  
    	
    	
    	shoot.startMotors(motorSpeed); //start motors
    	shoot.adjustAngle(shooterAngleSet); //set the shooter angle
   
    	if(Math.abs(shooterAngleSet - shoot.testAngle()) < shooterAngleError){ // if the set angle equals the current shooter angle
    		shoot.pushBall(.1); //activate push motor
    	} else {
    		shoot.pushBall(0);
    	}
    	
     */
    /*
    if (gamePad.getButtonX()){
    	ballPusher.set(.1);
    } else {
    	ballPusher.set(0);
    }
    */
    
    }
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
