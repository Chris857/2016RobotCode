package org.usfirst.frc.team857.robot;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;

public class Shooter {
	
	/*
	private static PIDController leftShooter; // left shooter motor 
    private static PIDController rightShooter; //right shooter motor
    */
	private static SpeedController leftShooter;
	private static SpeedController rightShooter;
	private static SpeedController pushMotor;
    private static PIDController angleMotor; //motor that adjusts the angle of the shooting plate
    private Encoder angleEncoder; //encoder for the angle motor
    private Counter leftMotorCounter; // counter for the left shooter motor
    private Counter rightMotorCounter; // counter for the right shooter motor
    
    
   // private Relay shootingSolenoid; //relay that will control the shooting solenoid
	
	
	
    public Shooter(SpeedController leftShooter,SpeedController rightShooter, SpeedController angleMotor, 
			       Encoder angleEncoder, SpeedController pushMotor){
    	
    	
		this.angleEncoder = angleEncoder;
		
		//These lines set variables to themselves
		//this.leftMotorCounter = leftMotorCounter;
		//this.rightMotorCounter = rightMotorCounter;
		
    	//What exactly is the purpose of the following code?
		/*this.leftMotorCounter.setPIDSourceParameter(PIDSourceParameter.kRate); //set the rate for the left motor counter
		this.rightMotorCounter.setPIDSourceParameter(PIDSourceParameter.kRate); // set the rate for the right motor counter
		this.angleEncoder.setPIDSourceParameter(PIDSourceParameter.kAngle); //set the angle for the angle motor
		*/
    	
		Shooter.leftShooter = leftShooter; //left shooter
		Shooter.rightShooter = rightShooter; //right shooter
		Shooter.pushMotor = pushMotor;
		Shooter.angleMotor = new PIDController(0.8, 0.2, 0.1, 0.5, this.angleEncoder, angleMotor); //PIDController for the angle motor
		
		
		//Shooter.leftShooter = new PIDController(0.8, 0.2, 0.1, 0.5, this.leftMotorCounter, leftShooter); //create a PIDController, inputting leftMotorCounter and outputting to the left shooter
		//Shooter.rightShooter = new PIDController(0.8, 0.2, 0.1, 0.5, this.rightMotorCounter, rightShooter); //create a PIDController, inputting rightMotorCounter and outputting to the right shooter
		
		
		//this.shootingSolenoid = new Relay(2,Relay.Direction.kForward);
		
	}
     /*
    public static void enable(){ //enables motors
    	leftShooter.enable();
    	rightShooter.enable();
    	angleMotor.enable();
    	
    }
    
    public void disable(){ //disables motors
    	leftShooter.disable();
    	rightShooter.disable();
    	angleMotor.disable();
    }
    */
    
    public void startMotors(double motorSpeed){
    	
    	leftShooter.set(motorSpeed);
    	rightShooter.set(motorSpeed);
    	
    }
    
   /* public void startMotors(double motorSpeed){ //start motors to the speed of motorSpeed
    	leftShooter.setSetpoint(motorSpeed);
    	rightShooter.setSetpoint(motorSpeed);
    } */
	
    public double findAngle(int r, double Vi){ // range and initial velocity, REFER TO TOBIN'S EQUATION
		
    	if(r == 0 || Vi == 0.0){
    		//Meltdown
    		//Print error message, or fudge the values a bit
    	}
    	
    	double angle; // final result angle
		
		double y12_1; // y12 value
		double y12_2; 
		double ySmall; //y12 value that is smaller
		
		double h = 68; //total distance from the goal to the ball
		
		double k = 4.905 * ((Math.pow(r,2)) / (Math.pow(Vi, 2)));
		
		y12_1 = (r/k) + (Math.sqrt(Math.pow((r/k), 2))- (4*((h/k) +1))); // tobins formula sheet 3, quadratic formula
		y12_2 = (r/k) - (Math.sqrt(Math.pow((r/k), 2))- (4*((h/k) +1)));
		
		if(y12_1 < y12_2){ // find smaller angle 
			ySmall = y12_1;
		} else {
			ySmall = y12_2; 
		}
		
		double radians = Math.atan(ySmall); //inverse tan, find radian
		angle = radians * (180/Math.PI); //convert to degrees
		
		
				
    	return angle;
    	
    }
    
    public void adjustAngle(double setPoint){
    	
    	angleMotor.setSetpoint(setPoint);
    	
    }
    
    public double testAngle(){
    	return angleEncoder.get();
    }
    
    public void pushBall(double pushSpeed){
    	pushMotor.set(pushSpeed); // set speed of the push motor
    }
    
    public void getBall(double motorSpeed){
    	leftShooter.set(-motorSpeed);
    	rightShooter.set(-motorSpeed);
    	
    }
    
	

}
