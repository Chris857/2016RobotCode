package org.usfirst.frc.team857.robot;

import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

/**
 * LidarLite Distance Sensor Class
 */
public class LidarSubsystem extends SensorBase{
	    
    I2C lidar;
         
    public LidarSubsystem(I2C.Port port) {  
    	lidar = new I2C(port, 0x62);  	          	  
    }

	public int getDistance() {
 
		byte[] buffer;
		int distance;
		buffer = new byte[2];
    	
		lidar.write(0x00, 0x04);
		
		Timer.delay(0.1);
		
		lidar.read(0x8f, 2, buffer);

		distance = (int)Integer.toUnsignedLong(buffer[0] << 8) + Byte.toUnsignedInt(buffer[1]);	
		
		//Check if distance is in a valid range
		if (distance < 0 || distance > 4000) {
			return 0;
		}
		return distance;
	}
}

