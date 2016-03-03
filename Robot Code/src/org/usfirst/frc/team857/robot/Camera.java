package org.usfirst.frc.team857.robot;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.CameraServer;

public class Camera {
	
	int session;
    Image frame;

	public Camera(String name) {
		
	    // create images
        frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);

        // the camera name (ex "cam0") can be found through the roborio web interface
        session = NIVision.IMAQdxOpenCamera(name,
                NIVision.IMAQdxCameraControlMode.CameraControlModeController);
	}
	
	public Image getFrame() {
        NIVision.IMAQdxConfigureGrab(session);
		NIVision.IMAQdxStartAcquisition(session);
		
    	//read file in from disk. For this example to run you need to copy image.jpg from the SampleImages folder to the
		//directory shown below using FTP or SFTP: http://wpilib.screenstepslive.com/s/4485/m/24166/l/282299-roborio-ftp
        NIVision.IMAQdxGrab(session, frame, 1);
        CameraServer.getInstance().setImage(frame);
        NIVision.IMAQdxStopAcquisition(session);
        
        return frame;
	}
}
