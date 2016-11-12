package org.usfirst.frc.team857.robot;

import java.lang.Math;
import java.util.Comparator;
import java.util.Vector;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;

import edu.wpi.first.wpilibj.CameraServer;


/**
 * 
 * @author candlewit
 *
 */
public class Computer_Vision {
	
		//A structure to hold measurements of a particle
		public class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport>{
			double PercentAreaToImageArea;
			double Area;
			double BoundingRectLeft;
			double BoundingRectTop;
			double BoundingRectRight;
			double BoundingRectBottom;
			
			public int compareTo(ParticleReport r)
			{
				return (int)(r.Area - this.Area);
			}
			
			public int compare(ParticleReport r1, ParticleReport r2)
			{
				return (int)(r1.Area - r2.Area);
			}
		};

		//Structure to represent the scores for the various tests used for target identification
		public class Scores {
			double Area;
			double Aspect;
		};

		
		int session;
	    Image frame;

		//Images
		Image binaryFrame;
		int imaqError;
		
		private boolean _goal;
		private double _distance;
		private double _center;
		
		//Constants
		NIVision.Range TOTE_HUE_RANGE = new NIVision.Range(50, 120);	//Default hue range for green light
		NIVision.Range TOTE_SAT_RANGE = new NIVision.Range(88, 255);	//Default saturation range for green light
		NIVision.Range TOTE_VAL_RANGE = new NIVision.Range(134, 255);	//Default value range for green light
		double AREA_MINIMUM = 0.5; //Default Area minimum for particle as a percentage of total image area
		double LONG_RATIO = 2.22; //Goal long side = 26.9 / Goal height = 12.1 = 2.22
		double SHORT_RATIO = 1.4; //Goal short side = 16.9 / Goal height = 12.1 = 1.4
		double SCORE_MIN = 75.0;  //Minimum score to be considered a goal
		double VIEW_ANGLE = 52; //68.5 according to ms, 52 for HD3000 square, 60 for HD3000 640x480
		NIVision.ParticleFilterCriteria2 criteria[] = new NIVision.ParticleFilterCriteria2[1];
		NIVision.ParticleFilterOptions2 filterOptions = new NIVision.ParticleFilterOptions2(0,0,1,1);
		Scores scores = new Scores();

		private Camera _camera;
		
		public Computer_Vision(String name) {
			
			_camera = new Camera(name);
			
		    // create images
	        frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
			
			binaryFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
			criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA, AREA_MINIMUM, 100.0, 0, 0);
			
		}
		
		public void update() {
			frame = _camera.getFrame();
         
			//Threshold the image looking for yellow (tote color)
			NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSV, TOTE_HUE_RANGE, TOTE_SAT_RANGE, TOTE_VAL_RANGE);

			//Send particle count to dashboard
			int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);

			//Send masked image to dashboard to assist in tweaking mask.
			CameraServer.getInstance().setImage(binaryFrame);

			//filter out small particles
			criteria[0].lower = (float) AREA_MINIMUM;
			imaqError = NIVision.imaqParticleFilter4(binaryFrame, binaryFrame, criteria, filterOptions, null);

			//Send particle count after filtering to dashboard
			numParticles = NIVision.imaqCountParticles(binaryFrame, 1);

			if(numParticles > 0)
			{
				//Measure particles and sort by particle size
				Vector<ParticleReport> particles = new Vector<ParticleReport>();
				for(int particleIndex = 0; particleIndex < numParticles; particleIndex++)
				{
					ParticleReport par = new ParticleReport();
					par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
					par.Area = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA);
					par.BoundingRectTop = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
					par.BoundingRectLeft = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
					par.BoundingRectBottom = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
					par.BoundingRectRight = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
					particles.add(par);
				}
				particles.sort(null);

				//This example only scores the largest particle. Extending to score all particles and choosing the desired one is left as an exercise
				//for the reader. Note that this scores and reports information about a single particle (single L shaped target). To get accurate information 
				//about the location of the tote (not just the distance) you will need to correlate two adjacent targets in order to find the true center of the tote.
				scores.Aspect = AspectScore(particles.elementAt(0));
				scores.Area = AreaScore(particles.elementAt(0));
				_goal = scores.Aspect > SCORE_MIN && scores.Area > SCORE_MIN;
				_distance = computeDistance(binaryFrame, particles.elementAt(0));

				_center = computeCenter(binaryFrame, particles.elementAt(0));
			} else {
				_goal = false;
				_center = 0;
			}
		}

		public boolean goal() {
			return _goal;
		}
		
		public double distance() {
			return _distance;
		}
		
		public double center() {
			return _center;
		}
		//Comparator function for sorting particles. Returns true if particle 1 is larger
		static boolean CompareParticleSizes(ParticleReport particle1, ParticleReport particle2)
		{
			//we want descending sort order
			return particle1.PercentAreaToImageArea > particle2.PercentAreaToImageArea;
		}

		/**
		 * Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
		 * linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside the range 0-2
		 */
		double ratioToScore(double ratio)
		{
			return (Math.max(0, Math.min(100*(1-Math.abs(1.4285714285714285714285714285714-ratio)), 100)));
		}

		double AreaScore(ParticleReport report)
		{
			double boundingArea = (report.BoundingRectBottom - report.BoundingRectTop) * (report.BoundingRectRight - report.BoundingRectLeft);
			//Tape is 7" edge so 49" bounding rect. With 2" wide tape it covers 24" of the rect.
			return ratioToScore((5.0)*(report.Area/boundingArea));
		}

		/**
		 * Method to score if the aspect ratio of the particle appears to match the retro-reflective target. Target is 7"x7" so aspect should be 1
		 */
		double AspectScore(ParticleReport report)
		{
			return ratioToScore(((report.BoundingRectRight-report.BoundingRectLeft)/(report.BoundingRectBottom-report.BoundingRectTop)));
		}

		/**
		 * Computes the estimated distance to a target using the width of the particle in the image. For more information and graphics
		 * showing the math behind this approach see the Vision Processing section of the ScreenStepsLive documentation.
		 *
		 * @param image The image to use for measuring the particle estimated rectangle
		 * @param report The Particle Analysis Report for the particle
		 * @return The estimated distance to the target in inches
		 */
		double computeDistance (Image image, ParticleReport report) {
			double normalizedWidth, targetWidth;
			NIVision.GetImageSizeResult size;

			size = NIVision.imaqGetImageSize(image);
			normalizedWidth = 2*(report.BoundingRectRight - report.BoundingRectLeft)/size.width;
			targetWidth = 20;

			return  targetWidth/(normalizedWidth*Math.tan(VIEW_ANGLE*Math.PI/(180*2)));
		}
		
		double computeCenter(Image image, ParticleReport report) {
			NIVision.GetImageSizeResult size;

			size = NIVision.imaqGetImageSize(image);

			return ((report.BoundingRectRight - report.BoundingRectLeft) / 2.0) + report.BoundingRectLeft - (size.width / 2.0);
			
		}
}
