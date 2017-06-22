package robotics.challenge.one;

import lejos.hardware.Audio;
import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.subsumption.Behavior;

/**
 * Behavior that drives inside of the grid to find the center waypoint.
 * Takes control if driven full circle.
 * @author johan
 *
 */
public class FollowLineInside implements Behavior {
	boolean suppressed = false;
	EV3GyroSensor gyro;
	EV3ColorSensor color;
	
	final int SPEED = 200;
	
	public FollowLineInside(EV3ColorSensor color, EV3GyroSensor gyro)
	{
		this.gyro = gyro;
		this.color = color;
	}
	
	@Override
	public boolean takeControl() 
	{
		float sampleGyro = readGyroAngle();
		return Math.abs(sampleGyro) > 270 ;
	}
	
	@Override
	public void suppress() 
	{
		suppressed = true;
	}
	
	public void unsuppress()
	{
		suppressed = false;
	}
	
	public float readGyroAngle()
	{
		float[] sample = new float[1];
		SampleProvider sampleprovider = gyro.getAngleMode();
		sampleprovider.fetchSample(sample, 0);
		return sample[0];
	} 
	
	public float readColorRedMode()
	{
		float[] sample = new float[1];
		SampleProvider sampleProvider = color.getRedMode();
		sampleProvider.fetchSample(sample, 0);
		return sample[0];
	}
	
	public void playSound()
	{
		EV3 ev3 = (EV3) BrickFinder.getDefault();
		Audio audio = ev3.getAudio();
		audio.systemSound(0);
	}
	
	public void motorsStop()
	{
		Motor.A.stop(true);
		Motor.C.stop(true);
	}
	
	public void motorsForward()
	{
		Motor.A.forward();
		Motor.C.forward();
	}
	
	public void motorsSpeed(int speedA, int speedC)
	{
		Motor.A.setSpeed(speedA);
		Motor.C.setSpeed(speedC);
	}
	
	public double avgThreshold(double white, double black)
	{
		return ((white - black) / 2) + black;
	}
	
	@Override
	public void action() 
	{
		unsuppress();

		//Color values
		double white = 0.3;		//change
		double black = 0.05;		//change
		double avgThreshold = avgThreshold(white, black);
		
		//PID-controller values
		double Kp = 1000; 		//change
		double Ki = 0;			//change
		double Kd = 0;		//change
		
		//PID-controller variables
		double lastError = 0;
		double intergral = 0;
		
		while (!suppressed) {
			float sampleColor = readColorRedMode();
			
			//PID-controller calculations
			double newError = avgThreshold - sampleColor;
			intergral = newError + intergral;
			double derivative = newError - lastError;
			int correction = (int) (Kp * newError + Ki * intergral + Kd * derivative);
			lastError = newError;
			
			//Normal PID-controller behavior
			motorsSpeed(SPEED - correction, SPEED + correction);
			motorsForward();
			
			//Turn faster if outside Bounds
			double lowerBound = 0.3 * avgThreshold;
			double upperBound = 1.6 * avgThreshold;
			
			if (sampleColor < lowerBound)
				//Turn ... if on ...
				Motor.C.backward();
			else if (sampleColor >= upperBound)
				//Turn ... if on ...
				Motor.A.backward();
			
			Thread.yield();
		}
		
		motorsStop();
	}
}
