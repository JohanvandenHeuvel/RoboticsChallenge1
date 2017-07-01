package robotics.challenge.one;

import lejos.hardware.Audio;
import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.subsumption.Behavior;
import lejos.utility.Delay;

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
	
	final int SPEED = 150;
	final double WHITE = 0.3;		
	final double BLACK = 0.05;	
	
	public FollowLineInside(EV3ColorSensor color, EV3GyroSensor gyro)
	{
		this.gyro = gyro;
		this.color = color;
	}
	
	@Override
	public boolean takeControl() 
	{
//		return true;
		float sampleGyro = readGyroAngle();
		return Math.abs(sampleGyro) >= 270 ;
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
	
	public void turnRight()
	{
		motorsSpeed(SPEED,SPEED);
		Motor.C.backward();
		Motor.A.forward();
		Delay.msDelay(1500);
		motorsStop();
	}
	
	public double avgThreshold(double white, double black)
	{
		return ((white - black) / 2) + black;
	}
	
	@Override
	public void action() 
	{
		unsuppress();
		
		turnRight();
		System.out.println("Continue..");

		//Color values
		double avgThreshold = avgThreshold(WHITE, BLACK);
		double lastSample = readColorRedMode();
		
		//PID-controller values
		double Kp = 1000; 		//change
		
		while (!suppressed) {
			float newSample = readColorRedMode();
			float avgSample = (float) ((newSample + lastSample) / 2);
			lastSample = newSample;
			
			//PID-controller calculations
			double newError = avgThreshold - avgSample;
			
			//Normal PID-controller behavior
			int correction = (int) (Kp * newError);

			
//			//Turn faster if outside Bounds
			double lowerBound = 0.10; //0.35 * avgThreshold;
			double upperBound = 0.25; //1.35 * avgThreshold;
			
//			motorsSpeed(SPEED + correction, SPEED - correction);
//			motorsForward();
			
			if (avgSample >= upperBound)
			{
				//Turn right if on middle of tape
				motorsSpeed(SPEED + correction, SPEED - correction);
				Motor.A.stop();
				Motor.C.backward();
			}
//			else if (avgSample >= upperBound)
//			{
//				//Turn left if on middle of tape
//				motorsSpeed(SPEED - correction, SPEED + correction);
//				Motor.A.backward();
//				Motor.C.backward();
//			}
			else
			{
				motorsSpeed(SPEED - correction, SPEED + correction);
				motorsForward();
			}
		}
		motorsStop();
	}
}
