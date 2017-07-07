package robotics.challenge.one;

import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.subsumption.Behavior;
import lejos.utility.Delay;

/**
 * Behavior that follows a line.
 * @author johan
 *
 */
public class FollowLine implements Behavior{
	boolean suppressed;
	
	EV3GyroSensor gyro;
	EV3ColorSensor color;
	
	final double THRESHOLD = 0.15;
	final int SPEED = 150;
	
	public FollowLine(EV3ColorSensor color, EV3GyroSensor gyro)
	{
		suppressed = false;
		this.color = color;
		this.gyro = gyro;
	}
	
	@Override
	public boolean takeControl() 
	{
//		return true;
		float sampleColor = readColorRedMode();
		return sampleColor > THRESHOLD;
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
	
	public float readColorRedMode()
	{
		float[] sample = new float[1];
		SampleProvider sampleProvider = color.getRedMode();
		sampleProvider.fetchSample(sample, 0);
		return sample[0];
	}
	
	public float readGyroAngle()
	{
		float[] sample = new float[1];
		SampleProvider sampleprovider = gyro.getAngleMode();
		sampleprovider.fetchSample(sample, 0);
		return sample[0];
	} 
	
	public double avgThreshold(double white, double black)
	{
		return ((white - black) / 2) + black;
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
	
	public void turnLeft()
	{
		motorsSpeed(SPEED,SPEED);

		Motor.A.forward();
		Motor.C.forward();
		Delay.msDelay(500);
		while(Math.abs(readGyroAngle()) < 90)
		{
			Motor.A.backward();
			Motor.C.forward();
//			Delay.msDelay(750);
		}
		
//		
//		Motor.A.backward();
//		Motor.C.forward();
//		Delay.msDelay(750);
		

		motorsStop();
	}
	
	public void turnRight()
	{
		motorsSpeed(SPEED,SPEED);
		Motor.C.backward();
		Motor.A.forward();
		Delay.msDelay(1000);
		motorsStop();
	}
	
	@Override
	public void action() 
	{
		unsuppress();
		
		turnLeft();

		//Color values
		double white = 0.4;		//change
		double black = 0.05;	//change
		double avgThreshold = avgThreshold(white, black);
		double lastSample = readColorRedMode();
		
		//PID-controller values
		double Kp = 1000; 		//change
		double Kd = 0;
		
		//PID-controller variables
		double lastError = 0;
		
		while (!suppressed) {
			float newSample = readColorRedMode();
			float avgSample = (float) ((newSample + lastSample) / 2);
			lastSample = newSample;
			
			//PID-controller calculations
			double newError = avgThreshold - avgSample;
			double derivative = newError - lastError;
			lastError = newError;
			
			//Normal PID-controller behavior
			int correction = (int) (Kp * newError + Kd * derivative);

			
//			//Turn faster if outside Bounds
			double lowerBound = 0.10; //0.35 * avgThreshold;
			double upperBound = 0.30; //1.35 * avgThreshold;
			
//			motorsSpeed(SPEED + correction, SPEED - correction);
//			motorsForward();
			
			if (avgSample < lowerBound)
			{
				//Turn right if on black
				motorsSpeed(SPEED - correction, SPEED + correction);
				Motor.C.backward();
				Motor.A.forward();
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
				motorsSpeed(SPEED + correction, SPEED - correction);
				motorsForward();
			}
		}
		motorsStop();
	}
}
