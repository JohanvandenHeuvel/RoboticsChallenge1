package robotics.challenge.one;

import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.subsumption.Behavior;

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
	final int SPEED = 300;
	
	public FollowLine(EV3ColorSensor color, EV3GyroSensor gyro)
	{
		suppressed = false;
		this.color = color;
		this.gyro = gyro;
	}
	
	@Override
	public boolean takeControl() 
	{
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
	
	@Override
	public void action() 
	{
		unsuppress();

		//Color values
		double white = 0.3;		//change
		double black = 0.05;	//change
		double avgThreshold = avgThreshold(white, black);
		
		//PID-controller values
		double Kp = 1000; 		//change
		double Ki = 0;			//change
		double Kd = 0;			//change
		
		//PID-controller variables
		double lastError = 0;
		double intergral = 0;
		
		//############
		
		//############
		
		while (!suppressed) {
			float sampleColor = readColorRedMode();
			System.out.println(readGyroAngle());
			
			//PID-controller calculations
			double newError = avgThreshold - sampleColor;
			intergral = newError + intergral;
			double derivative = newError - lastError;
			int correction = (int) (Kp * newError + Ki * intergral + Kd * derivative);
			lastError = newError;
			
			//Normal PID-controller behavior
			motorsSpeed(SPEED + correction, SPEED - correction);
			
//			//Turn faster if outside Bounds
			double lowerBound = 0.3 * avgThreshold;
			double upperBound = 1.6 * avgThreshold;
//			
			if (sampleColor < lowerBound)
			{
				//Turn left if on middle of tape
				Motor.C.backward();
				Motor.A.forward();
			}
			else if (sampleColor >= upperBound)
			{
				//Turn right if on left side of tape
				Motor.A.backward();
				Motor.C.forward();
			}
			else
			{
				motorsForward();
			}
				
			
//			Thread.yield();
		}
		
		motorsStop();
	}
}
