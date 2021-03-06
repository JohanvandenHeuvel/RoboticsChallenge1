package robotics.challenge.one;

import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.subsumption.Behavior;

public class followLine implements Behavior{
	boolean suppressed = false;
	EV3ColorSensor color;
	double threshold = 0.15;
	
	public followLine(EV3ColorSensor color)
	{
		this.color = color;
	}
	
	@Override
	public boolean takeControl() 
	{
		SampleProvider sampleprovider = color.getRedMode();
		float[] sample = new float[1];
		sampleprovider.fetchSample(sample, 0);
		return sample[0] > threshold;
	}
	
	@Override
	public void suppress() {
		suppressed = true;
		
	}
	
	@Override
	public void action() {
		
		suppressed = false;
		
		SampleProvider sampleprovider = color.getRedMode();
		float[] samplecolor = new float[1];

		double speed = 200;
		double white = 0.3;		//change
		double black = 0.05;		//change
		double avg_threshold = ( white - black ) / 2 + black;
		
		double Kp = 1000; 		//change
		double Ki = 0;			//change
		double Kd = 0;		//change
		
		double last_error = 0;
		double intergral = 0;
		
		while (!suppressed) {
			sampleprovider.fetchSample(samplecolor, 0);
			
			double error = avg_threshold - samplecolor[0];
			intergral = error + intergral;
			double derivative = error - last_error;
			double correction = Kp * error + Ki * intergral + Kd * derivative;
			
			last_error = error;
			
			if (samplecolor[0] < (0.3 *avg_threshold))
			{
				Motor.C.setSpeed((int) (speed - correction));
				Motor.C.backward();
				
				
				Motor.A.setSpeed((int) (speed + correction));
				Motor.A.forward();
				
			}
			else if (samplecolor[0] >= (1.6 * avg_threshold))
			{
				Motor.C.setSpeed((int) (speed - correction));
				Motor.C.forward();
				
				
				Motor.A.setSpeed((int) (speed + correction));
				Motor.A.backward();
			}
			else 
			{
				Motor.C.setSpeed((int) (speed - correction));
				Motor.C.forward();
				
				Motor.A.setSpeed((int) (speed + correction));
				Motor.A.forward();
				
				
			}
			
//			Motor.A.backward();
//			Motor.C.backward();
			
			//if(suppressed)
			//	suppress();
			Thread.yield();
			
		}
		
		Motor.A.stop(true);
		Motor.C.stop(true);
		
	}
}
