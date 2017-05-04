package robotics.challenge.one;

import lejos.hardware.Audio;
import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.subsumption.Behavior;

public class followLine implements Behavior{
	boolean suppressed = false;
	EV3ColorSensor color;
	double threshold = 0.20;
	
	public followLine(Port port)
	{
		color = new EV3ColorSensor(port);
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
		float[] sample = new float[1];

		//int CONSTANT = 240;
		//int CONSTANT_ZERO = 0;
		double speed = 300;
		double threshold1 = 0.20;
		double threshold2 = 0.15;
		double avg_threshold = (threshold1+threshold2)/2;
		
		double Kp = 1000; 
		double Ki = 0;
		double Kd = 0;
		
		double last_error = 0;
		double intergral = 0;
		
		
		while (!suppressed) {
			sampleprovider.fetchSample(sample, 0); //get sensor value
			
			//0.025 * 500 = 12.5
			
			double error = avg_threshold - sample[0];
			intergral = error + intergral;
			double derivative = error - last_error;
			double correction = Kp * error + Ki * intergral + Kd * derivative;
			
			last_error = error;
			
			Motor.A.setSpeed((int) (speed - correction));
			Motor.C.setSpeed((int) (speed + correction));
			
			Motor.A.forward();
			Motor.C.forward();
			
			/*
			if (sample[0] > threshold1) {
				Motor.A.setSpeed(CONSTANT);
				Motor.C.setSpeed(CONSTANT_ZERO);

				Motor.A.forward();
				Motor.C.forward();
			}

			else if (sample[0] < threshold2) {
				Motor.C.setSpeed(CONSTANT);
				Motor.A.setSpeed(CONSTANT_ZERO);

				Motor.A.forward();
				Motor.C.forward();
			} 
			
			else {
				Motor.C.setSpeed(CONSTANT);
				Motor.A.setSpeed(CONSTANT);

				Motor.A.forward();
				Motor.C.forward();
			} 
			*/
			
			//	suppress();
			Thread.yield();
		}
		
		Motor.A.stop(true);
		Motor.C.stop(true);

		
		//EV3 ev3 = (EV3) BrickFinder.getDefault();
		//Audio audio = ev3.getAudio();
		//audio.systemSound(0);
		
	}
}
