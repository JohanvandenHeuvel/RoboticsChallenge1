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

		int CONSTANT = 240;
		int CONSTANT_TWO = 0;
		
		while (!suppressed) {

			
			//-0.000208333x A
			//0.000208333x
			sampleprovider.fetchSample(sample, 0);
			if (sample[0] > 0.20) {
				Motor.A.setSpeed(CONSTANT_TWO);
				Motor.C.setSpeed(CONSTANT);

				Motor.A.forward();
				Motor.C.forward();
			}

			else if (sample[0] < 0.15) {
				Motor.C.setSpeed(CONSTANT_TWO);
				Motor.A.setSpeed(CONSTANT);

				Motor.A.forward();
				Motor.C.forward();
			} 
			
			else {
				Motor.C.setSpeed(CONSTANT);
				Motor.A.setSpeed(CONSTANT);

				Motor.A.forward();
				Motor.C.forward();
			} 
			
			
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
