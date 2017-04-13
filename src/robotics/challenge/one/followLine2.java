package robotics.challenge.one;

import lejos.hardware.Audio;
import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.subsumption.Behavior;

public class followLine2 implements Behavior{
	boolean suppressed = false;
	EV3ColorSensor color;
	double threshold = 0.15;
	
	public followLine2(Port port)
	{
		color = new EV3ColorSensor(port);
	}
	
	@Override
	public boolean takeControl() 
	{
		SampleProvider sampleprovider = color.getRedMode();
		float[] sample = new float[1];
		sampleprovider.fetchSample(sample, 0);
		color.close();
		return sample[0] > threshold;
	}
	
	@Override
	public void suppress() {
		suppressed = true;
		
	}
	
	@Override
	public void action() {
		suppressed = false;
		
		Motor.C.setSpeed(240);
		Motor.C.forward();
		
		while(!suppressed)
			Thread.yield();
		
		Motor.C.stop();
		//EV3 ev3 = (EV3) BrickFinder.getDefault();
		//Audio audio = ev3.getAudio();
		//audio.systemSound(0);
		
	}
}
