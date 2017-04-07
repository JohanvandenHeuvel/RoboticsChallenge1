package robotics.challenge.one;

import lejos.hardware.Audio;
import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.subsumption.Behavior;

public class followLine implements Behavior{
	boolean suppressed = false;
	EV3ColorSensor color;
	double threshold = 0;
	
	public followLine(Port port)
	{
		color = new EV3ColorSensor(port);
	}
	
	@Override
	public boolean takeControl() {
		SampleProvider sampleprovider = color.getColorIDMode();
		float[] sample = new float[1];
		sampleprovider.fetchSample(sample, 0);
		return sample[0] <= threshold;
	}
	
	@Override
	public void suppress() {
		suppressed = true;
		
	}
	
	@Override
	public void action() {
		EV3 ev3 = (EV3) BrickFinder.getDefault();
		Audio audio = ev3.getAudio();
		audio.systemSound(0);
		
		//follow the white line
		
	}
}
