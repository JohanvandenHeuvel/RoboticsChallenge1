package robotics.challenge.one;

import lejos.hardware.Audio;
import lejos.hardware.BrickFinder;
import lejos.hardware.Sound;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.subsumption.Behavior;
import lejos.utility.Delay;

/**
 * Behavior that does an action when close to a blue pillar.
 * @author johan
 *
 */
public class RedPillar implements Behavior{
	boolean suppressed;
	boolean inRange = false;
	
	EV3UltrasonicSensor sonic;
//	EV3ColorSensor color;
	
	//SampleProvider colorRed;
	SampleProvider colorID;
	
	final double THRESHOLD = 0.25;
	final int SPEED = 200;
	final float RED = 0;
	final float BLUE = 2;
	
	public RedPillar(EV3ColorSensor color, EV3UltrasonicSensor sonic) 
	{
		suppressed = false;
		this.sonic = sonic;
//		this.color = color;
		
		this.colorID = color.getColorIDMode();
		//this.colorRed = color.getRedMode();
	}
	
	@Override
	/**
	 * Take control if pillar very close and color is blue
	 */
	public boolean takeControl() 
	{
		inRange = readUltraSonic() < THRESHOLD;
		return inRange && readColorIDMode() == RED;
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
	
	public float readUltraSonic()
	{
		float[] sample = new float[1];
		SampleProvider sampleProvider = sonic.getDistanceMode();
		sampleProvider.fetchSample(sample, 0);
		return sample[0];
	}
	
	public float readColorIDMode()
	{
		float[] sample = new float[1];
//		SampleProvider sampleProvider = color.getColorIDMode();
		colorID.fetchSample(sample, 0);
		return sample[0];
	}
	
	public void playSound()
	{
		EV3 ev3 = (EV3) BrickFinder.getDefault();
		Audio audio = ev3.getAudio();
		Sound.beepSequenceUp();
//		audio.systemSound(0);
//		File file = new File("sound.wav");
//		System.out.println(file.exists());
//		System.out.println(Sound.playSample(file, 100));
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
	public void action() {
		unsuppress();
		motorsStop();
		
		EV3 ev3 = (EV3) BrickFinder.getDefault();
		Audio audio = ev3.getAudio();
		
		audio.playTone(440, 125, 80);
		audio.playTone(392, 125, 80);
		audio.playTone(440, 500, 80);
		audio.playTone(294, 500, 80);
		Delay.msDelay(750);
		audio.playTone(466, 125, 80);
		audio.playTone(440, 125, 80);
		audio.playTone(466,	250, 80);
		audio.playTone(440, 250, 80);
		audio.playTone(392, 500, 80);
		System.out.println("RED");
	}
}
