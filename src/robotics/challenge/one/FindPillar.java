package robotics.challenge.one;

import java.io.File;

import lejos.hardware.Audio;
import lejos.hardware.BrickFinder;
import lejos.hardware.Sound;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.subsumption.Behavior;

/**
 * Behavior that finds a pillar.
 * @author johan
 *
 */
public class FindPillar implements Behavior{
	boolean suppressed;
	boolean inRange = false;
	
	EV3UltrasonicSensor sonic;
	EV3ColorSensor color;
	
	final double THRESHOLD = 0.08;
	final int SPEED = 200;
	final int RED = 5;
	final int BLUE = 2;
	
	public FindPillar(EV3ColorSensor color, EV3UltrasonicSensor sonic) 
	{
		suppressed = false;
		this.sonic = sonic;
		this.color = color;
	}
	
	@Override
	public boolean takeControl() 
	{
		return !inRange;
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
		SampleProvider sampleProvider = color.getColorIDMode();
		sampleProvider.fetchSample(sample, 0);
		return sample[0];
	}
	
	public void playSound()
	{
//		EV3 ev3 = (EV3) BrickFinder.getDefault();
//		Audio audio = ev3.getAudio();
//		audio.systemSound(0);
		File file = new File("sound.wav");
		System.out.println(file.exists());
		System.out.println(Sound.playSample(file, 100));
	}
	
	public void inRange()
	{
		inRange = true;
		suppress();
		playSound();
		
		float sampleColor = readColorIDMode();
		if (sampleColor == RED)
			System.out.println("RED");
		if (sampleColor == BLUE)
			System.out.println("BLUE");
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
		
		System.out.println("Playing sound..");
		playSound();
		System.out.println("Done");
		
		while (!suppressed) {
			float sampleUltraSonic = readUltraSonic();
			
			if(sampleUltraSonic < THRESHOLD)
			{
				inRange();
			}
			else
			{
				/**
				 * Turn if no object in range
				 * Forward if object in range
				 */
				int speedMotorA = (sampleUltraSonic > 100) ? 0 : SPEED;
				int speedMotorC = SPEED;
				
				motorsSpeed(speedMotorA, speedMotorC);
				motorsForward();
			
				Thread.yield();
			}
		}
		
		motorsStop();
	}
}
