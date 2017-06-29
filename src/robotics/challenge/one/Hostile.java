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
import lejos.utility.Stopwatch;

/**
 * Behavior that follows a line.
 * @author johan
 *
 */
public class Hostile implements Behavior{
	boolean suppressed;
	
	EV3GyroSensor gyro;
	EV3ColorSensor color;
	
	final double THRESHOLD = 0.15;
	final int SPEED = 150;
	
	public Hostile(EV3ColorSensor color, EV3GyroSensor gyro)
	{
		suppressed = false;
		this.color = color;
		this.gyro = gyro;
	}
	
	@Override
	public boolean takeControl() 
	{
		return true;
//		float sampleColor = readColorRedMode();
//		return sampleColor > THRESHOLD;
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
		Motor.A.backward();
		Motor.C.forward();
		try {
			Thread.sleep (500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		motorsStop();
	}
	
	public void turnRight()
	{
		motorsSpeed(SPEED,SPEED);
		Motor.C.backward();
		Motor.A.forward();
		try {
			Thread.sleep (500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		motorsStop();
	}
	
	@Override
	public void action() 
	{
		unsuppress();
		
		Motor.C.backward();
		Motor.A.backward();
		Delay.msDelay(1000);
		Motor.A.stop(true);
		Motor.C.stop(true);
		
		EV3 ev3 = (EV3) BrickFinder.getDefault();
		Audio audio = ev3.getAudio();
		
		audio.playTone(250, 1000, 80);
		audio.playTone(265, 1000, 80);
		audio.playTone(250, 1000, 80);
		audio.playTone(265, 1000, 80);
		
		Motor.C.forward();
		Motor.A.forward();
		Delay.msDelay(1000);
		Motor.A.stop(true);
		Motor.C.stop(true);
		
		
		
		motorsStop();
	}
}
