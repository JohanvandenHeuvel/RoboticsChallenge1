package robotics.challenge.one;

import lejos.hardware.Audio;
import lejos.hardware.BrickFinder;
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
public class BluePillar implements Behavior{
	boolean suppressed;
	boolean inRange = false;
	boolean pillarFound = false;
	
	EV3UltrasonicSensor sonic;
//	EV3ColorSensor color;
	EV3GyroSensor gyro;
	
	SampleProvider colorRed;
	SampleProvider colorID;
	
	final double THRESHOLD = 0.20;
	final int SPEED = 200;
	final int RED = 0;
	final int BLUE = 2;
	double WHITE = 0.3;		//change
	double BLACK = 0.03;	//change
	
	public BluePillar(EV3ColorSensor color, EV3UltrasonicSensor sonic, EV3GyroSensor gyro) 
	{
		suppressed = false;
		this.sonic = sonic;
//		this.color = color;
		this.gyro = gyro;
		
		this.colorID = color.getColorIDMode();
		this.colorRed = color.getRedMode();
	}
	
	@Override
	/**
	 * Take control if pillar very close and color is blue
	 */
	public boolean takeControl() 
	{
//		return true;
		if(pillarFound)
			return true;
		inRange = readUltraSonic() < THRESHOLD;
		return inRange && readColorIDMode() == BLUE;
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
	
	public double avgThreshold(double white, double black)
	{
		return ((white - black) / 2) + black;
	}
	
	public float readColorRedMode()
	{
		float[] sample = new float[1];
//		SampleProvider sampleProvider = color.getRedMode();
		colorRed.fetchSample(sample, 0);
		return sample[0];
	}
	
	public void playSound()
	{
		EV3 ev3 = (EV3) BrickFinder.getDefault();
		Audio audio = ev3.getAudio();
		audio.systemSound(0);
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
	
	public float readGyroAngle()
	{
		float[] sample = new float[1];
		SampleProvider sampleprovider = gyro.getAngleMode();
		sampleprovider.fetchSample(sample, 0);
		return sample[0];
	} 
	
	public void turnRight()
	{
//		motorsSpeed(0,SPEED);
//		Motor.A.forward();
//		Motor.C.backward();
//		Delay.msDelay(750);
//		while(Math.abs(readGyroAngle()) < 90)
//		{
			
//		}
		
//		
//		Motor.A.backward();
//		Motor.C.forward();
//		Delay.msDelay(750);
		

		motorsStop();
	}
	
	@Override
	public void action() {
		//Start maze
		playSound();
		pillarFound = true;
		
		unsuppress();
		
		turnRight();
		System.out.println("Continue..");

		//Color values
		double avgThreshold = avgThreshold(WHITE, BLACK);
		double lastSample = readColorRedMode();
		
		//PID-controller values
		double Kp = 1000; 		//change
		
		while (!suppressed) {
			float newSample = readColorRedMode();
			float avgSample = (float) ((newSample + lastSample) / 2);
			lastSample = newSample;
			
			//PID-controller calculations
			double newError = avgThreshold - avgSample;
			
			//Normal PID-controller behavior
			int correction = (int) (Kp * newError);

			
//			//Turn faster if outside Bounds
			double lowerBound = 0.10; //0.35 * avgThreshold;
			double upperBound = 0.20; //1.35 * avgThreshold;
			
//			motorsSpeed(SPEED + correction, SPEED - correction);
//			motorsForward();
			
			if (avgSample >= upperBound)
			{
				//Turn right if on middle of tape
				motorsSpeed(SPEED - correction, SPEED + correction);
				Motor.C.backward();
				Motor.A.stop();
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
				motorsSpeed(SPEED - correction, SPEED + correction);
				motorsForward();
			}
		}
		motorsStop();
	}
}
