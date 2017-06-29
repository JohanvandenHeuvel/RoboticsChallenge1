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
	EV3ColorSensor color;
	
	
	
	final double THRESHOLD = 0.20;
	final int SPEED = 200;
	final int RED = 0;
	final int BLUE = 2;
	double white = 0.3;		//change
	double black = 0.05;	//change
	
	public BluePillar(EV3ColorSensor color, EV3UltrasonicSensor sonic) 
	{
		suppressed = false;
		this.sonic = sonic;
		this.color = color;
	}
	
	@Override
	/**
	 * Take control if pillar very close and color is blue
	 */
	public boolean takeControl() 
	{
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
		SampleProvider sampleProvider = color.getColorIDMode();
		sampleProvider.fetchSample(sample, 0);
		return sample[0];
	}
	
	public double avgThreshold(double white, double black)
	{
		return ((white - black) / 2) + black;
	}
	
	public float readColorRedMode()
	{
		float[] sample = new float[1];
		SampleProvider sampleProvider = color.getRedMode();
		sampleProvider.fetchSample(sample, 0);
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
	
	@Override
	public void action() {
		//Start maze
		pillarFound = true;
		
		unsuppress();
		motorsStop();
		playSound();
		playSound();

		//Color values
		double avgThreshold = avgThreshold(white, black);
		
		//PID-controller values
		double Kp = 1000; 		//change
		double Ki = 0;			//change
		double Kd = 0;			//change
		
		//PID-controller variables
		double lastError = 0;
		double intergral = 0;
		float lastSample = readColorRedMode();
		
		while (!suppressed) {
			float sample = readColorRedMode();
			float sampleColor = (float) (0.5 * lastSample + 0.5 * sample);
			lastSample = sampleColor;
			
			//PID-controller calculations
			double newError = avgThreshold - sampleColor;
			intergral = newError + intergral;
			double derivative = newError - lastError;
			int correction = (int) (Kp * newError + Ki * intergral + Kd * derivative);
			lastError = newError;
			
			//Normal PID-controller behavior
			
			
//			//Turn faster if outside Bounds
			double lowerBound = 0.10; //0.35 * avgThreshold;
			double upperBound = 0.25; //1.35 * avgThreshold;
			
			
			motorsForward();
			
			if (sampleColor < lowerBound)
			{
				//Turn left if on middle of tape
				motorsSpeed(SPEED - correction, SPEED + correction);
				Motor.A.backward();
				Motor.C.forward();
			}
			else if (sampleColor >= upperBound)
			{
				//Turn right if on left side of tape
				motorsSpeed(SPEED + correction, SPEED - correction);
				Motor.C.backward();
				Motor.A.backward();
			}
			else
			{
				motorsSpeed(SPEED - correction, SPEED + correction);
				motorsForward();
			}
		}
		
		motorsStop();
	}
}
