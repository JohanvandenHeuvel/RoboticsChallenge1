package robotics.challenge.one;

import lejos.hardware.Audio;
import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.subsumption.Behavior;

public class corner implements Behavior {
	boolean suppressed = false;
	EV3GyroSensor gyro;
	EV3ColorSensor color;
	
	public corner(EV3ColorSensor color, EV3GyroSensor gyro)
	{
		this.gyro = gyro;
		this.color = color;
	}
	
	@Override
	public boolean takeControl() 
	{
		SampleProvider sampleprovider = gyro.getAngleMode();
		float[] sample = new float[1];
		sampleprovider.fetchSample(sample, 0);
		return Math.abs(sample[0]) > 270 ;
	}
	
	@Override
	public void suppress() {
		suppressed = true;
	}
	
	@Override
	public void action() {
		suppressed = false;
		
//		Motor.A.stop(true);
//		Motor.C.stop(true);
		
//		gyro.reset();
		
		EV3 ev3 = (EV3) BrickFinder.getDefault();
		Audio audio = ev3.getAudio();
		
		// Make EV3 beep
		
		//while(!suppressed)
		audio.systemSound(0);
		
		SampleProvider sampleprovider = color.getRedMode();
		float[] samplecolor = new float[1];

		double speed = 400;
		double white = 0.20;		//change
		double black = 0.05;		//change
		double avg_threshold = ( white - black ) / 2 + black;
		
		double Kp = 1500; 		//change
		double Ki = 0;		//change
		double Kd = 0;			//change
		
		double last_error = 0;
		double intergral = 0;
		
		while (!suppressed) {
			audio.systemSound(1);
			
			sampleprovider.fetchSample(samplecolor, 0);
			
			double error = avg_threshold - samplecolor[0];
			intergral = error + intergral;
			double derivative = error - last_error;
			double correction = Kp * error + Ki * intergral + Kd * derivative;
			
			last_error = error;
			
			Motor.A.setSpeed((int) (speed - correction));
			Motor.C.setSpeed((int) (speed + correction));
			
//			Motor.A.forward();
//			Motor.C.forward();
			
			Motor.A.backward();
			Motor.C.backward();
			
			//if(suppressed)
			//	suppress();
			Thread.yield();
			
		}
		
		Motor.A.stop(true);
		Motor.C.stop(true);
		
	}
}
