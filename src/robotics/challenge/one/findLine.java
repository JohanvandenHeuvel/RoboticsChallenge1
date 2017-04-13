package robotics.challenge.one;

import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.subsumption.Behavior;

public class findLine implements Behavior {
	boolean suppressed = false;

	@Override
	public boolean takeControl() {
		// TODO Auto-generated method stub
		return true;
	}
	
	@Override
	public void suppress() {
		suppressed = true;
		//should higher behaviors call suppress?
		
	}
	
	@Override
	public void action() {
		suppressed = false;
		
		Motor.A.setSpeed(360);
		Motor.C.setSpeed(360);

		
		Motor.A.forward();
		Motor.C.forward();
		
		while(!suppressed)
			Thread.yield();
		
		Motor.A.stop(true);
		Motor.C.stop(true);
		
	}

}
