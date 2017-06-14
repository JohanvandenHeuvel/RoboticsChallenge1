package robotics.challenge.one;

import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.subsumption.Behavior;

public class findLine implements Behavior {
	boolean suppressed = false;

	@Override
	public boolean takeControl() {
		return true;
	}
	
	@Override
	public void suppress() {
		suppressed = true;
	}
	
	@Override
	public void action() {
		suppressed = false;
		
		Motor.A.setSpeed(360);
		Motor.C.setSpeed(360);

		
		Motor.A.forward();
		Motor.C.forward();
		
//		Motor.A.backward();
//		Motor.C.backward();
		
		while(!suppressed)
			Thread.yield();
		
		Motor.A.stop(true);
		Motor.C.stop(true);
		
	}

}
