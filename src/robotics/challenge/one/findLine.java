package robotics.challenge.one;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.subsumption.Behavior;

public class findLine implements Behavior {
	boolean suppressed = false;

	@Override
	public boolean takeControl() {
		// TODO Auto-generated method stub
		return false;
	}
	
	@Override
	public void suppress() {
		suppressed = true;
		//should higher behaviors call suppress?
		
	}
	
	@Override
	public void action() {
		//Drive around randomly
		
	}

}
