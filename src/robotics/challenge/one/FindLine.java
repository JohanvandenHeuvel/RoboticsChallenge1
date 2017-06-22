package robotics.challenge.one;

import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.subsumption.Behavior;

/**
 * Initial behavior, drive straight ahead.
 * @author johan
 *
 */
public class FindLine implements Behavior {
	boolean suppressed;
	
	final int SPEED = 200;
	
	public FindLine() 
	{
		suppressed = false;
	}

	@Override
	public boolean takeControl() 
	{
		return true;
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
		
		while(!suppressed)
		{
			motorsSpeed(SPEED, SPEED);
			motorsForward();
			Thread.yield();
		}
			
		motorsStop();
	}

}
