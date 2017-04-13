package robotics.challenge.one;

import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

public class ExploreIsland {

	public static void main(String[] args) {
		Behavior findLine = new findLine();
		Behavior followLine = new followLine(SensorPort.S1);
		Behavior corner = new Behavior() {
			
			@Override
			public boolean takeControl() {
				//takeControl if on corner
				return false;
			}
			
			@Override
			public void suppress() {
				//suppressed = true
				
			}
			
			@Override
			public void action() {
				//make a sound
				
			}
		};
		
		Behavior intersection = new Behavior() {
			
			@Override
			public boolean takeControl() {
				//takeControl if on intersection
				return false;
			}
			
			@Override
			public void suppress() {
				//suppressed = true
				
			}
			
			@Override
			public void action() {
				//make a sound
				
			}
		};
		
		Behavior bluePillar = new Behavior() {
			
			@Override
			public boolean takeControl() {
				//takeControl if sees blue pillar
				return false;
			}
			
			@Override
			public void suppress() {
				//suppressed = true
				
			}
			
			@Override
			public void action() {
				//remember the location of the blue pillar 
				
			}
		};
		
		Behavior [] bArray = {findLine, followLine};
		//Behavior [] bArray = {findLine,followLine,corner,intersection,bluePillar};
		
		Arbitrator arby = new Arbitrator(bArray);
		
		arby.start();
	}

}
