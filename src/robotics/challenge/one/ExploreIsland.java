package robotics.challenge.one;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

public class ExploreIsland {

	public static void main(String[] args) {
		System.out.println("Starting..");
		
		EV3ColorSensor color = new EV3ColorSensor(SensorPort.S1);
		EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S3);
		
		Behavior findLine = new findLine();
		Behavior followLine = new followLine(color);
		Behavior corner = new corner(color, gyro);
		
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
		
		Arbitrator arby = new Arbitrator(bArray);
		
		arby.start();
	}

}
