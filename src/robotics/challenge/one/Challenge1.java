package robotics.challenge.one;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

public class Challenge1 {

	/**
	 * Grid task
	 * Maze task
	 * @param args
	 */
	public static void main(String[] args) {
		System.out.println("Starting..");
		
		//Sensors
		EV3ColorSensor color = new EV3ColorSensor(SensorPort.S1);
		EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S3);
		EV3UltrasonicSensor sonic = new EV3UltrasonicSensor(SensorPort.S4);
		System.out.println("Sensors loaded..");
		
		//Behaviors
		Behavior FindLine = new FindLine();
		Behavior FollowLine = new FollowLine(color);
		Behavior FollowLineInside = new FollowLineInside(color, gyro);
		Behavior FindPillar = new FindPillar(color, sonic);
		System.out.println("Behaviors loaded..");
		
		//Arbitrator
		Behavior [] bArray = {FindLine, FollowLine};
		Arbitrator arbitrator = new Arbitrator(bArray);
		arbitrator.start();
	}
	
	

}
