package robotics.challenge.one;

import javax.swing.plaf.basic.BasicTableHeaderUI;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
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
		Behavior FollowLine = new FollowLine(color, gyro);
		Behavior FollowLineInside = new FollowLineInside(color, gyro);
//		Behavior FindPillar = new FindPillar(gyro, color, sonic);
//		Behavior BluePillar = new BluePillar(color, sonic);
//		Behavior RedPillar = new RedPillar(color, sonic);
		System.out.println("Behaviors loaded..");
		 
		
		//Arbitrator
//		Behavior [] bArray = {FindLine, FollowLine, FollowLineInside, FindPillar, BluePillar, RedPillar};
		Behavior [] bArray = {FindLine, FollowLine, FollowLineInside};
		
		Arbitrator arbitrator = new Arbitrator(bArray);
		arbitrator.start();
	}
}
