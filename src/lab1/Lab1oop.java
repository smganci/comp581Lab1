package lab1;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class Lab1oop {

	
	//measuring points
	//absolute tip is 2.4 cm from sonic
	//pushed in 2.0 cm
	public static void main(String[] args) {

		/*Create new Charlie object with following specs:
		 * left motor: MotorPort.B
		 * right motor: MotorPort.C
		 * left touch: SensorPort.S1
		 * right touch: SensorPort.S4
		 * sonic sensor: SensorPort.S2
		 * left wheel radius: 0.028
		 * right wheel radius: 0.028
		*/
		Charlie charlie= new Charlie(new EV3LargeRegulatedMotor(MotorPort.B), 
				new EV3LargeRegulatedMotor(MotorPort.C), 
				new EV3TouchSensor(SensorPort.S1), 
				new EV3TouchSensor(SensorPort.S4), 
				new EV3UltrasonicSensor(SensorPort.S2),
				0.028,
				0.028);

		//Objective 1: Move forward 1.2m
		System.out.println("Objective 1");
		System.out.println("Press Button to Start");
		charlie.buttonWait();
		charlie.setBothSpeed(360);
		charlie.syncMotors();
		charlie.moveForwardDist(1.2);
		charlie.beep();
		
		//Objective 2: Move forward till sensing obstacle .5 meters away
		System.out.println("Objective 2");
		System.out.println("Press Button to Continue");
		charlie.buttonWait();
		charlie.moveTillSense(0.5);
		charlie.beep();
		
		//Objective 3: Move forward till hit wall, stop, reverse .5 meters
		System.out.println("Objective 3");
		System.out.println("Press Button to Continue");
		charlie.buttonWait();
		charlie.moveTillTouch();
		charlie.moveBackwardDist(.5);
	}

}
