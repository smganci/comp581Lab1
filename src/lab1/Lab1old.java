package lab1;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;
//test

//sadlfkadsjf;
public class Lab1old {

	public static void main(String[] args) {
		// TODO Auto-generated method stub

		/*Objective 1:
		 * press button
		 * move forward
		 * play tone
		 * */
		//wait for button to press
		Button.ENTER.waitForPressAndRelease();

		EV3MediumRegulatedMotor mB = new EV3MediumRegulatedMotor(MotorPort.B);
		EV3MediumRegulatedMotor mC = new EV3MediumRegulatedMotor(MotorPort.C);
 	  mB.synchronizeWith(new EV3MediumRegulatedMotor[] {mC});

		//r radius should be roughly

		while() {
		  mB.forward();
		  mC.forward();

		}

		Sound.playTone();
	}

	//move time take in
	//angularv (angular velocity in radians per second)
	//dist (desired distance in meters)
	//r (radius of wheel)
	public static int moveTime(double angularv, double dist, double r){

	}
}
