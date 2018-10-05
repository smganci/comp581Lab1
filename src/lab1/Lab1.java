package lab1;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;

public class Lab1 {

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
		    
	    
		while() {
		  mB.forward();
		  mC.forward();
		  	  
		}
		
		Sound.playTone();
	}
}
