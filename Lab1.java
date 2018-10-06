package lab1;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;

public class Lab1 {

	public static void main(String[] args) {
		
		System.out.println("Lab1 Start");
		System.out.println("Calibrating Sensors");
		EV3LargeRegulatedMotor mB = new EV3LargeRegulatedMotor(MotorPort.B);
		EV3LargeRegulatedMotor mC = new EV3LargeRegulatedMotor(MotorPort.C);
		EV3TouchSensor touchsensor1 = new EV3TouchSensor(SensorPort.S1);
		EV3UltrasonicSensor ultrasensor = new EV3UltrasonicSensor(SensorPort.S2);
		EV3TouchSensor touchsensor4 = new EV3TouchSensor(SensorPort.S4);

		SensorMode touch1 = touchsensor1.getTouchMode();
		SensorMode touch4 = touchsensor4.getTouchMode();
		SensorMode sonic = (SensorMode) ultrasensor.getDistanceMode();
		
		// variables
		// radius is approximately .028 cm
		double r = .028;

		// Move forward 1.2m
		System.out.println("Objective 1");
		System.out.println("Press Button to Start");
		Button.ENTER.waitForPressAndRelease();
		
		mB.setSpeed(360);
		mC.setSpeed(360);
		long time1 = moveTime(360, 1.2, r);
		mC.synchronizeWith(new EV3LargeRegulatedMotor[] { mB });
		mB.forward();
		mC.forward();
		Delay.msDelay(time1);
		mB.stop(true);
		mC.stop(true);
		Sound.beep();
		
		System.out.println("Objective 2");
		System.out.println("Press Button to Continue");
		Button.ENTER.waitForPressAndRelease();
		
		float[] sample_sonic = new float[sonic.sampleSize()];
		
		sonic.fetchSample(sample_sonic, 0);
		while(sample_sonic[0] > .5) {
			mB.forward();
			mC.forward();
			sonic.fetchSample(sample_sonic, 0);
			System.out.println("Distance from object: " + sample_sonic[0]);
		}
		mB.stop(true);
		mC.stop(true);

		// Move forward unknown distance
		// Sense object .5m away
		// Stop
		Sound.beep();
		
		System.out.println("Objective 3");
		System.out.println("Press Button to Continue");
		Button.ENTER.waitForPressAndRelease();
		
		float[] sample_touch1 = new float[touch1.sampleSize()];
		float[] sample_touch4 = new float[touch4.sampleSize()];

	    while(sample_touch1[0] == 0 && sample_touch4[0]==0) {
		      mC.forward();
		      mB.forward();
		      touch1.fetchSample(sample_touch1,0);
		      touch4.fetchSample(sample_touch4,0);
		}
		mB.stop(true);
		mC.stop(true);
		
		long time2=moveTime(360,.5,r);
		mB.backward();
		mC.backward();
		Delay.msDelay(time2);
		mB.stop(true);
		mC.stop(true);
		
		System.out.println("End Lab1");

		// Move forward until touch
		// Reverse directions for .5m
	}

	// move time take in
	// angularv (angular velocity in radians per second)
	// dist (desired distance in meters)
	// r (radius of wheel)
	// returns time in milliseconds
	public static long moveTime(int angularv, double dist, double r) {

		return (long) (dist / (angularv * (Math.PI / 180) * r) * 1000);

	}

}
