package lab1;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;
/*Collaborators:
 * Tricia Bacon (730011125)
 * Sarah Ganci (720510446)
 * 
 * */
public class Lab1 {

	public static void main(String[] args) {
		
		/*//////////////////Objective 0: Set Up//////////////////
		 * create instances of motors and sensors
		 * create necessary variables
		 * */
		
		//0.1: print startup message
		System.out.println("Lab1 Start");
		System.out.println("Calibrating Sensors");
		
		//0.2: instantiate sensors and motors
		EV3LargeRegulatedMotor mB = new EV3LargeRegulatedMotor(MotorPort.B);
		EV3LargeRegulatedMotor mC = new EV3LargeRegulatedMotor(MotorPort.C);
		EV3TouchSensor touchsensor1 = new EV3TouchSensor(SensorPort.S1);
		EV3UltrasonicSensor ultrasensor = new EV3UltrasonicSensor(SensorPort.S2);
		EV3TouchSensor touchsensor4 = new EV3TouchSensor(SensorPort.S4);

		SensorMode touch1 = touchsensor1.getTouchMode();
		SensorMode touch4 = touchsensor4.getTouchMode();
		SensorMode sonic = (SensorMode) ultrasensor.getDistanceMode();

		//0.3: set radius variable (approximately .028 m)
		double r = .028;

		
		/*//////////////////Objective 1://////////////////
		 * move forward 1.2 meters
		 * stop
		 * beep
		 * */
		
		//1.1: print message and wait for button press
		System.out.println("Objective 1");
		System.out.println("Press Button to Start");
		Button.ENTER.waitForPressAndRelease();
		
		//1.2: set motor speed to 180 deg
		mB.setSpeed(180);
		mC.setSpeed(180);
		
		//1.3: calculate necesarry time to travel 1.2 meters (1.23 adjusted value after trials)
		long time1 = moveTime(180, 1.23, r);
		
		//1.4: synchronize motors
		mC.synchronizeWith(new EV3LargeRegulatedMotor[] { mB });
		
		//1.5: move both motors forward
		mB.forward();
		mC.forward();
		
		//1.6: delay by travel time
		Delay.msDelay(time1);
		
		//1.7: stop both motors at same time
		mB.stop(true);
		mC.stop(true);
		
		//1.8: beep
		Sound.beep();
		
		
		/*//////////////////Objective 2://////////////////
		 * after press of button
		 * move forward till 0.5 meters away from sensed obstacle
		 * stop
		 * beep
		 * */	
		//2.1: print message and wait for button press
		System.out.println("Objective 2");
		System.out.println("Press Button to Continue");
		Button.ENTER.waitForPressAndRelease();
		
		//2.2: create float array to store sample's from sonic sensor
		float[] sample_sonic = new float[sonic.sampleSize()];
		sonic.fetchSample(sample_sonic, 0);
		
		//2.3: move forward and fetch sonic samples till sonic sensor value is less than or equal to .5 (.54 adjusted after trials)
		while(sample_sonic[0] > .54) {
			mB.forward();
			mC.forward();
			sonic.fetchSample(sample_sonic, 0);
		}
		
		//2.4: stop both motors at same time
		mB.stop(true);
		mC.stop(true);
		
		//2.5: beep
		Sound.beep();
		
		
		/*//////////////////Objective 3: //////////////////
		 * wait for button press
		 * move forward till contact with obstacle
		 * stop
		 * move backwards till 0.5 meters away from obstacle
		 * */		
		//3.1: print message and wait for button press
		System.out.println("Objective 3");
		System.out.println("Press Button to Continue");
		Button.ENTER.waitForPressAndRelease();
		
		//3.2: sample_touch 1 and 4 will hold the result of fetching touch samples from both touch sensors
		float[] sample_touch1 = new float[touch1.sampleSize()];
		float[] sample_touch4 = new float[touch4.sampleSize()];

		//3.3: move forward and fetch another sample of touch while both touch sensor arrays have no values (ie no contact)
		while(sample_touch1[0] == 0 && sample_touch4[0]==0) {
		      mC.forward();
		      mB.forward();
		      touch1.fetchSample(sample_touch1,0);
		      touch4.fetchSample(sample_touch4,0);
		}
	    
	    //3.4: after loop ends (touch sensors have made contact) stop both motors at same time
		mB.stop(true);
		mC.stop(true);
		
		//3.5: calculate time needed to move backwards .5 meters (.52 adjusted after trials)
		long time2=moveTime(180,.52,r);
		
		//3.6: move backward
		mB.backward();
		mC.backward();
		
		//3.7: delay by travel time
		Delay.msDelay(time2);
		
		//3.8: stop both motors at same time
		mB.stop(true);
		mC.stop(true);
		
		//3.9: print end lab message
		System.out.println("End Lab1");


	}

	/*Name: moveTime
	 * in: 
	 * 	angularv (in degrees)
	 * 	dist (in meters)
	 * 	radius (in meters)
	 * out:
	 * 	time in seconds required to move to achieve some distance
	 */
	public static long moveTime(int angularv, double dist, double r) {
		return (long) (dist / (angularv * (Math.PI / 180) * r) * 1000);
	}

}