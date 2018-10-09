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

public class Charlie {
	//components
	private EV3LargeRegulatedMotor motorL;
	private EV3LargeRegulatedMotor motorR;
	private EV3TouchSensor touchSensorL;
	private EV3TouchSensor touchSensorR;
	private EV3UltrasonicSensor sonicSensor;
	private double radiusL;
	private double radiusR;

	//sensor modes
	private SensorMode touchL;
	private SensorMode touchR;
	private SensorMode sonic;
	
	public Charlie(EV3LargeRegulatedMotor motorL, EV3LargeRegulatedMotor motorR, EV3TouchSensor touchSensorL, EV3TouchSensor touchSensorR, EV3UltrasonicSensor sonicSensor,double radiusL, double radiusR) {
		this.motorL=motorL;
		this.motorR=motorR;
		this.touchSensorL=touchSensorL;
		this.touchSensorR=touchSensorR;
		this.sonicSensor=sonicSensor;
		
		this.touchL=this.touchSensorL.getTouchMode();
		this.touchR = this.touchSensorR.getTouchMode();
		this.radiusL=radiusL;
		this.radiusR=radiusR;
	}
	
	/*Name: setLeftSpeed
	 * in: angular velocity in degrees (float s) 
	 * out: nothing 
	 * description: sets left motor's speed to that angular velocity 
	 * */
	public void setLeftSpeed(float s) {
		this.motorL.setSpeed(s);
	}
	
	
	/*Name: setRighttSpeed
	 * in: angular velocity in degrees (float s) 
	 * out: nothing 
	 * description: sets right motor's speed to that angular velocity 
	 * */
	public void setRightSpeed(float s) {
		this.motorR.setSpeed(s);
	}
	
	/*Name: setBothSpeed
	 * in: angular velocity in degrees (float s) 
	 * out: nothing 
	 * description: sets both motors speed to s
	 * */
	public void setBothSpeed(float s) {
		this.setLeftSpeed(s);
		this.setRightSpeed(s);
	}
	
	/*Name: moveForwardBoth
	 * in: nothing 
	 * out: nothing 
	 * description: moves both motors forward
	 * */
	public void moveForwardBoth() {
		this.motorL.forward();
		this.motorR.forward();
	}
	
	/*Name: moveBackwardBoth
	 * in: nothing 
	 * out: nothing 
	 * description: moves both motors backward
	 * */
	public void moveBackwardBoth() {
		this.motorL.backward();
		this.motorR.backward();
	}
	
	/*Name: stopBothInstant
	 * in: nothing 
	 * out: nothing 
	 * description: stops both instantly
	 * */
	public void stopBothInstant() {
		this.motorL.stop(true);
		this.motorR.stop(true);
	}
	
	/*Name: moveTillSense
	 * in: distance in meters (dist) 
	 * out: nothing 
	 * description: moves robot forward till sonic sensor senses an obstacle a certain distance away
	 * */
	public void moveTillSense(double d) {
		float[] sample_sonic = new float[this.sonic.sampleSize()];
		this.sonic.fetchSample(sample_sonic, 0);
		while(sample_sonic[0] > d) {
			this.moveForwardBoth();
			sonic.fetchSample(sample_sonic, 0);
		}
		this.stopBothInstant();
	}
	
	
	/*Name: moveTillTouch
	 * in: nothing 
	 * out: nothing 
	 * description: moves robot forward till touch sensor encounters an obstacle
	 **/
	public void moveTillTouch() {
		float[] sample_touchL = new float[touchL.sampleSize()];
		float[] sample_touchR = new float[touchR.sampleSize()];

	    while(sample_touchL[0] == 0 && sample_touchR[0]==0) {
		      this.moveForwardBoth();
		      touchL.fetchSample(sample_touchL,0);
		      touchR.fetchSample(sample_touchR,0);
		}
		this.stopBothInstant();		
	}
	

	/*Name: syncMotors
	 * in: nothing
	 * out: nothing 
	 * description: synchronizes left and right motors
	 * */
	public void syncMotors() {
		this.motorL.synchronizeWith(new EV3LargeRegulatedMotor[] {this.motorR });
	}
	
	/*Name: moveForwardTime
	 * in: time to move in seconds
	 * out: nothing 
	 * description: makes robot move forward for a certain amount of time
	 * */
	public void moveForwardTime(long sec) {
		this.syncMotors();
		this.moveForwardBoth();
		Delay.msDelay(sec);
		this.stopBothInstant();
	}
	
	/*Name: moveBackwardTime
	 * in: time to move in seconds
	 * out: nothing 
	 * description: makes robot move Backward for a certain amount of time
	 * */
	public void moveBackwardTime(long sec) {
		this.syncMotors();
		this.moveBackwardBoth();
		Delay.msDelay(sec);
		this.stopBothInstant();
	}
	///////////////////////////////////need to alter move time to account for diff radius
	/*Name: moveTime
	 * in: time to move in seconds
	 * out: nothing 
	 * description: makes robot move Backward for a certain amount of time
	 * */	
	private long moveTime(float angularv, double d) {
		return (long) (d / (angularv * (Math.PI / 180) *(this.radiusL+this.radiusR)/2.0) * 1000);
	}
	
	/*Name: moveForwardDist
	 * in: distance in meters 
	 * out: nothing 
	 * description: moves robot forward a certain distance
	 **/
	public void moveForwardDist(double d) {
		//w=(ul+ur)/2
		float w= (this.motorL.getSpeed()+this.motorR.getSpeed())/2;
		long sec =this.moveTime(w, d);
		this.moveForwardTime(sec);
	}

	/*Name: moveBackwardDist
	 * in: distance in meters 
	 * out: nothing 
	 * description: moves robot backward a certain distance
	 **/
	public void moveBackwardDist(double d) {
		//w=(ul+ur)/2
		float w= (this.motorL.getSpeed()+this.motorR.getSpeed())/2;
		long sec =this.moveTime(w, d);
		this.moveBackwardTime(sec);
	}

	/*Name: beep
	 * in: nothing
	 * out: nothing 
	 * description: makes robot beep
	 * */
	public void beep() {
		Sound.beep();
	}
	
	/*Name: buttonWait
	 * in: nothing
	 * out: nothing 
	 * description: waits for button press to continue
	 * */
	public void buttonWait() {
		Button.ENTER.waitForPressAndRelease();
	}
}
