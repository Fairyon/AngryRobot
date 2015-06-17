import lejos.nxt.*;

public class Main {
  
  protected final static int usLimit = 160; // limit for us in cm
  
	protected final static int width = 114; // every cm
	protected final static int length = 240; // every cm
	protected final static int candiam = 7; // size in cm
	protected final static int grabberlen = 13; // size in cm
	protected final static int distToEyes = 14; // distance in cm
	protected final static int angleToEyes = 20; // angle in degree

	protected final static MotorPort leftMotorPort = MotorPort.C;
	protected final static MotorPort rightMotorPort = MotorPort.B;
	protected final static MotorPort grabMotorPort = MotorPort.A;

	protected final static SensorPort lightSensorPort = SensorPort.S3;
	protected final static SensorPort canTouchSensorPort = SensorPort.S4;
	protected final static SensorPort usSensorPort = SensorPort.S1;

	protected final static int grabMotorSpeed = 250;
	protected final static int rotationSpeed = 200;

	public static void main(String[] args) {
		// Button.waitForAnyPress();

		Controller c = new Controller();
		c.start();

		Button.waitForAnyPress();
		c.stop();
		System.exit(0);
	}

}
