import lejos.nxt.*;
import lejos.nxt.comm.RConsole;

public class Main {

	protected final static int minimalDelta = 6;

	protected final static float canRotFactor = 1.98f;
	protected final static int usLimit = 160; // limit for us in cm

	protected final static int mapWidth = 114; // every mm
	protected final static int mapLength = 240; // every mm
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
		/*
		 * RConsole.openUSB(0); if (RConsole.isOpen()) {
		 * System.setOut(RConsole.getPrintStream());
		 * System.setErr(RConsole.getPrintStream()); }
		 */

		// try {
		Button.waitForAnyPress();

		Controller c = new Controller();
		c.start();

		Button.waitForAnyPress();
		c.stop();
		/*
		 * } finally { if (RConsole.isOpen()) { RConsole.close(); } }
		 */

		System.exit(0);
	}

}
