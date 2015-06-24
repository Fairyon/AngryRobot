public class Controller extends Thread {
	private static final int distanceDelta = 10;

	private ColaBot robot;

	// private UsMapListener usListener;,

	protected Controller() {
		this.robot = new ColaBot();
		// this.usListener = new UsMapListener(robot, map);
	}

	public void run() {
		robot.init();
		robot.findCan(90);
		//robot.liftCan();
		//robot.stop();
		
		/*
		robot.travel(10 * 10, true);
		robot.raisingCrane();
		if (robot.isTouchSensorPressed()) {
			Sound.beep();
		}
		Button.waitForAnyPress();

		robot.rotate(180);
		Sound.beep();
		Button.waitForAnyPress();

		robot.loweringCrane();
		Sound.beep();
		Button.waitForAnyPress();
		*/

		// robot.addUsSensorPortListener(usListener);

		/*
		 * while (!isInterrupted()) { if (findCan()) { Sound.beepSequenceUp();
		 * Button.waitForAnyPress(); } }
		 */

		/*
		 * try { // robot.rangecalibration();
		 * 
		 * robot.usSensor.addSensorPortListener(new UsMapListener(robot, map));
		 * 
		 * while (!isInterrupted()) { // TODO Zur Wand justieren
		 * 
		 * // TODO Sensor ausrichten
		 * 
		 * while (robot.usSensor.getDistance() >= Main.width) { // TODO Sensor
		 * pruefen // TODO Fahren }
		 * 
		 * // TODO Roboter zum Ziel rotieren
		 * 
		 * // TODO Losfahren
		 * 
		 * // TODO Dose schnappen //robot.getCan();
		 * 
		 * // TODO Zurueck fahren
		 * 
		 * } } finally { robot.stop(); }
		 */
	}

	public void stop() {
		this.interrupt();
		try {
			this.join();
		} catch (InterruptedException e) {
			System.out.println("'Stop' interrupted");
			e.printStackTrace();
		}
	}
}
