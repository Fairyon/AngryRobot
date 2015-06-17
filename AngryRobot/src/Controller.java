public class Controller extends Thread {
	private ColaBot robot;
	private Map map;

	protected Controller() {
		this.robot = new ColaBot();
		this.map = new Map(Main.length, Main.width);
	}

	public void run() {
	  robot.init();
	  robot.usSensor.addSensorPortListener(new UsMapListener(robot, map));
	  robot.test();
	  
	  /*
		try {
			// robot.rangecalibration();

			robot.usSensor.addSensorPortListener(new UsMapListener(robot, map));

			while (!isInterrupted()) {
				// TODO Zur Wand justieren

				// TODO Sensor ausrichten

				while (robot.usSensor.getDistance() >= Main.width) {
					// TODO Sensor pruefen
					// TODO Fahren
				}
				
				// TODO Roboter zum Ziel rotieren

				// TODO Losfahren

				// TODO Dose schnappen
				//robot.getCan();

				// TODO Zurueck fahren

			}
		} finally {
			robot.stop();
		}
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
