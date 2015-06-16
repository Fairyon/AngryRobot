import lejos.nxt.Button;

public class Controller extends Thread {

	private CokeBot robot;
	private Map map;

	protected Controller() {
		this.robot = new CokeBot();
		this.map = new Map(Main.length, Main.width);
	}

	public void run() {
		robot.init();
		try {
			//robot.rangecalibration();
		  robot.init();
		  
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
				robot.getCan();

				// TODO Zurueck fahren

			}
		} finally {
			robot.stop();
		}
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
