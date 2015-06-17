

import java.util.ArrayList;

/**
 * Hilfsklasse zur Weiterleitung von Events an SensorPortListeners
 */
public class UltrasonicListenerThread extends Thread {
	private static final int pause = 50;
	private static UltrasonicListenerThread singleton = null;

	/**
	 * Gibt das einzige Objekt der Klasse zurueck
	 * 
	 * @return Das einzige Thread-Objekt
	 */
	public static synchronized UltrasonicListenerThread getSingleton() {
		if (singleton == null) {
			singleton = new UltrasonicListenerThread();
			singleton.setDaemon(true);
			singleton.setPriority(Thread.MAX_PRIORITY);
		}
		return singleton;
	}

	private ArrayList<ColaUltrasonicSensor> callers;

	private UltrasonicListenerThread() {
		callers = new ArrayList<ColaUltrasonicSensor>();
	}

	/**
	 * Fuegt einen Listener hinzu.
	 * 
	 * @param caller
	 *            Das Callback-Objekt, das aufgerufen wird, wenn sich der Wert
	 *            des Sensors aendert.
	 */
	public synchronized void addCallback(ColaUltrasonicSensor caller) {
		callers.add(caller);

		// Thread starten, wenn das erste CallBack-Objekt registriert wurde
		if (callers.size() == 1)
			singleton.start();
	}

	@Override
	public void run() {
		ColaUltrasonicSensor caller;

		while (!isInterrupted()) {
			try {
				Thread.sleep(pause);
			} catch (InterruptedException e) {
			}

			for (int i = 0; i < callers.size(); i++) {
				caller = callers.get(i);
				caller.callListeners();
			}
		}
	}
}