

import lejos.nxt.*;

/**
 * Erweiter die Klasse <code>UltrasonicSensor</code> um das Listener-Konzept.
 */
public class UltrasonicSensorExtended extends UltrasonicSensor {
	private SensorPort port;
	private short numListeners;
	private SensorPortListener[] listeners;
	private int previousValue;
	private int tolerance;

	/**
	 * Erstellt ein neues <code>UltrasonicSensor</code>-Objekt mit dem
	 * uebergebenen Port
	 * 
	 * @param port
	 *            Port, an den der Ultraschallsensor angeschlossen wurde.
	 */
	public UltrasonicSensorExtended(SensorPort port) {
		super(port);

		this.port = port;
		this.numListeners = 0;
		this.tolerance = 1;

		this.continuous();
		this.previousValue = getDistance();
	}

	/**
	 * Fuegt einen <code>SensorPortListener</code> hinzu.<br>
	 * Hinweis: Es koennen hoechstens 8 Listener hinzugefuegt werden.
	 * 
	 * @param listener
	 *            Listener for CallBacks
	 */
	public synchronized void addSensorPortListener(SensorPortListener listener) {
		if (listeners == null) {
			listeners = new SensorPortListener[8];

			UltrasonicListenerThread thread = UltrasonicListenerThread
					.getSingleton();
			thread.addCallback(this);
		}
		listeners[numListeners++] = listener;
	}

	/**
	 * Setzt die Toleranz, die ueberschritten werden muss, damit der Distanzwert
	 * als geaendet wahrgenommen wird.
	 * 
	 * @param tolerance
	 *            Toleranzwert
	 */
	public synchronized void setListenerTolerance(int tolerance) {
		if (tolerance <= 0)
			throw new IllegalArgumentException("Invalid tolerance");

		this.tolerance = tolerance;
	}

	/**
	 * Gibt die Toleranz, die ueberschritten werden muss, damit der Distanzwert
	 * als geaendet wahrgenommen wird, zurueck.
	 * 
	 * @return Toleranzwert
	 */
	public int getListenerTolerance() {
		return tolerance;
	}

	/**
	 * Gibt den Wert zurueck, den das vorherige Event ausgeloest hat.
	 * 
	 * @return Vorheriger Wert
	 */
	public int getPreviousListenerValue() {
		return previousValue;
	}

	/**
	 * Ruft die PortListeners auf, die mit dem Sensor verknuepft wurden.
	 */
	public synchronized void callListeners() {
		int newValue = getDistance();

		if (Math.abs(newValue - previousValue) >= tolerance) {
			for (int i = 0; i < numListeners; i++)
				listeners[i].stateChanged(port, previousValue, newValue);
			previousValue = newValue;
		}
	}
}
