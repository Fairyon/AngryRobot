import lejos.nxt.SensorPort;
import lejos.nxt.SensorPortListener;

public class UsWallListener implements SensorPortListener {

	private static Point refVector = new Point(1, 0);

	private ColaBot robot;

	public UsWallListener(ColaBot robot) {
		if (robot == null)
			throw new NullPointerException("robot null");

		this.robot = robot;
	}

	@Override
	public void stateChanged(SensorPort source, int oldValue, int newValue) {

	}
}
