import lejos.nxt.SensorPort;
import lejos.nxt.SensorPortListener;

public class UsMapListener implements SensorPortListener {

	private static Point refVector = new Point(1, 0);
	private static Map map;

	private ColaBot robot;

	public UsMapListener(ColaBot robot, Map map) {
		if (robot == null)
			throw new NullPointerException("robot null");
		if (map == null)
			throw new NullPointerException("map null");

		this.robot = robot;
	}

	@Override
	public void stateChanged(SensorPort source, int oldValue, int newValue) {
		Point pos = robot.getUsPosition();
		float angle = robot.getUSAngle();
		int distance = robot.getUsDistance();
		// System.out.print(distance+" ");

		Point direction = Point.getDirectionVector(1, angle);
		Point destination = pos.clone();

		int i = distance;

		if (distance > Main.usLimit)
			distance = Main.usLimit;

		try {
			while (i-- > 0) {
				destination.moveBy(direction);
				map.decrement(destination);
			}
			destination.moveBy(direction);
			map.increment(destination);
		} catch (ArrayIndexOutOfBoundsException e) {
			// TODO Rekalibrierung
		}
	}
}
