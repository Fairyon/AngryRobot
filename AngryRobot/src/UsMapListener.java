import java.awt.*;
import lejos.nxt.*;

public class UsMapListener implements SensorPortListener {
	private static Point refVector = new Point(1, 0);

	private CokeBot robot;
	private Map map;

	public UsMapListener(CokeBot robot, Map map) {
		if (robot == null)
			throw new NullPointerException("robot null");
		if (map == null)
			throw new NullPointerException("map null");

		this.robot = robot;
		this.map = map;
	}

	@Override
	public void stateChanged(SensorPort source, int oldValue, int newValue) {
		Point pos = robot.getUsPosition();
		double angle = robot.getAngle();
		int dist = robot.usSensor.getDistance();

		//System.out.println("pos: (" + pos.x + "," + pos.y + ")");
		//System.out.println("angle: " + angle);
		//System.out.println("dist: " + dist);

		//Button.waitForAnyPress(3000);

		double rad = Math.toRadians(angle);
		double cos = Math.cos(rad);
		double sin = Math.sin(rad);

		double rotatedX = refVector.x * cos - refVector.y * sin;
		double rotatedY = refVector.x * sin + refVector.y * cos;

		double stretchX, stretchY;
		int x, y;
		for (int i = 0; i <= dist; i++) {
			// Button.waitForAnyPress(125);

			stretchX = rotatedX * i;
			stretchY = rotatedY * i;

			x = (int) (stretchX + pos.x);
			y = (int) (stretchY + pos.y);

			if (x < 0 || x >= map.getMaxX() || y < 0 || y >= map.getMaxY())
				break;

			if (i == dist && dist != 255) {
				map.increment(x, y);
				//System.out.println("inc (" + x + "," + y + "): " + dist);
			} else {
				map.decrement(x, y);
				//System.out.println("dec (" + x + "," + y + "): " + dist);
			}
		}
	}
}
