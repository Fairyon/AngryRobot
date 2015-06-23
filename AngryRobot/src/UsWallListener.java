import lejos.nxt.SensorPort;
import lejos.nxt.SensorPortListener;
import lejos.nxt.Sound;

public class UsWallListener implements SensorPortListener {
	private static final int wallTolerance = 10;

	private static final int minWallDiff = wallTolerance + Main.grabberlen;
	private static final int minX = minWallDiff;
	private static final int maxX = Main.mapWidth - minWallDiff;
	private static final int minY = minWallDiff;
	private static final int maxY = Main.mapHeight - minWallDiff;

	private ColaBot robot;

	public UsWallListener(ColaBot robot) {
		if (robot == null)
			throw new NullPointerException("robot null");

		this.robot = robot;
	}

	@Override
	public void stateChanged(SensorPort source, int oldValue, int newValue) {
		float usAngle = robot.getAngle();

		if (usAngle == 0) {
			float botAngle = robot.getAngle();

			Point botPosition = robot.getPosition();
			Point target = botPosition.pointAt(minWallDiff, botAngle);
			float tarX = target.getX();
			float tarY = target.getY();
			if (tarX < minX || tarX > maxX || tarY < minY || tarY > maxY) {
				// target in wall radius
				Sound.beep();
			}
		}
	}
}
