import java.util.ArrayList;

import javax.microedition.lcdui.Graphics;

import lejos.nxt.*;
import lejos.robotics.*;
import lejos.robotics.navigation.*;

public class ColaBot {
	private static final int wallTolerance = 5;

	private static final int minWallDiff = wallTolerance + Main.grabberlen;
	private static final int minX = minWallDiff;
	private static final int maxX = Main.mapWidth - minWallDiff;
	private static final int minY = minWallDiff;
	private static final int maxY = Main.mapHeight - minWallDiff;

	private static final float distanceFactor = 0.09639f;
	private static final float rotationFactor = 0.995f;

	private static final Point homePosition = new Point(20, 20);

	private final ColaDifferentialPilot pilot;
	private final RegulatedMotor leftMotor;
	private final RegulatedMotor rightMotor;
	private final GrabMotor craneMotor;
	private final LightSensor lightSensor;
	private final TouchSensor canTouchSensor;
	private final ColaUltrasonicSensor usSensor;

	private Point curpos;
	private float curangle;

	private boolean isMoving;

	public ColaBot() {
		this.leftMotor = new NXTRegulatedMotor(Main.leftMotorPort);
		this.rightMotor = new NXTRegulatedMotor(Main.rightMotorPort);
		this.craneMotor = new GrabMotor(Main.grabMotorPort);
		this.lightSensor = new LightSensor(Main.lightSensorPort);
		this.canTouchSensor = new TouchSensor(Main.canTouchSensorPort);
		this.usSensor = new ColaUltrasonicSensor(Main.usSensorPort);
		this.pilot = new ColaDifferentialPilot(56, 138 - 26, leftMotor,
				rightMotor);

		curpos = new Point(40, 40);
		curangle = 0;
	}

	public void init() {
		usSensor.continuous();
		lightSensor.setFloodlight(true);
		pilot.addMoveListener(new PositionKeeper());

		pilot.setRotateSpeed(75);
		pilot.setTravelSpeed(150);
	}

	public void stop() {
		pilot.stop();
		loweringCrane();
		lightSensor.setFloodlight(false);
		usSensor.off();
	}

	public void addUsSensorPortListener(SensorPortListener listener) {
		if (listener == null)
			throw new NullPointerException("listener is null");

		usSensor.addSensorPortListener(listener);
	}

	public void loweringCrane() {
		craneMotor.lookAhead();
	}

	public void raisingCrane() {
		craneMotor.setAcceleration(12000);
		craneMotor.setSpeed(300);
		craneMotor.rotateTo(-45);
		craneMotor.setSpeed(50);
		craneMotor.setAcceleration(6000);
	}

	public void usRotate(float angle, boolean immediateReturn) {
		pilot.usRotate(angle, immediateReturn);
	}

	public void usRotate(float angle) {
		craneMotor.rotate(-5);
		
		usRotate(angle, false);
		
		loweringCrane();
	}

	public void usRotateTo(float angle) {
		craneMotor.rotate(-5);
		
		usRotateTo(angle, false);
		
		loweringCrane();
	}

	public void usRotateTo(float angle, boolean immediateReturn) {
		float relAngle = angle - getAngle();
		
		System.out.println("abs:" + angle + " rel:" + relAngle);
		
		pilot.usRotate(relAngle, immediateReturn);
	}

	public int getUsDistance() {
		return usSensor.getDistance();
	}

	public int getDistToObject(int usRange, float usAngle) {
		Point usPos = getUsPosition();
		Point r1 = curpos.getDirectionTo(usPos);
		Point r2 = Point.getDirectionVector(usRange, usAngle);
		return (int) r1.addTo(r2).getLength();
	}

	private boolean isValidPoint(Point p) {
		float tarX = p.getX();
		float tarY = p.getY();

		return tarX >= minX && tarX <= maxX && tarY >= minY && tarY <= maxY;
	}

	/**
	 * Lets the robot travel a specified range
	 * 
	 * @param distance
	 *            Range to travel
	 * @param immediateReturn
	 *            If true this method returns immediately
	 */
	public boolean travel(double distance, boolean immediateReturn) {
		Point target;
		Point position = getPosition();
		float angle = getAngle();
		boolean changed = false;

		// TODO Infinite loop if location is already too close to the wall
		double tmpDistance = distance / 10;
		do {
			target = position.pointAt((float) tmpDistance, angle);
			if (isValidPoint(target)) {
				distance = 10 * tmpDistance;
				break;
			} else {
				changed = true;
				tmpDistance--;
			}
			System.out.println(target);
		} while (true);

		pilot.travel(distance, immediateReturn);

		return changed;
	}

	/**
	 * Lets the robot travel a specified range
	 * 
	 * @param distance
	 *            Range to travel
	 */
	public boolean travel(double distance) {
		return travel(distance, false);
	}

	/**
	 * Check if the sensor is pressed
	 * 
	 * @return true if sensor is pressed, false otherwise
	 */
	public boolean isTouchSensorPressed() {
		return canTouchSensor.isPressed();
	}

	public boolean[] calibrate(int corner) {
		if (pilot.isMoving() || pilot.isStalled())
			return null;
		int obstacleGeer = 0;
		boolean hasObstacles = false;
		int calibrated = 0;
		int min1 = 255;
		int min2 = 255;
		int curdist, lastdist, distdiff;
		float angle;
		float minangle1 = 360;
		float minangle2 = 360;
		float edgeangle = 45 + corner * 90;
		float relangle = (curangle + 360) % 360 - edgeangle;
		System.out.println(relangle);
		float oStartAngle = 0; // obstacle start angle
		float oEndAngle = 180;
		int direction; // -1 = left, +1 = right

		if (relangle > 180)
			relangle -= 360;
		else if (relangle < -180)
			relangle += 360;

		direction = (int) Math.signum(-relangle);

		pilot.rotate(-direction * 90 - relangle); // 90 to the l(-1)/r of the
		// edge
		pilot.rotate(direction * 90, true); // l(-1)/r wall
		lastdist = usSensor.getDistance();
		while (pilot.isMoving()) {
			curdist = usSensor.getDistance();
			angle = getAngle();
			distdiff = curdist - lastdist;
			if (distdiff <= -Main.minimalDelta) { // came across an obstacle
				obstacleGeer++;
				hasObstacles = true;
				if (obstacleGeer == 1)
					oStartAngle = angle;
			} else if (distdiff >= Main.minimalDelta) { // came off an obstacle
				obstacleGeer--;
				if (obstacleGeer == 0)
					oEndAngle = angle;
			}
			if (obstacleGeer > 0)
				continue;
			if (curdist < min1) {
				min1 = curdist;
				minangle1 = angle;
			}
			lastdist = curdist;
		}
		if (!(-direction * oStartAngle > -direction * minangle1 && -direction
				* minangle1 > -direction * oEndAngle)
				&& min1 >= 25 && min1 < 255) {
			calibrated++;
		}
		/* other wall */
		oStartAngle = 180;
		oEndAngle = 0;
		pilot.rotate(direction * 90, true); // r(-1)/l wall
		while (pilot.isMoving()) {
			curdist = usSensor.getDistance();
			angle = getAngle();
			distdiff = curdist - lastdist;
			if (distdiff <= -Main.minimalDelta) { // came across an obstacle
				obstacleGeer++;
				hasObstacles = true;
				if (obstacleGeer == 1)
					oStartAngle = angle;
			} else if (distdiff >= Main.minimalDelta) { // came off an obstacle
				obstacleGeer--;
				if (obstacleGeer == 0)
					oEndAngle = angle;
			}
			if (obstacleGeer > 0)
				continue;
			if (curdist < min2) {
				min2 = curdist;
				minangle2 = angle;
			}
			lastdist = curdist;
		}
		if (!(direction * oStartAngle < direction * minangle2 && direction
				* minangle2 < direction * oEndAngle)
				&& min2 >= 25 && min1 < 255) {
			calibrated += 2;
		}
		pilot.rotate(-direction * 90 + relangle); // rotate back
		switch (calibrated) {
		case 0:
			System.out.println("Keine Messungen!");
			return new boolean[] { false, hasObstacles };
		case 1:
			calcCalibration(corner, min1, minangle1, direction > 0);
			break;
		case 2:
			calcCalibration(corner, min2, minangle2, direction < 0);
			break;
		case 3:
			calcCalibration(corner, min1, minangle1, direction > 0);
			calcCalibration(corner, min2, minangle2, direction < 0);
			break;
		default:
			System.out.println("o_O Calib Error!");
		}
		System.out.println("Dist: " + min1 + ", " + min2);
		System.out.println("Pos: " + curpos);
		System.out.println("Angle: " + curangle);
		System.out.println(hasObstacles);
		System.out.println(obstacleGeer);
		return new boolean[] { true, hasObstacles };
	}

	private void calcCalibration(int corner, int dist, float angle, boolean side) {
		switch (corner) {
		case 0: // top right
			if (side) { // right
				curpos.setX(Main.mapWidth - dist - Main.grabberlen);
				curangle = -angle;
			} else {
				curpos.setY(Main.mapHeight - dist - Main.grabberlen);
				curangle = 90 - angle;
			}
			break;
		case 1: // top left
			if (side) { // right
				curpos.setY(Main.mapHeight - dist - Main.grabberlen);
				curangle = 90 - angle;
			} else {
				curpos.setX(dist + Main.grabberlen);
				curangle = 180 - angle;
			}
			break;
		case 2: // bottom left
			if (side) { // right
				curpos.setX(dist + Main.grabberlen);
				curangle = 180 - angle;
			} else {
				curpos.setY(dist + Main.grabberlen);
				curangle = 270 - angle;
			}
			break;
		case 3: // bottom right
			if (side) { // right
				curpos.setY(dist + Main.grabberlen);
				curangle = 270 - angle;
			} else {
				curpos.setX(Main.mapWidth - dist - Main.grabberlen);
				curangle = -angle;
			}
			break;
		default:
			System.out.println("Wrong Corner");
		}
	}

	protected Point getUsPosition() {
		if (isMoving) {
			Move event = pilot.getMovement();
			if (event.getMoveType().equals(Move.MoveType.ROTATE)) {
				return curpos.pointAt(Main.distToEyes, Main.angleToEyes
						+ curangle + event.getAngleTurned());
			} else {
				return curpos.pointAt(event.getDistanceTraveled(), curangle)
						.pointAt(Main.distToEyes, Main.angleToEyes + curangle);
			}

		} else {
			return curpos.pointAt(Main.distToEyes, Main.angleToEyes + curangle);
		}
	}

	protected float getCraneTachoCount() {
		return craneMotor.getTachoCount();
	}

	protected float getAngle() {
		if (isMoving) {
			Move event = pilot.getMovement();
			if (event.getMoveType().equals(Move.MoveType.ROTATE)) {
				float angle = curangle + event.getAngleTurned();
				if (angle < -180)
					angle += 360;
				else if (angle > 180)
					angle -= 360;
				return angle;
			} else if(event.getMoveType().equals(Move.MoveType.ARC)) {
				// TODO position �ndern
				float angle = curangle + event.getAngleTurned();
				if (angle < -180)
					angle += 360;
				else if (angle > 180)
					angle -= 360;
				return angle;
			} else {
				return curangle;
			}

		} else {
			return curangle;
		}
	}

	protected Point getPosition() {
		if (isMoving) {
			Move event = pilot.getMovement();
			if (event.getMoveType().equals(Move.MoveType.ROTATE)) {
				return curpos;
			} else {
				return curpos.pointAt(event.getDistanceTraveled(), curangle);
			}
		} else {
			return curpos;
		}
	}

	/**
	 * true if the robot is moving
	 * 
	 * @return true if the robot is moving
	 */
	public boolean isMoving() {
		return pilot.isMoving();
	}

	/**
	 * Rotates the robot through a specific angle
	 * 
	 * @param angle
	 *            The wanted angle of rotation in degrees. Positive angle rotate
	 *            left (anti-clockwise), negative right.
	 * @param immediateReturn
	 *            If true this method returns immediately.
	 */
	public boolean rotate(float angle, boolean immediateReturn) {
		Point botPosition = getPosition();
		Point targetUsPos = botPosition.pointAt(Main.distToEyes,
				Main.angleToEyes + curangle + angle);

		boolean result = isValidPoint(targetUsPos);
		if (result)
			pilot.rotate(angle, immediateReturn);
		else {
			System.out.println();
			System.out.println("rotate failed");
			System.out.println("angle: " + angle);
			System.out.println("eyepos: " + targetUsPos);
			System.out.println();
		}

		return result;
	}

	/**
	 * Rotates the robot through a specific angle
	 * 
	 * @param angle
	 *            The wanted angle of rotation in degrees. Positive angle rotate
	 *            left (anti-clockwise), negative right.
	 */
	public boolean rotate(float angle) {
		return rotate(angle, false);
	}

	/**
	 * Rotates the robot to the specific angle
	 * 
	 * @param angle
	 *            The wanted angle of the robot after the rotation in degrees
	 * @param immediateReturn
	 *            If true this method returns immediately.
	 */
	public boolean rotateTo(float absoluteAngle, boolean withCan,
			boolean immediateReturn) {
		float relativeAngle = (absoluteAngle - getAngle() + 360) % 360;
		if (withCan)
			relativeAngle *= Main.canRotFactor;
		return rotate(relativeAngle, immediateReturn);
	}

	/**
	 * Rotates the robot to the specific angle
	 * 
	 * @param angle
	 *            The wanted angle of the robot after the rotation in degrees
	 */
	public boolean rotateTo(float absoluteAngle, boolean withCan) {
		return rotateTo(absoluteAngle, withCan, false);
	}

	/**
	 * Stops the robots movement
	 */
	public void stopMovement() {
		pilot.stop();
	}

	private class PositionKeeper implements MoveListener {

		public void moveStarted(Move event, MoveProvider mp) {
			isMoving = true;
		}

		@Override
		public void moveStopped(Move event, MoveProvider mp) {
				if (event.getMoveType().equals(Move.MoveType.ROTATE)) {
					curangle += event.getAngleTurned() * rotationFactor;
				} else if (event.getMoveType().equals(Move.MoveType.ARC)) {
					curangle += event.getAngleTurned() * rotationFactor;
					//TODO position andern
				} else {
					curpos.moveAt(event.getDistanceTraveled() * distanceFactor,
							curangle);
				}
			isMoving = false;
		}
	}

	private boolean canTravel(Point start, Point target, Point[] cans) {
		System.out.println();
		System.out.println("enter canTravel");
		System.out.println("start:" + start);
		System.out.println("target:" + target);
		System.out.println("cans:" + cans.length);
		System.out.println();

		if (!isValidPoint(target))
			return false;

		Point direction = new Point(target.getX() - start.getX(), target.getY()
				- start.getY());
		float length = direction.getLength();
		Point norm = direction.scalePoint(1 / length);

		for (float t = 0; t <= length; t += 1) {
			Point cur = start.addTo(norm.scalePoint(t));

			for (int i = 0; i < cans.length; i++) {
				Point canPos = cans[i];
				float dist = canPos.getDistance(cur);

				if (dist <= Main.robotWidth / 2 + Main.candiam / 2) {
					System.out.println("failed at:");
					System.out.println("cur: " + cur);
					System.out.println("canPos: " + canPos);
					System.out.println("dist: " + dist);
					System.out.println();

					System.out.println("leave canTravel");
					System.out.println("return false");
					System.out.println();

					return false;
				}
			}
		}

		System.out.println("leave canTravel");
		System.out.println("return true");
		System.out.println();

		return true;
	}

	protected boolean findCan() {
		Point botPos = getPosition();
		Point usPos = getUsPosition();
		ArrayList<Can> canList = lookForCans(0, 90, 4);
		if (canList == null)
			return false;

		System.out.println();
		Point[] canPos = new Point[canList.size()];
		Can[] cans = canList.toArray(new Can[canList.size()]);
		int bestIndex = -1;
		for (int i = 0; i < cans.length; i++) {
			System.out.println("Can " + (i+1));
			System.out.println("angle: " + cans[i].getAngle());
			System.out.println("distance: " + cans[i].getDistance());
			System.out.println("count: " + cans[i].getCount());
			
			Can curCan = cans[i];
			canPos[i] = usPos.pointAt(curCan.getDistance(), curCan.getAngle());
			if (bestIndex == -1
					|| curCan.getCount() > cans[bestIndex].getCount()) {
				bestIndex = i;
			}
			
			System.out.println("abs: " + canPos[i]);
			System.out.println();
		}

		Point canPoint = canPos[bestIndex];
		float absCanAngle = canPoint.getAngleBetween(new Point(1, 0), true);
		Point targetPoint = canPoint.pointAt(Main.candiam / 2 + Main.grabberlen
				/ 2, absCanAngle);
		if (!isValidPoint(targetPoint)) {
			System.out.println("Invalid target");
			return false;
		}

		/*
		 * if (canTravel(botPos, targetPoint, canPos)) {
		 * System.out.println("Direct way");
		 * 
		 * float angle = botPos.getAngleBetween(targetPoint, true); float
		 * distance = botPos.getDistance(targetPoint);
		 * 
		 * rotate(angle); travel(10 * distance); } else {
		 * System.out.println("No direct way");
		 * 
		 * Point t1 = new Point(targetPoint.getX(), botPos.getY()); Point t2 =
		 * new Point(botPos.getX(), targetPoint.getY());
		 * 
		 * if (canTravel(botPos, t1, canPos) && canTravel(t1, targetPoint,
		 * canPos)) { float angle = botPos.getAngleBetween(t1, true); float
		 * distance = botPos.getDistance(t1);
		 * 
		 * rotate(angle); travel(10 * distance);
		 * 
		 * angle = t1.getAngleBetween(targetPoint, true); distance =
		 * t1.getDistance(targetPoint);
		 * 
		 * if (targetPoint.getX() < t1.getX()) {
		 * System.out.println("tarX < t1X"); } else {
		 * System.out.println("tarX >= t1X"); }
		 * 
		 * rotate(angle); travel(10 * distance); } else if (canTravel(botPos,
		 * t2, canPos) && canTravel(t2, targetPoint, canPos)) { float angle =
		 * botPos.getAngleBetween(t2, true); float distance =
		 * botPos.getDistance(t2);
		 * 
		 * rotate(angle); travel(10 * distance);
		 * 
		 * if (targetPoint.getY() < t2.getY()) {
		 * System.out.println("tarY < t1Y"); } else {
		 * System.out.println("tarY >= t1Y"); }
		 * 
		 * angle = t2.getAngleBetween(targetPoint, true); distance =
		 * t2.getDistance(targetPoint);
		 * 
		 * rotate(angle); travel(10 * distance); } else {
		 * System.out.println("Uups"); System.out.println("t1:" + t1);
		 * System.out.println("t2:" + t2);
		 * 
		 * return false; } }
		 */

		return true;
	}

	protected boolean isThereCan(Point canCoord) {
		// grabMotor.rotate(getUsPosition().getDirectionTo(canCoord));
		return true;
	}

	/**
	 * 
	 * @param startAngle
	 * @param endAngle
	 * @return Polar with distance and angle to the next can relative to robot
	 */
	protected ArrayList<Can> lookForCans(int startAngle, int endAngle, int count) {
		// grabMotor.setSpeed(50);

		System.out.println();
		System.out.println("enter lookForCans");
		System.out.println("startAngle: " + startAngle);
		System.out.println("endAngle: " + endAngle);
		System.out.println("count: " + count);
		System.out.println();

		float angleBackup = getAngle();
		float angle, newAngle;
		int range, newRange;
		int delta;
		CanAdder helpThread = null;
		Can tmpCan = new Can();
		ArrayList<Can> cans = new ArrayList<Can>();

		usRotateTo(startAngle);
		angle = getAngle();

		range = usSensor.getDistance();
		boolean rotation = false; // false - from start, true - back
		while (count-- > 0) {
			usRotateTo(rotation ? startAngle : endAngle, true);
			while (pilot.isMoving()) {
				newAngle = getAngle();
				newRange = usSensor.getDistance();
				delta = range - newRange;
				if ((!rotation) && (delta > Main.minimalDelta)
						&& (newRange < 200)) { // if new angle at object
					helpThread = new CanAdder(cans, tmpCan, newRange, newAngle);
					helpThread.start();
				} else if (rotation && (delta < -Main.minimalDelta)
						&& (range < 200)) { // if old angle at object
					helpThread = new CanAdder(cans, tmpCan, range, angle);
					helpThread.start();
				}
				range = newRange;
				angle = newAngle;
			}
			rotation = !rotation;
		}
		usRotateTo(angleBackup, true); // rotate to the backup position

		try {
			if (helpThread != null)
				helpThread.join();
		} catch (InterruptedException e) {
			return null;
		}

		System.out.println("can size: " + cans.size());
		if (cans.size() > 0) {
			return cans;
		}

		return null;
	}

	/**
	 * 
	 * @param startAngle
	 * @param endAngle
	 * @return Polar with distance and angle to the next can relative to robot
	 */
	protected Polar lookForCan(int startAngle, int endAngle, int count) {
		ArrayList<Can> cans = lookForCans(startAngle, endAngle, count);
		if (cans.size() > 0) {
			int bestIndex = -1;
			int bestCount = 0;
			for (int i = 0; i < cans.size(); i++) {
				Can can = cans.get(i);

				System.out.println(can);
				if (can.getCount() > bestCount) {
					bestCount = can.getCount();
					bestIndex = i;
				}
			}

			Can bestCan = cans.get(bestIndex);
			int bestRange = (int) bestCan.getDistance();
			float bestAngle = bestCan.getAngle();
			int distToObject = getDistToObject(bestRange, bestAngle);

			return new Polar(distToObject, bestAngle + getAngle());
		}
		return null;
	}

	private class CanAdder extends Thread {
		ArrayList<Can> cans;
		Can tmpCan;
		int range;
		float angle;

		public CanAdder(ArrayList<Can> cans, Can tmpCan, int range, float angle) {
			this.cans = cans;
			this.tmpCan = tmpCan;
			this.range = range;
			this.angle = angle;
		}

		public void run() {
			synchronized (cans) {
				tmpCan.changePol(range, angle);
				boolean canAdded = false;
				for (Can can : cans) { // check if can exists
					canAdded = can.addCan(tmpCan);
					if (canAdded) {
						break;
					}
				}
				if (!canAdded) { // if not, append them to the list
					cans.add(tmpCan.clone());
				}
			}
		}
	}

	protected void test() {
		pilot.setRotateSpeed(100);
		pilot.setTravelSpeed(150);
		// travel(1000);
		rotate(90);
		// travel(-1000);
		rotate(90);
		rotate(90);
		rotate(90);
		// Button.waitForAnyPress();
		/*
		 * while(!canTouchSensor.isPressed()); System.out.println("Pressed!!!");
		 * Sound.beepSequenceUp();
		 */
		/*
		 * grabMotor.setSpeed(50); grabMotor.rotateTo(90, true); float minrange
		 * = 300, range, mindeg = 0; while (grabMotor.isMoving()) { if ((range =
		 * usSensor.getRange()) < minrange) { minrange = range; mindeg =
		 * grabMotor.getTachoCount(); } } rotate(mindeg); grabMotor.lookAhead();
		 */
	}
}
