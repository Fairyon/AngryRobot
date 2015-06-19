public class Controller extends Thread {
	private static final int distanceDelta = 10;

	private ColaBot robot;

	// private UsMapListener usListener;,

	protected Controller() {
		this.robot = new ColaBot();
		// this.usListener = new UsMapListener(robot, map);
	}

	/**
	 * Tries to find a can
	 * 
	 * @return true if the robot sucessfully returned a can
	 */
	public boolean findCan() {
		// usListener.setEnabled(true);

		try {
			// Find possoble location of a can
			Point canPosition = lookForCan();

			if (canPosition == null)
				return false;

			// Total distance between robot and can
			double totalLen = robot.getUsDistance() + Main.grabberlen;

			// tangens of angle; 6 = half width of robot
			double tan = totalLen / 6;

			// angle to turn the robot directly to the can
			double angle = 90 - Math.toDegrees(Math.atan(tan));

			System.out.println("tan " + tan);
			System.out.println("angle " + angle);

			robot.rotate(-(float) angle);
			robot.travel(10 * totalLen);

			// TODO Fix!
			// while (!robot.isTouchSensorPressed()) {
			// robot.travel(10);
			// }

			// TODO Return can to home field
		} catch (InterruptedException interrupted) {
			return false;
		} finally {
			// usListener.setEnabled(false);
		}

		return true;
	}

	/**
	 * Turns the robot to the lowest captured distance within the range of the
	 * ultrasonic sensor
	 * 
	 * @param distance
	 *            Current distance to the object
	 * @param robotAngle
	 *            Current absolute angle of the robot
	 * @return absolute angle with the lowest distance
	 */
	private float turnToObject(int distance, float robotAngle)
			throws InterruptedException {
		System.out.println();
		System.out.println("Entering turnToObject");
		System.out.println("distance " + distance);
		System.out.println("angle " + robotAngle);
		System.out.println();

		int usSteps = 2;
		float minAngle = robotAngle;
		int minDistance = distance;

		// turns the sensor to the left
		for (int i = 0; i <= 90; i += usSteps) {
			robot.rotateUsTo(i);

			// TODO Remove or reduce
			Thread.sleep(50);

			int leftDistance = robot.getUsDistance();
			int leftDelta = Math.abs(distance - leftDistance);

			System.out.println("us left: " + i);
			System.out.println("leftDelta: " + leftDelta);
			System.out.println("leftDistance: " + leftDistance);
			System.out.println();

			//
			if (leftDelta > distanceDelta) {
				// Change of distance found
				break;
			} else if (leftDistance < minDistance) {
				// new value is closer than old one
				minAngle = robotAngle + i;
				minDistance = leftDistance;
			}
		}

		System.out.println();

		for (int i = usSteps; i <= 90; i += usSteps) {
			robot.rotateUsTo(-i);

			// TODO Remove or reduce
			Thread.sleep(50);

			int rightDistance = robot.getUsDistance();
			int rightDelta = Math.abs(distance - rightDistance);

			System.out.println("us right: " + i);
			System.out.println("rightDelta: " + rightDelta);
			System.out.println("rightDistance: " + rightDistance);
			System.out.println();

			if (rightDelta > distanceDelta) {
				// Change of distance found
				break;
			} else if (rightDistance < minDistance) {
				// new value is closer than old one
				minAngle = robotAngle - i;
				minDistance = rightDistance;
			}
		}

		// reset ultrasonic sensor rotation
		robot.resetUsRotation();

		System.out.println();
		System.out.println("Leaving turnToObject");
		System.out.println("minDistance " + minDistance);
		System.out.println("minAngle " + minAngle);
		System.out.println();

		return minAngle;
	}

	/**
	 * Tries to find the position of a possible can
	 * 
	 * @return Relative vector to the position of the possible can
	 */
	private Point lookForCan() throws InterruptedException {
		// Steps for the rotation
		int rotationSteps = 2;

		// Maximal angle to rotate the robot
		int rotation = 360;
		float startAngle = robot.getAngle();
		int distance = robot.getUsDistance();

		System.out.println("start angle: " + startAngle);
		System.out.println("start distance: " + distance);
		System.out.println();

		System.out.println("start rotation");
		for (int i = 0; i <= rotation; i += rotationSteps) {
			// Stepwise rotation of the robot
			robot.rotate(rotationSteps);

			// TODO Remove or reduce
			Thread.sleep(100);

			int newDistance = robot.getUsDistance();
			System.out.println("newDistance: " + newDistance);

			float rotated;
			int delta = Math.abs(newDistance - distance);
			if (delta > distanceDelta) {
				// Change of distance found

				robot.stopMovement();
				rotated = robot.getAngle();

				System.out.println();
				System.out.println("stop rotation");
				System.out.println("Rotated: " + rotated);

				// Find angle with lowest distance
				float targetAngle = turnToObject(newDistance, rotated);
				robot.rotateTo(targetAngle, false);

				// returns direction to possible can
				return Point.getDirectionVector(robot.getUsDistance(), 0);
			} else {
				// No relevant change of distance
				distance = newDistance;
			}
		}

		// No can found
		return null;
	}

	public void run() {
		robot.init();
		robot.findCan();
		// robot.addUsSensorPortListener(usListener);
		// robot.test();

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
