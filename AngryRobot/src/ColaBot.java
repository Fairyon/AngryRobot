import java.lang.annotation.ElementType;
import java.util.ArrayList;

import lejos.nxt.*;
import lejos.robotics.*;
import lejos.robotics.navigation.*;

public class ColaBot {
	private static final int wallTolerance = 5;

	private static final int minWallDiff = wallTolerance;
	private static final int minX = minWallDiff;
	private static final int maxX = Main.mapWidth - minWallDiff;
	private static final int minY = minWallDiff;
	private static final int maxY = Main.mapHeight - minWallDiff;

	private static final float distanceFactor = 0.09639f;
	private static final float negdistanceFactor = 0.098f;
	private static final float rotationFactor = 0.9536f;
	private static final float negrotationFactor = 0.9515f;
	private static final float arcRotationFactor = 0.9897f;
	private static final float arcnegRotationFactor = 0.9758f;

	private static final Point homePosition = new Point(20, 20);
	
	private Waypoint[] waypoints;
	
	private int homeColor;

	private final ColaDifferentialPilot pilot;
	private final RegulatedMotor leftMotor;
	private final RegulatedMotor rightMotor;
	private final CraneMotor craneMotor;
	private final LightSensor lightSensor;
	private final TouchSensor canTouchSensor;
	private final ColaUltrasonicSensor usSensor;

	private Point curpos;
	private Point curUSpos;
	private float curangle;

	private boolean isMoving;

	public ColaBot() {
		this.leftMotor = new NXTRegulatedMotor(Main.leftMotorPort);
		this.rightMotor = new NXTRegulatedMotor(Main.rightMotorPort);
		this.craneMotor = new CraneMotor(Main.grabMotorPort);
		this.lightSensor = new LightSensor(Main.lightSensorPort);
		this.canTouchSensor = new TouchSensor(Main.canTouchSensorPort);
		this.usSensor = new ColaUltrasonicSensor(Main.usSensorPort);
		this.pilot = new ColaDifferentialPilot(56, 138 - 26, leftMotor, rightMotor);

		curpos = new Point(20, 20);
		curUSpos = curpos.pointAt(Main.distToEyes, Main.angleToEyes + curangle);
		curangle = 0;
	}

	public void init() {
		usSensor.continuous();
		lightSensor.setFloodlight(true);
		pilot.addMoveListener(new PositionKeeper());

		pilot.setRotateSpeed(75);
		pilot.setTravelSpeed(150);

		craneMotor.setSpeed(Main.craneNormSpeed);
		craneMotor.up(false);
		
		initWaypoints();
		
	}
	
	
	protected void makeWork(){
		homeColor = lightSensor.getNormalizedLightValue();
		int wayposindex=0;
		while(true){
			calibrate();
			while(!findCan(waypoints[wayposindex].endAngle)){
				wayposindex++;
				wayposindex%=18;
				travelTo(waypoints[wayposindex].pos,30);
				rotateTo(waypoints[wayposindex].startAngle);
			}
		}
	}
	
	private void initWaypoints(){
		waypoints = new Waypoint[18];
		waypoints[0] = new Waypoint(new Point(20,20),0,90);
		waypoints[1] = new Waypoint(new Point(20,60),90,-90);
		waypoints[2] = new Waypoint(new Point(20,100),0,-90);
		waypoints[3] = new Waypoint(new Point(60,20),0,180);
		waypoints[4] = new Waypoint(new Point(60,60),0,360);
		waypoints[5] = new Waypoint(new Point(60,100),0,-180);
		waypoints[6] = new Waypoint(new Point(100,20),0,180);
		waypoints[7] = new Waypoint(new Point(100,60),0,360);
		waypoints[8] = new Waypoint(new Point(100,100),0,-180);
		waypoints[9] = new Waypoint(new Point(140,20),0,180);
		waypoints[10] = new Waypoint(new Point(140,60),0,360);
		waypoints[11] = new Waypoint(new Point(140,100),0,-180);
		waypoints[12] = new Waypoint(new Point(180,20),0,180);
		waypoints[13] = new Waypoint(new Point(180,60),0,360);
		waypoints[14] = new Waypoint(new Point(180,100),0,-180);
		waypoints[15] = new Waypoint(new Point(220,20),90,180);
		waypoints[16] = new Waypoint(new Point(220,60),90,270);
		waypoints[17] = new Waypoint(new Point(220,100),180,270);
	}
	

	public void stop() {
		pilot.stop();
		craneMotor.down();
		lightSensor.setFloodlight(false);
		usSensor.off();
	}

	public void addUsSensorPortListener(SensorPortListener listener) {
		if (listener == null)
			throw new NullPointerException("listener is null");

		usSensor.addSensorPortListener(listener);
	}

	public void loweringCrane() {
		craneMotor.down();
	}

	public void raisingCrane(boolean withCan) {
		craneMotor.up(withCan);
	}

	public void usRotate(float angle, boolean immediateReturn) {
		pilot.usRotate(angle, immediateReturn);
	}

	public void usRotate(float angle) {
		usRotate(angle, false);
	}
	
	public void lookAt (Point pos){
		usRotateTo(getUsPosition().getAngleTo(pos, true), false);
	}

	public void usRotateTo(float angle) {
		usRotateTo(angle, false);
	}

	public void usRotateTo(float angle, boolean immediateReturn) {
		float relAngle = angle - getAngle();
		
		pilot.usRotate(relAngle, immediateReturn);
	}
	
	public boolean usCanRotate(float angle) {
		return canRotate(curangle, angle);
	}

	public boolean usCanRotate(float fromAngle, float angle) {
		if(angle>=0){
			while ((angle -= 5) > 0) {
				if (!isValidPoint(getPosition()
						.pointAt(Main.distFromEyesToEdge, fromAngle + angle))){
					return false;
				}
			}
		} else {
			while ((angle += 5) < 0) {
				if (!isValidPoint(getPosition()
						.pointAt(Main.distFromEyesToEdge, fromAngle + angle))){
					return false;
				}
			}
		}
		return true;
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
	
	public float getAbsCanAngle(float usDistance, float usAngle){
		return (float) (usAngle+Math.atan(Main.distToEyes/usDistance));
	}

	private boolean isValidPoint(Point p) {
		float tarX = p.getX();
		float tarY = p.getY();

		return tarX >= minX && tarX <= maxX && tarY >= minY && tarY <= maxY;
	}
	
	public boolean canTravel(float distance){
		return canTravel(curpos, distance, curangle);
	}
	
	public boolean canTravel(float distance, float angle){
		return canTravel(curpos, distance, angle);
	}
	
	public boolean canTravel(Point fromPos, float distance, float angle){
		return canTravel(fromPos.pointAt(distance, angle));
	}

	private boolean canTravel(Point toPos) {
		return isValidPoint(toPos);
	}

	/**
	 * Lets the robot travel a specified range
	 * 
	 * @param distance
	 *          Range to travel
	 * @param immediateReturn
	 *          If true this method returns immediately
	 */
	public void travel(float distance, boolean immediateReturn) {

		pilot.travel(distance*10, immediateReturn);

	}

	/**
	 * Lets the robot travel a specified range
	 * 
	 * @param distance
	 *          Range to travel
	 */
	public void travel(float distance) {
		travel(distance, false);
	}

	public void travelTo(Point pos, int checkdist) {
		Point direct = pos.getDirectionTo(getPosition());
		System.out.println(direct.getAngleBetween(new Point(1,0), true));
		/*float distance = direct.getLength();
		float angle = direct.getAngleBetween(new Point(1,0), true)-getAngle();
		
		rotate(angle);
		travel(distance, true);
		Point startpos=getPosition();
		int i=checkdist;
		while(pilot.isMoving()){
			if(getPosition().getDistance(startpos)>i){
				i+=checkdist;
				if(!checkWay()){
					travelTo(pos, 10);
				} else {
					travelTo(pos, 30);
				}
			}
		}*/
		
		travelTo( direct.getLength(), direct.getAngleBetween(new Point(1,0), true)-getAngle());
	}

	public void travelTo(float distance, float angle) {
		rotate(angle);
		travel(distance);
	}
	
	/*private boolean checkWay(){
		travel(-3);
		usRotate(30, true);
		while(pilot.isMoving()){
			if(usSensor.getDistance()<Main.distFromEyesToEdge+8){
				pilot.stop();
				travelTo(10,-70);
				return false;
			}
		}
		usRotate(-30, false);
		usRotate(-15, true);
		while(pilot.isMoving()){
			if(usSensor.getDistance()<Main.distFromEyesToEdge+8){
				pilot.stop();
				travelTo(10,70);
				return false;
			}
		}
		return true;
	}*/

	/**
	 * Check if the sensor is pressed
	 * 
	 * @return true if sensor is pressed, false otherwise
	 */
	public boolean isTouchSensorPressed() {
		return canTouchSensor.isPressed();
	}

	private int getCorner(Point pos){
		if(pos.getX()>Main.mapWidth/2){
			if(pos.getY()>Main.mapHeight/2){
				return 0;
			} else {
				return 3;
			}
		} else {
			if(pos.getY()>Main.mapHeight/2){
				return 1;
			} else {
				return 2;
			}
		}
	}
	
	public boolean[] calibrate() {
		int corner = getCorner(getPosition());
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
		float oStartAngle = 0; // obstacle start angle
		float oEndAngle = 180;
		int direction; // -1 = left, +1 = right

		float relangle = (curangle + 360) % 360 - edgeangle;
		
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
					curpos.setX(Main.mapWidth - dist);
					curangle = -angle;
				} else {
					curpos.setY(Main.mapHeight - dist);
					curangle = 90 - angle;
				}
				break;
			case 1: // top left
				if (side) { // right
					curpos.setY(Main.mapHeight - dist);
					curangle = 90 - angle;
				} else {
					curpos.setX(dist);
					curangle = 180 - angle;
				}
				break;
			case 2: // bottom left
				if (side) { // right
					curpos.setX(dist);
					curangle = 180 - angle;
				} else {
					curpos.setY(dist);
					curangle = 270 - angle;
				}
				break;
			case 3: // bottom right
				if (side) { // right
					curpos.setY(dist);
					curangle = 270 - angle;
				} else {
					curpos.setX(Main.mapWidth - dist);
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
				return curpos.pointAt(Main.distToEyes, Main.angleToEyes + curangle
						+ event.getAngleTurned());
			} else if (event.getMoveType().equals(Move.MoveType.ARC)) {
				return curUSpos;
			} else {
				return curpos.pointAt(event.getDistanceTraveled(), curangle).pointAt(
						Main.distToEyes, Main.angleToEyes + curangle);
			}
		} else {
			return curUSpos;
		}
	}

	protected float getCraneTachoCount() {
		return craneMotor.getTachoCount();
	}

	protected float getAngle() {
		float angle;
		if (isMoving) {
			Move event = pilot.getMovement();
			if (event.getMoveType().equals(Move.MoveType.ROTATE)) {
				float tmpangle = event.getAngleTurned();
				tmpangle*=(tmpangle>0)?rotationFactor:negrotationFactor;
				angle = curangle + tmpangle;
			} else if (event.getMoveType().equals(Move.MoveType.ARC)) {
				float tmpangle = event.getAngleTurned();
				tmpangle*=(tmpangle>0)?arcRotationFactor:arcnegRotationFactor;
				angle = curangle + tmpangle;
			} else {
				angle = curangle;
			}
		} else {
			angle = curangle;
		}
		if (angle < -180)
			angle += 360;
		else if (angle > 180)
			angle -= 360;
		return angle;
	}

	protected Point getPosition() {
		if (isMoving) {
			Move event = pilot.getMovement();
			if (event.getMoveType().equals(Move.MoveType.ROTATE)) {
				return curpos;
			} else if (event.getMoveType().equals(Move.MoveType.ARC)) {
				float angle = event.getAngleTurned();
				angle*=(angle>0)?arcRotationFactor:arcnegRotationFactor;
				return curUSpos.pointAt(Main.distToEyes, angle - Main.angleToEyes);
			} else {
				float distance = event.getDistanceTraveled();
				distance*=(distance>0)?distanceFactor:negdistanceFactor;
				return curpos.pointAt(distance, curangle);
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
	 *          The wanted angle of rotation in degrees. Positive angle rotate
	 *          left (anti-clockwise), negative right.
	 * @param immediateReturn
	 *          If true this method returns immediately.
	 */
	public void rotate(float angle, boolean immediateReturn) {
		if (angle < -180)
			angle += 360;
		else if (angle > 180)
			angle -= 360;
		
		pilot.rotate(angle, immediateReturn);
	}
	
	public boolean canRotate(float angle) {
		return canRotate(curangle, angle);
	}

	public boolean canRotate(float fromAngle, float angle) {
		if(angle>=0){
			while ((angle -= 5) > 0) {
				if (!isValidPoint(getPosition()
						.pointAt(Main.craneLength, fromAngle + angle))){
					return false;
				}
			}
		} else {
			while ((angle += 5) < 0) {
				if (!isValidPoint(getPosition()
						.pointAt(Main.craneLength, fromAngle + angle))){
					return false;
				}
			}
		}
		return true;
	}

	/**
	 * Rotates the robot through a specific angle
	 * 
	 * @param angle
	 *          The wanted angle of rotation in degrees. Positive angle rotate
	 *          left (anti-clockwise), negative right.
	 */
	public void rotate(float angle) {
		rotate(angle, false);
	}

	/**
	 * Rotates the robot to the specific angle
	 * 
	 * @param angle
	 *          The wanted angle of the robot after the rotation in degrees
	 * @param immediateReturn
	 *          If true this method returns immediately.
	 */
	public void rotateTo(float absoluteAngle, boolean immediateReturn) {
		float relativeAngle = absoluteAngle - getAngle();
		rotate(relativeAngle, immediateReturn);
	}

	/**
	 * Rotates the robot to the specific angle
	 * 
	 * @param angle
	 *          The wanted angle of the robot after the rotation in degrees
	 */
	public void rotateTo(float absoluteAngle) {
		rotateTo(absoluteAngle, false);
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
				float angle = event.getAngleTurned();
				angle*=(angle>0)?rotationFactor:negrotationFactor;
				curangle += angle;
			} else if (event.getMoveType().equals(Move.MoveType.ARC)) {
				float angle = event.getAngleTurned();
				angle*=(angle>0)?arcRotationFactor:arcnegRotationFactor;
				curangle += angle;
				curpos = curUSpos.pointAt(Main.distToEyes, angle - Main.angleToEyes);
			} else {
				float distance = event.getDistanceTraveled();
				distance*=(distance>0)?distanceFactor:negdistanceFactor;
				curpos.moveAt(distance, curangle);
			}
			curUSpos = curpos.pointAt(Main.distToEyes, Main.angleToEyes + curangle);
			isMoving = false;
		}
	}
	
	public void liftCan(){
		craneMotor.up(true);
	}
	
	private void setCanDown(){
		craneMotor.down();
		travel(-Main.candiam-5);
		craneMotor.up(false);
	}
	
	public boolean travelHome(){
		int color;
		travelTo(new Point(20,20),3000);
		color = lightSensor.getNormalizedLightValue();
		if(Math.abs(homeColor-color)<50) return true;
		calibrate();
		return travelHome();
	}

	protected boolean findCan(int angle) {
		pilot.setTravelSpeed(75);
		float angleBackup = getAngle();
		Point usPos = getUsPosition();
		
		Can can = lookForCan(angle, 3);
		pilot.setTravelSpeed(150);
		if (can == null)
			return false;

		Point absCanPos = usPos.pointAt(can.getDistance(), can.getAngle());

		float absCanAngle = getAbsCanAngle(can.getDistance(), can.getAngle());

		if (canTravel(absCanPos)) {
			travelTo(can.getDistance()-Main.craneLength-5, absCanAngle-getAngle());
			
			if(!isThereCan(absCanAngle)){
				findCan(360);
				//TODO better search for the can
			} else {
				craneMotor.down();
				travel(Main.craneLength+5, true);
				boolean canFound = false;
				while(pilot.isMoving()){
					if (isTouchSensorPressed()) {
						canFound=true;
						pilot.stop();
						Sound.beep();
					}
				}
				while(!canFound){
					craneMotor.up(false);
					travel(-Main.craneLength-5);
					if(!isThereCan(absCanAngle)) break;
					craneMotor.down();
					travel(Main.craneLength+5, true);
					while(pilot.isMoving()){
						if (isTouchSensorPressed()) {
							canFound=true;
							pilot.stop();
							Sound.beep();
						}
					}
				}
				if(canFound){
					liftCan();
					travelHome();
					craneMotor.down();
					setCanDown();
					
					rotateTo(0);
					return true;
				}
			}
		} else {
			//calibrate(0);
			rotateTo(angleBackup);
			return findCan(angle);
		}

		return false;
	}

	protected boolean isThereCan(float canAngle) {
		rotateTo(canAngle);
		usRotate(35);
		usRotate(-50, true);
		while(pilot.isMoving()){
			if(usSensor.getDistance()<Main.distFromEyesToEdge+8){
				usRotate(-25);
				pilot.stop();
				return true;
			}
		}
		return false;
	}

	/**
	 * 
	 * @param startAngle
	 * @param angle
	 * @return Polar with distance and angle to the next can relative to robot
	 */
	protected ArrayList<Can> lookForCans(int angle, int count) {

		float startAngle = getAngle();
		float oldAngle, newAngle;
		int oldRange, newRange;
		int delta;
		CanAdder helpThread = null;
		Can tmpCan = new Can();
		ArrayList<Can> cans = new ArrayList<Can>();

		oldAngle = getAngle();

		oldRange = usSensor.getDistance();
		boolean rotation = false; // false - from start, true - back
		int iter=0;
		while (count-- > 0) {
			usRotateTo(rotation ? startAngle : angle, true);
			while (pilot.isMoving()) {
				iter++;
				newAngle = getAngle();
				newRange = usSensor.getDistance();
				delta = oldRange - newRange;
				if ((!rotation) && (delta > Main.minimalDelta) && (newRange < 200)) {
					// if new angle at object
					helpThread = new CanAdder(cans, tmpCan, newRange, newAngle);
					helpThread.start();
				} else if (rotation && (delta < -Main.minimalDelta) && (oldRange < 200)) {
					// if old angle at object
					helpThread = new CanAdder(cans, tmpCan, oldRange, oldAngle);
					helpThread.start();
				}
				oldRange = newRange;
				oldAngle = newAngle;
			}
			rotation = !rotation;
		}
		System.out.println(iter);

		try {
			if (helpThread != null)
				helpThread.join();
		} catch (InterruptedException e) {
			return null;
		} 

		// System.out.println("can size: " + cans.size());
		if (cans.size() > 0) {
			return cans;
		}

		return null;
	}

	/**
	 * 
	 * @param startAngle
	 * @param angle
	 * @return Polar with distance and angle to the next can relative to robot
	 */
	protected Can lookForCan(int angle, int count) {
		ArrayList<Can> cans = lookForCans(angle, count);
		if (cans == null)
			return null;
		if (cans.size() > 0) {
			Can bestCan = cans.get(0);
			int i=0;
			for (Can can : cans) {
				System.out.println(can);
				if(i==0) {
					i++;
					continue;
				}
				if (can.getCount() > bestCan.getCount()) {
					bestCan = can;
				} else if (can.getCount() == bestCan.getCount()
						&& can.getDistance() < bestCan.getDistance()) {
					bestCan = can;
				}
			}

			return bestCan;
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
}
