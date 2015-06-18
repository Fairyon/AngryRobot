import lejos.nxt.Button;
import lejos.nxt.LightSensor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.TouchSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Move;
import lejos.robotics.navigation.MoveListener;
import lejos.robotics.navigation.MoveProvider;

public class ColaBot {

  private final float distanceFactor = 0.09639f;
  private final float rotationFactor = 0.995f;
	private final DifferentialPilot pilot;
	protected final RegulatedMotor leftMotor;
	protected final RegulatedMotor rightMotor;
	protected final GrabMotor grabMotor;
	protected final LightSensor lightSensor;
	protected final TouchSensor canTouchSensor;
	protected final ColaUltrasonicSensor usSensor;

	protected Point curpos;
	protected Point usPosition;
	protected float curangle;

	private boolean isMoving;

	public ColaBot() {
		this.leftMotor = new NXTRegulatedMotor(Main.leftMotorPort);
		this.rightMotor = new NXTRegulatedMotor(Main.rightMotorPort);
		this.grabMotor = new GrabMotor(Main.grabMotorPort);
		this.lightSensor = new LightSensor(Main.lightSensorPort);
		this.canTouchSensor = new TouchSensor(Main.canTouchSensorPort);
		this.usSensor = new ColaUltrasonicSensor(Main.usSensorPort);
		this.pilot = new ColaDifferentialPilot(56, 142 - 26, leftMotor,
				rightMotor);

		curpos = new Point(20, 20);
		curangle = 0;
	}

	public void init() {
		usSensor.continuous();
		lightSensor.setFloodlight(true);
		pilot.addMoveListener(new PositionKeeper());
		//calibrate();
	}

	public void stop() {
		pilot.stop();
		grabMotor.stop();
		lightSensor.setFloodlight(false);
		usSensor.off();
	}

	private void calibrate() {
		int minx = 300;
		int miny = 300;
		int curdist;
		float angle;
		float minxangle = 360;
		float minyangle = 360;
		this.pilot.setRotateSpeed(100);
		pilot.rotate(-45, false); // TODO mapping
		pilot.rotate(-90, true); // look to left bottom corner
		while (pilot.isMoving()) {
			curdist = usSensor.getDistance();
			angle = grabMotor.getTachoCount();
			if (curdist < miny) {
				miny = curdist;
				minyangle = -45 + angle;
			}
		}
		miny += Main.grabberlen;
		pilot.rotate(-90, true);
		while (pilot.isMoving()) { // look to left top corner
			curdist = usSensor.getDistance();
			angle = grabMotor.getTachoCount();
			if (curdist < minx) {
				minx = (int) curdist;
				minxangle = -135 + angle;
			}
		}
		curangle = ((-90-minyangle)+(-180-minxangle))/2 + 135;
		minx += Main.grabberlen;
		System.out.println("Startpos: " + minx + ", " + miny);
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
	
	protected float getUSAngle() {
    return getAngle()+grabMotor.getTachoCount();
  }

	protected float getAngle() {
		if (isMoving) {
			Move event = pilot.getMovement();
			if (event.getMoveType().equals(Move.MoveType.ROTATE)) {
				return curangle + event.getAngleTurned();
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

	protected Point lookForCan() {
		// TODO Richtigen Punkt zurueckgeben
		return new Point(0, 0);
	}

	public void travelToCan() {

	}

	protected void getCan() {

	}

	private class PositionKeeper implements MoveListener {

		public void moveStarted(Move event, MoveProvider mp) {
			isMoving = true;
		}

		@Override
		public void moveStopped(Move event, MoveProvider mp) {
			if (event.getMoveType().equals(Move.MoveType.ROTATE)) {
				curangle += event.getAngleTurned()*rotationFactor;
			} else {
				curpos.moveAt(event.getDistanceTraveled()*distanceFactor, curangle);
			}
			isMoving = false;
		}

	}

	protected void test() {
	  pilot.setRotateSpeed(100);
	  pilot.setTravelSpeed(150);
	  //pilot.travel(1000);
    pilot.rotate(90);
    //pilot.travel(-1000);
    pilot.rotate(90);
    pilot.rotate(90);
    pilot.rotate(90);
	  Button.waitForAnyPress();
		/*grabMotor.setSpeed(50);
		grabMotor.rotateTo(90, true);
		float minrange = 300, range, mindeg = 0;
		while (grabMotor.isMoving()) {
			if ((range = usSensor.getRange()) < minrange) {
				minrange = range;
				mindeg = grabMotor.getTachoCount();
			}
		}
		pilot.rotate(mindeg);
		grabMotor.lookAhead();*/
	}
}
