import lejos.nxt.Button;
import lejos.nxt.LightSensor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.TouchSensor;
import lejos.nxt.comm.RConsole;
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
    this.pilot = new ColaDifferentialPilot(56, 142 - 26, leftMotor, rightMotor);

    curpos = new Point(20, 20);
    curangle = 0;
  }

  public void init() {
    usSensor.continuous();
    lightSensor.setFloodlight(true);
    pilot.addMoveListener(new PositionKeeper());
  }

  public void stop() {
    pilot.stop();
    grabMotor.stop();
    lightSensor.setFloodlight(false);
    usSensor.off();
  }

  /**
   * @param corner
   * @return
   */
  public boolean[] calibrate(int corner) {
    if(pilot.isMoving()||pilot.isStalled()) return null;
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
    float relangle = (curangle+360)%360 - edgeangle;
    System.out.println(relangle);
    float oStartAngle = 0; // obstacle start angle
    float oEndAngle = 180;
    int direction; // -1 = left, +1 = right

    if (relangle > 180)
      relangle -= 360;
    else if (relangle < -180)
      relangle += 360;

    direction = (int) Math.signum(-relangle);

    pilot.rotate(-direction * 90 - relangle); // 90 to the l(-1)/r of the edge
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
        && min1 >= 25 && min1<255) {
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
        && min2 >= 25 && min1<255) {
      calibrated += 2;
    }
    pilot.rotate(-direction * 90 + relangle); // rotate back
    switch (calibrated) {
      case 0:
        System.out.println("Keine Messungen!");
        return new boolean[] { false, hasObstacles };
      case 1:
        calcCalibration(corner, min1, minangle1, direction>0);
        break;
      case 2:
        calcCalibration(corner, min2, minangle2, direction<0);
        break;
      case 3:
        calcCalibration(corner, min1, minangle1, direction>0);
        calcCalibration(corner, min2, minangle2, direction<0);
        break;
      default:
          System.out.println("o_O Calib Error!");
    }
    System.out.println("Dist: "+min1+", "+min2);
    System.out.println("Pos: " + curpos);
    System.out.println("Angle: " + curangle);
    System.out.println(hasObstacles);
    System.out.println(obstacleGeer);
    return new boolean[] { true, hasObstacles };
  }
  
  private void calcCalibration(int corner, int dist, float angle, boolean side){
    switch(corner){
      case 0: // top right
        if(side){ // right
          curpos.setX(Main.length-dist-Main.grabberlen);
          curangle = -angle;
        } else {
          curpos.setY(Main.width-dist-Main.grabberlen);
          curangle = 90-angle;
        }
        break;
      case 1: // top left
        if(side){ // right
          curpos.setY(Main.width-dist-Main.grabberlen);
          curangle = 90-angle;
        } else {
          curpos.setX(dist+Main.grabberlen);
          curangle = 180-angle;
        }
        break;
      case 2: // bottom left
        if(side){ // right
          curpos.setX(dist+Main.grabberlen);
          curangle = 180-angle;
        } else {
          curpos.setY(dist+Main.grabberlen);
          curangle = 270-angle;
        }
        break;
      case 3: // bottom right
        if(side){ // right
          curpos.setY(dist+Main.grabberlen);
          curangle = 270-angle;
        } else {
          curpos.setX(Main.length-dist-Main.grabberlen);
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
      } else {
        return curpos.pointAt(event.getDistanceTraveled(), curangle).pointAt(
            Main.distToEyes, Main.angleToEyes + curangle);
      }

    } else {
      return curpos.pointAt(Main.distToEyes, Main.angleToEyes + curangle);
    }
  }

  protected float getUSAngle() {
    float angle = getAngle() + grabMotor.getTachoCount();
    if(angle<-180) angle+=360;
    else if(angle>180) angle-=360;
    return angle;
  }

  protected float getAngle() {
    if (isMoving) {
      Move event = pilot.getMovement();
      if (event.getMoveType().equals(Move.MoveType.ROTATE)) {
        float angle = curangle + event.getAngleTurned();
        if(angle<-180) angle+=360;
        else if(angle>180) angle-=360;
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
        curangle += event.getAngleTurned() * rotationFactor;
        if(curangle<-180) curangle+=360;
        else if(curangle>180) curangle-=360;
      } else {
        curpos.moveAt(event.getDistanceTraveled() * distanceFactor, curangle);
      }
      isMoving = false;
    }

  }

  protected void test() {
    pilot.setRotateSpeed(100);
    pilot.setTravelSpeed(150);
    // pilot.travel(1000);
    pilot.rotate(90);
    // pilot.travel(-1000);
    pilot.rotate(90);
    pilot.rotate(90);
    pilot.rotate(90);
    Button.waitForAnyPress();
    pilot.rotate(-90);
    // pilot.travel(-1000);
    pilot.rotate(-90);
    pilot.rotate(-90);
    pilot.rotate(-90);
    Button.waitForAnyPress();
    /*
     * grabMotor.setSpeed(50); grabMotor.rotateTo(90, true); float minrange =
     * 300, range, mindeg = 0; while (grabMotor.isMoving()) { if ((range =
     * usSensor.getRange()) < minrange) { minrange = range; mindeg =
     * grabMotor.getTachoCount(); } } pilot.rotate(mindeg);
     * grabMotor.lookAhead();
     */
  }
}
