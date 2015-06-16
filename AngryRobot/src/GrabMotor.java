import lejos.nxt.LightSensor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.TachoMotorPort;
import lejos.robotics.navigation.DifferentialPilot;

public class GrabMotor extends NXTRegulatedMotor{

  public GrabMotor(TachoMotorPort port) {
    super(port);
    this.setSpeed(Main.grabMotorSpeed);
  }
  
  public void lookAhead(){
    this.rotateTo(0);
  }
  
  public void lookLeft(){
    this.rotateTo(90);
  }
  
  public void lookRight(){
    this.rotateTo(-90);
  }

}
