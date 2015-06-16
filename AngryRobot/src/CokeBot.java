import lejos.nxt.LightSensor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.TouchSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;

public class CokeBot {
  
	private final DifferentialPilot pilot;
	protected final RegulatedMotor leftMotor;
	protected final RegulatedMotor rightMotor;
	protected final GrabMotor grabMotor;
	protected final LightSensor lightSensor;
	protected final TouchSensor canTouchSensor;
	protected final CokeUltrasonic usSensor;
	protected int[] curpos;

	public CokeBot() {
	  this.leftMotor = new NXTRegulatedMotor(Main.leftMotorPort);
	  this.rightMotor = new NXTRegulatedMotor(Main.rightMotorPort);
	  this.grabMotor = new GrabMotor(Main.grabMotorPort);
	  this.lightSensor = new LightSensor(Main.lightSensorPort);
	  this.canTouchSensor = new TouchSensor(Main.canTouchSensorPort);
	  this.usSensor = new CokeUltrasonic(Main.usSensorPort);
		this.pilot = new CalibratedDifferentialPilot(56, 142-26, leftMotor, rightMotor);
		curpos = new int[]{0, 0};
	}
	
	public void init(){
	  lightSensor.setFloodlight(true);
	  calibrateMapPosition();
	}
	
  public void stop(){
    pilot.stop();
    grabMotor.stop();
    lightSensor.setFloodlight(false);
    usSensor.off();
  }
	
	private void calibrateMapPosition(){
	  int minx = 300;
	  int miny = 300;
	  float curdist;
    this.pilot.setRotateSpeed(100);
	  pilot.rotate(-120, true); // look to left bottom corner
	  while(pilot.isMoving()){
	    curdist = usSensor.getDistance();
	    if(curdist<miny){
	      miny=(int) curdist;
	    }
	  }
	  miny+=Main.grabberlen;
	  pilot.rotate(-90, true);
    while(pilot.isMoving()){ // look to left top corner
      curdist = usSensor.getDistance();
      if(curdist<minx){
        minx=(int) curdist;
      }
    }
    minx+=Main.grabberlen;
    System.out.println("Startpos: "+minx+", "+miny);
    curpos[0]=minx;
    curpos[1]=miny;
    pilot.rotate(-150); // rotate to initial state
	}
	
	protected int[] lookForCan(){
	  
	  return new int[]{};
	}
	
	protected void getCan(){
	  
	}

}
