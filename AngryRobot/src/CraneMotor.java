import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.TachoMotorPort;

public class CraneMotor extends NXTRegulatedMotor {

	public CraneMotor(TachoMotorPort port) {
		super(port);
		this.setSpeed(Main.grabMotorSpeed);
	}
	
	public void down(){
		this.rotateTo(0, false);
		while(isMoving());
	}
	
	public void up(boolean withCan){
		//setAcceleration(12000);
		if(withCan) setSpeed(Main.craneCanSpeed);
		rotateTo(-15, false);
		if(withCan) setSpeed(Main.craneNormSpeed);
		//setAcceleration(6000);
		while(isMoving());
	}

}
