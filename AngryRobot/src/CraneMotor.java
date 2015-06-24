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
		if(withCan){ 
			//setAcceleration(12000);
			setSpeed(Main.craneCanSpeed);
			rotate(-45, true);
			while(isMoving()){
				if(isStalled()) stop();
			}
			setSpeed(Main.craneNormSpeed);
			//setAcceleration(6000);
		} else {
			rotateTo(-15, false);
			while(isMoving());
		}
	}

}
