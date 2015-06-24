import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;

public class ColaDifferentialPilot extends DifferentialPilot {

	private static float rotationFactor = 1.05f;
	private static float arcRotationFactor = 1.01f;
	private static float arcnegRotationFactor = 1.025f;
	private static float travelFactor = 1.037f;
	private static float negtravelFactor = 1.030f;

	private double usRotateRadius;
	
	public ColaDifferentialPilot(double wheelDiameter, double trackWidth,
			RegulatedMotor leftMotor, RegulatedMotor rightMotor) {
		super(wheelDiameter, trackWidth, leftMotor, rightMotor);
		
		this.usRotateRadius = trackWidth / 2;
		this.setRotateSpeed(Main.rotationSpeed);
	}

	public void rotate(double angle) {
		rotate(angle, false);
	}

	public void rotate(double angle, boolean immediateReturn) {
		super.rotate(angle * rotationFactor, immediateReturn);
	}

	public void travel(double distance) {
		travel(distance, false);
	}

	public void travel(double distance, boolean immediateReturn) {
		if(distance<0){
			super.travel(distance * negtravelFactor, immediateReturn);
		} else {
			super.travel(distance * travelFactor, immediateReturn);
		}
	}
	
	public void usRotate(final double angle) {
		usRotate(angle, false);
	}
	
	public void usRotate(final double angle, boolean immediateReturn) {

		//this.setAcceleration((int) (3 * getMaxTravelSpeed()));
		if(angle<0){
			this.arc(-usRotateRadius, angle * arcnegRotationFactor, immediateReturn);
		} else {
			this.arc(-usRotateRadius, angle * arcRotationFactor, immediateReturn);
		}
		//this.setAcceleration((int) (4 * getMaxTravelSpeed()));

	}
}
