import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;

public class ColaDifferentialPilot extends DifferentialPilot {

	private static double rotationFactor = 1.01;
	private static double travelFactor = 1.037;

	private double trackWidth;
	
	public ColaDifferentialPilot(double wheelDiameter, double trackWidth,
			RegulatedMotor leftMotor, RegulatedMotor rightMotor) {
		super(wheelDiameter, trackWidth, leftMotor, rightMotor);
		
		this.trackWidth = trackWidth;
		this.setRotateSpeed(Main.rotationSpeed);
	}

	public void rotate(final double angle) {
		rotate(angle, false);
	}

	public void rotate(final double angle, final boolean immediateReturn) {
		super.rotate(angle * rotationFactor, immediateReturn);
	}

	public void travel(double distance) {
		travel(distance, false);
	}

	public void travel(double distance, final boolean immediateReturn) {
		super.travel(distance * travelFactor, immediateReturn);
	}
	
	public void usRotate(final double angle) {
		usRotate(angle, false);
	}
	
	public void usRotate(final double angle, boolean immediateReturn) {
		//double tmp = angle / 39;
		
		this.setAcceleration((int) (3 * getMaxTravelSpeed()));
		this.arc(-trackWidth / 2, angle, immediateReturn);
		this.setAcceleration((int) (4 * getMaxTravelSpeed()));
		
		
		//this._left.rotate((int) (Math.PI * tmp * trackWidth / 2), immediateReturn);
	}
}
