import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;

public class CalibratedDifferentialPilot extends DifferentialPilot {

	private static double rotationFactor = 1;

	public CalibratedDifferentialPilot(double wheelDiameter, double trackWidth,
			RegulatedMotor leftMotor, RegulatedMotor rightMotor) {
		super(wheelDiameter, trackWidth, leftMotor, rightMotor);
		this.setRotateSpeed(Main.rotationSpeed);
	}

	public void rotate(final double angle, final boolean immediateReturn) {
		super.rotate(angle * rotationFactor, immediateReturn);
	}
}
