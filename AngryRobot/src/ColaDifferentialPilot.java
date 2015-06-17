import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;

public class ColaDifferentialPilot extends DifferentialPilot {

  private static double rotationFactor = 1;
  private static double travelFactor = 1.037;

  public ColaDifferentialPilot(double wheelDiameter, double trackWidth,
      RegulatedMotor leftMotor, RegulatedMotor rightMotor) {
    super(wheelDiameter, trackWidth, leftMotor, rightMotor);
    this.setRotateSpeed(Main.rotationSpeed);
  }
  
  public void rotate(final double angle){
    rotate(angle, false);
  }

  public void rotate(final double angle, final boolean immediateReturn) {
    super.rotate(angle * rotationFactor, immediateReturn);
  }
  
  public void travel(double distance){
    travel(distance, false);
  }
  
  public void travel(double distance, final boolean immediateReturn){
    super.travel(distance * travelFactor,  immediateReturn);
  }
}
