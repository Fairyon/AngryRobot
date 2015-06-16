import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.SensorPort;


public class Main {
  
  protected final static int width = 114; // every cm
  protected final static int length = 240; // every cm
  protected final static int candiam = 6; // size in cm
  protected final static int grabberlen = 13; // size in cm
  
  protected final static MotorPort leftMotorPort = MotorPort.C;
  protected final static MotorPort rightMotorPort = MotorPort.B;
  protected final static MotorPort grabMotorPort = MotorPort.A;
  
  protected final static SensorPort lightSensorPort = SensorPort.S3;
  protected final static SensorPort canTouchSensorPort = SensorPort.S4;
  protected final static SensorPort usSensorPort = SensorPort.S1;
  
  protected final static int grabMotorSpeed = 250;
  protected final static int rotationSpeed = 200;

  public static void main(String[] args) {
    //Button.waitForAnyPress();
    
    Controller c = new Controller();
    c.start();

    Button.waitForAnyPress();
    c.stop();
    try {
      c.join();
    } catch (InterruptedException e) {
      System.out.println("'Stop' interrupted");
      e.printStackTrace();
    }
    System.exit(0);
  }

}
