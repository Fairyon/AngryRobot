import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;

public class CokeUltrasonic {
  
  final float rangefactor = 1.f;
  UltrasonicSensor uss;

  public CokeUltrasonic(SensorPort port) {
    uss = new UltrasonicSensor(port);
    uss.continuous();
  }
  
  public float getDistance(){
    return uss.getRange() * rangefactor;
  }
  
  public int off(){
    return uss.off();
  }

  /*public UltrasonicSensorExtended(I2CPort port) {
    super(port);
    this.port = (SensorPort) port;
    this.continuous();
    lastDistance = getDistance();
  }*/
  
 
}