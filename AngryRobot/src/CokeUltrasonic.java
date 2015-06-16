import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;

public class CokeUltrasonic extends UltrasonicSensor{
  
  final float rangefactor = 1.f;

  public CokeUltrasonic(SensorPort port) {
    super(port);
  }
  
  public float getRange(){
    return this.getDistance() * rangefactor;
  }  

  /*public UltrasonicSensorExtended(I2CPort port) {
    super(port);
    this.port = (SensorPort) port;
    this.continuous();
    lastDistance = getDistance();
  }*/
  
 
}