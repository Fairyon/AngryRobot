import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.RangeFinder;

public class CokeUltrasonic extends UltrasonicSensor{
  
  public CokeUltrasonic(SensorPort port) {
    super(port);
  }
  
  public float getRange(){
    return this.getDistance();
  }  

  /*public UltrasonicSensorExtended(I2CPort port) {
    super(port);
    this.port = (SensorPort) port;
    this.continuous();
    lastDistance = getDistance();
  }*/
  
 
}