
public class Controller extends Thread{
  
  private CokeBot robot;
  private Map map;
  
  protected Controller(){
    this.robot = new CokeBot();
    this.map = new Map(Main.length, Main.width);
  }
  
  public void run(){
    robot.init();
  }
  
  public void stop(){
    
  }
  
}
