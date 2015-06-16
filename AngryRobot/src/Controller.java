
public class Controller extends Thread{
  
  private CokeBot robot;
  private Map map;
  
  protected Controller(){
    this.robot = new CokeBot();
    this.map = new Map(Main.length, Main.width);
  }
  
  public void run(){
    //robot.init();
    try {
      robot.rangecalibration();
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }
  
  public void stop(){
    
  }
  
}
