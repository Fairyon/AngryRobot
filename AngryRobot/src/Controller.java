public class Controller extends Thread {

  private CokeBot robot;
  private Map map;

  protected Controller() {
    this.robot = new CokeBot();
    this.map = new Map(Main.length, Main.width);
  }

  public void run() {
    try {
      // robot.init();
      robot.rangecalibration();
    } finally {
      robot.stop();
    }
  }

  public void stop() {
    this.interrupt();

    try {
      this.join();
    } catch (InterruptedException e) {
      System.out.println("'Stop' interrupted");
      e.printStackTrace();
    }
  }

}
