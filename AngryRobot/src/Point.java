public class Point {
  /**
   * The x coordinate of the point
   */
  public float x;
  /**
   * The y coordinate of the point
   */
  public float y;
  
  /**
   * Create a point at (0,0) with float coordinates
   */
  public Point() {}
  
  /**
   * Create a point at (x,y) with float coordinates
   * 
   * @param x the x coordinate
   * @param y the y coordinate
   */
  public Point(float x, float y) {
    this.x = x;
    this.y = y;
  }
  
  public Point(double x, double y) {
    this.x = (float) x;
    this.y = (float) y;     
  }

  public float getX() {
    return x;
  }

  public float getY() {
    return y;
  }
  
  /**
   * Set the new location of the point
   * 
   * @param x the new x coordinate
   * @param y the new y coordinate
   */
  public void setLocation(float x, float y) {
    this.x = x;
    this.y = y;     
  }
  
  /**
   * Set the new location of the point
   * 
   * @param x the new x coordinate
   * @param y the new y coordinate
   */
  public void setLocation(int x, int y) {
    this.x = x;
    this.y = y;     
  }
  
  /**
   * Set the new location of the point
   * 
   * @param dx the x distance to the new x coordinate
   * @param dy the x distance to the new y coordinate
   */
  public void moveBy(float dx, float dy) {
    this.x += dx;
    this.y += dy;     
  }
  
  /**
   * Set the new location of the point at the specified distance 
   * in the direction angle from current Position.
   * @param distance the distance to the new point
   * @param angle the angle to the new point
   * @return the new point
   */
  public void moveAt(float distance, float angle)
  {
    this.x = (float) (distance * Math.cos(Math.toRadians(angle)) + getX());
    this.y = (float) (distance * Math.sin(Math.toRadians(angle)) + getY());
  }

  /**
   * Represent the Point2SD.Float as a String
   */
  @Override
    public String toString() {
        return "("+x+", "+y+")";
    }

  /**
   * Returns a new point at the specified distance in the direction angle  from
   * this point.
   * @param distance the distance to the new point
   * @param angle the angle to the new point
   * @return the new point
   */
  public Point pointAt(float distance, float angle)
  {
    double xx = distance * Math.cos(Math.toRadians(angle)) + getX();
    double yy = distance * Math.sin(Math.toRadians(angle)) + getY();
    return new Point(xx,yy);
  }
}
