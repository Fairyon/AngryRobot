public class Point {

	static public Point getMean(Point p1, Point p2) {
		return new Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
	}

	static public Point getDirectionVector(float distance, float angle) {
		float x = (float) (distance * Math.cos(Math.toRadians(angle)));
		float y = (float) (distance * Math.sin(Math.toRadians(angle)));
		return new Point(x, y);
	}

	/**
	 * The x coordinate of the point
	 */
	private float x;
	/**
	 * The y coordinate of the point
	 */
	private float y;

	/**
	 * Create a point at (0,0) with float coordinates
	 */
	public Point() {
	}

	/**
	 * Create a point at (x,y) with float coordinates
	 * 
	 * @param x
	 *            the x coordinate
	 * @param y
	 *            the y coordinate
	 */
	public Point(float x, float y) {
		this.x = x;
		this.y = y;
	}

	public Point(double x, double y) {
		this.x = (float) x;
		this.y = (float) y;
	}

	public Point(Point pos) {
		this.x = pos.x;
		this.y = pos.y;
	}

	public Point(Polar pol) {
		this.moveTo(pol.getDistance(), pol.getAngle());
	}

	public float getX() {
		return x;
	}

	public float getY() {
		return y;
	}

	protected void setX(float x) {
		this.x = x;
	}

	protected void setY(float y) {
		this.y = y;
	}

	/**
	 * Set the new location of the point
	 * 
	 * @param x
	 *            the new x coordinate
	 * @param y
	 *            the new y coordinate
	 */
	public void setLocation(float x, float y) {
		this.x = x;
		this.y = y;
	}

	/**
	 * Set the new location of the point
	 * 
	 * @param x
	 *            the new x coordinate
	 * @param y
	 *            the new y coordinate
	 */
	public void setLocation(int x, int y) {
		this.x = x;
		this.y = y;
	}

	/**
	 * Set the new location of the point
	 * 
	 * @param dx
	 *            the x distance to the new x coordinate
	 * @param dy
	 *            the x distance to the new y coordinate
	 */
	public void moveBy(float dx, float dy) {
		this.x += dx;
		this.y += dy;
	}

	public void moveBy(Point vector) {
		this.x += vector.x;
		this.y += vector.y;
	}

	/**
	 * Set the new location of the point at the specified distance in the
	 * direction angle from current Position.
	 * 
	 * @param distance
	 *            the distance to the new point
	 * @param angle
	 *            the angle to the new point
	 * @return the new point
	 */
	public void moveAt(float distance, float angle) {
		this.x = (float) (distance * Math.cos(Math.toRadians(angle)) + getX());
		this.y = (float) (distance * Math.sin(Math.toRadians(angle)) + getY());
	}
	
	public void moveTo(float distance, float angle) {
		this.x = (float) (distance * Math.cos(Math.toRadians(angle)));
		this.y = (float) (distance * Math.sin(Math.toRadians(angle)));
	}

	/**
	 * Represent the Point2SD.Float as a String
	 */
	@Override
	public String toString() {
		return "(" + ((int) this.x) + ", " + ((int) this.y) + ")";
	}

	/**
	 * Returns a new point at the specified distance in the direction angle from
	 * this point.
	 * 
	 * @param distance
	 *            the distance to the new point
	 * @param angle
	 *            the angle to the new point
	 * @return the new point
	 */
	public Point pointAt(float distance, float angle) {
		Point ret = this.clone();
		ret.moveAt(distance, angle);
		return ret;
	}

	public Point clone() {
		return new Point(this);
	}

	public float getLength() {
		return getLength(this.x, this.y);
	}

	public Point getDirectionTo(Point p) {
		return new Point(p.x - this.x, p.y - this.y);
	}

	public Point addTo(Point p) {
		return new Point(p.x + this.x, p.y + this.y);
	}

	public static float getLength(float x, float y) {
		return (float) Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
	}

	public float getDistance(Point other) {
		Point vector = new Point(this.x - other.x, this.y - other.y);
		return vector.getLength();
	}

	public float getAngleBetween(Point other, boolean degrees) {
		double scalar = this.x * other.x + this.y * other.y;
		double norms = this.getLength() * other.getLength();
		double cos = scalar / norms;
		double angle = Math.acos(cos);

		if (degrees)
			return (float) Math.toDegrees(angle);
		return (float) angle;
	}

	public float getAngleTo(Point other, boolean degrees) {
		float xDiff = this.x - other.x;
		
		double scalar = xDiff * xDiff;
		double norms = getLength(xDiff, this.y - other.y)
				* getLength(xDiff, 0);
		double cos = scalar / norms;
		double angle = Math.acos(cos);

		if (degrees)
			return (float) Math.toDegrees(angle);
		return (float) angle;
	}

	public Point scalePoint(float scalar) {
		return new Point(this.x * scalar, this.y * scalar);
	}
}
