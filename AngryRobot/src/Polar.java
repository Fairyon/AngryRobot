public class Polar {

	private int distance;
	private float angle;

	static public Polar getMean(Polar p1, Polar p2) {
		return new Polar((p1.distance + p2.distance) / 2,
				(p1.angle + p2.angle) / 2);
	}

	/**
	 * Create a polar at distance=0; angle=0;
	 */
	public Polar() {
	}

	public Polar(int distance, float angle) {
		this.distance = distance;
		this.angle = angle;
	}

	public Polar(Polar pol) {
		this.distance = pol.distance;
		this.angle = pol.angle;
	}
	
	public Polar(Point pos){
		this.distance = (int) pos.getDistance(new Point(0,0));
		this.angle = pos.getAngleBetween(new Point(1,0), true);
	}

	public float getDistance() {
		return this.distance;
	}

	public float getAngle() {
		return this.angle;
	}

	public void moveTo(int distance, float angle) {
		this.distance = distance;
		this.angle = angle;
	}

	protected void setDistance(int distance) {
		this.distance = distance;
	}

	protected void setAngle(float angle) {
		this.angle = angle;
	}

	public Polar clone() {
		return new Polar(this);
	}

	public String toString() {
		return ((int) angle) + " deg, " + distance + " cm";
	}
}
