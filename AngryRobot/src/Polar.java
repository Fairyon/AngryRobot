public class Polar {

	private int distance;
	private float angle;

	/**
	 * Create a polar at distance=0; angöe=0;
	 */
	public Polar() {
	}

	public Polar(int distance, float angle) {
		this.distance = distance;
		this.angle = angle;
	}

	public float getDistance() {
		return this.distance;
	}

	public float getAngle() {
		return this.angle;
	}

	protected void setDistance(int distance) {
		this.distance = distance;
	}

	protected void setAngle(float angle) {
		this.angle = angle;
	}
}
