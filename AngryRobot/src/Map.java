import java.util.ArrayList;

public class Map {
	private byte[][] map;

	public Map(int xdim, int ydim) {
		if (xdim < 1)
			throw new IllegalArgumentException("xdim too small");
		if (ydim < 1)
			throw new IllegalArgumentException("ydim too small");

		map = new byte[xdim + 2][ydim + 2];
	}

	public byte getValue(Point point) throws ArrayIndexOutOfBoundsException {
		return getValue((int)point.x, (int)point.y);
	}

	public byte getValue(int x, int y) throws ArrayIndexOutOfBoundsException {
		return map[x][y];
	}

	public void increment(Point point) throws ArrayIndexOutOfBoundsException {
		increment((int)point.x, (int)point.y);
	}

	public void increment(int x, int y) throws ArrayIndexOutOfBoundsException {
		if (map[x][y] < Byte.MAX_VALUE) 
		  map[x][y]++;
	}

	public void decrement(Point point) throws ArrayIndexOutOfBoundsException {
		decrement((int)point.x, (int)point.y);
	}

	public void decrement(int x, int y) throws ArrayIndexOutOfBoundsException {
		if (map[x][y] > 0) 
		  map[x][y]--;
	}

	public void reset(Point point) throws ArrayIndexOutOfBoundsException {
		reset((int)point.x, (int)point.y);
	}

	public void reset(int x, int y) throws ArrayIndexOutOfBoundsException {
		map[x][y] = 0;
	}

	public int getMaxX() {
		return map.length;
	}

	public int getMaxY() {
		return map[0].length;
	}

	private int countLeftNeighbours(int x, int y){
		int left = 0;

		if (x > 0 && getValue(x - 1, y) > 0) {
			left += 1 + countLeftNeighbours(x - 1, y);
		}

		return left;
	}

	private int countRightNeighbours(int x, int y) {
		int right = 0;

		if (x < getMaxX() - 1 && getValue(x + 1, y) > 0) {
			right = 1 + countRightNeighbours(x + 1, y);
		}

		return right;
	}

	private int countButtomNeighbours(int x, int y) {
		int buttom = 0;

		if (y > 0 && getValue(x, y - 1) > 0) {
			buttom += 1 + countButtomNeighbours(x, y - 1);
		}

		return buttom;
	}

	private int countTopNeighbours(int x, int y) {
		int top = 0;

		if (y < getMaxY() - 1 && getValue(x, y + 1) > 0) {
			top += 1 + countTopNeighbours(x, y + 1);
		}

		return top;
	}

	private Point getSize(int x, int y) {
		int width = 1 + countLeftNeighbours(x, y) + countRightNeighbours(x, y);
		int height = 1 + countButtomNeighbours(x, y) + countTopNeighbours(x, y);

		return new Point(width, height);
	}

	public Point[] getPossibleCans(Point current) {
		int xMax = getMaxX();
		int yMax = getMaxY();

		ArrayList<Point> points = new ArrayList<Point>();

		for (int x = 0; x < xMax; x++) {
			for (int y = 0; y < yMax; y++) {
				if (getValue(x, y) < 1)
					continue;

				Point size = getSize(x, y);
				if (size.x >= Main.candiam - 2 && size.x <= Main.candiam
						&& size.y >= Main.candiam - 2 && size.y <= Main.candiam) {
					System.out.println("Possible can: " + size);
					points.add(new Point(x, y));
				}
			}
		}

		return points.toArray(new Point[points.size()]);
	}
}
