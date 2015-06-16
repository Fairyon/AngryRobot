import java.awt.Point;

public class Map {
	private byte[][] map;

	public Map(int xdim, int ydim) {
		if(xdim <= 0)
			throw new IllegalArgumentException("xdim too small");
		if(ydim <= 0)
			throw new IllegalArgumentException("ydim too small");
		
		map = new byte[xdim][ydim];
	}

	public byte getValue(Point point) {
		return getValue(point.x, point.y);
	}

	public byte getValue(int x, int y) {
		return map[x][y];
	}
	
	public int getMaxX() {
		return map.length;
	}
	
	public int getMaxY() {
		return map[0].length;
	}
}
