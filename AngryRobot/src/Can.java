
public class Can {

	Polar pos;
	Point coords;
	int count;
	
	public Can(Polar position){
		this.pos=position;
		this.coords=new Point(position);
	}
	
	public boolean equals(Can can){
		return 	can.coords.getX()-Main.candiam <= this.coords.getX() && 
						can.coords.getX()+Main.candiam >= this.coords.getX() && 
						can.coords.getY()-Main.candiam <= this.coords.getY() && 
						can.coords.getY()+Main.candiam >= this.coords.getY();
	}
}
