
public class Can {

	private Polar pol;
	private Point pos;
	private byte count;

	public Can(){
		this.pol=new Polar();
		this.pos=new Point();
		this.count = 1;
	}
	
	public Can(Polar position){
		this.pol=position.clone();
		this.pos=new Point(position);
		this.count = 1;
	}
	
	public Can(Can can){
		this.pol=can.pol.clone();
		this.pos=can.pos.clone();
		this.count = can.count;
	}
	
	public Polar getPol() {
		return pol;
	}

	public Point getPos() {
		return pos;
	}

	public byte getCount() {
		return count;
	}
	
	public float getDistance(){
		return this.pol.getDistance();
	}
	
	public float getAngle(){
		return this.pol.getAngle();
	}
	
	public void changePol(int distance, float angle){
		this.pol.moveTo(distance, angle);
		this.pos.moveTo(distance, angle);
	}
	
	public boolean addCan(Can can){
		if(!equals(can)) return false;
		this.pol=Polar.getMean(this.pol, can.pol);
		this.pos=Point.getMean(this.pos, can.pos);
		count++;
		return true;
	}
	
	public boolean equals(Can can){
		//System.out.println(this.pos+", "+can.pos);
		return 	can.pos.getX()-Main.candiam <= this.pos.getX() && 
						can.pos.getX()+Main.candiam >= this.pos.getX() && 
						can.pos.getY()-Main.candiam <= this.pos.getY() && 
						can.pos.getY()+Main.candiam >= this.pos.getY();
	}
	
	public String toString(){
		return count+":"+pol+"\n"+pos;
	}
	
	public Can clone(){
		return new Can(this);
	}
}
