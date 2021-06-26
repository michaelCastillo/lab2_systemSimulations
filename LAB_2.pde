public static int curr_n_fluid = 0;
public static int SIZE_X = 800;
public static int SIZE_Y = 600;
//Fluid
public static ArrayList<Particle> fluid;
public static float DIAMETER = 10;
public static PVector numParticles = new PVector(20,55);
public static PVector initialPosition = new PVector(400,300);
public static float R = 40;
//Symulation parameters
public static float m = 65;
public static PVector g = new PVector(0,-9.8*1200);
public static float delta_t = 8*pow(10,-4);
public static float density_initial = 1000;
public static float k = 2000;
// 



class Particle{
  PVector position;
  int id;
  float density=density_initial;
  PVector velocity;
  Particle(float x, float y) {
    this.id = curr_n_fluid;
    this.position = new PVector(x, y);
    this.velocity=new PVector(0,0);
  }
  void updatePosition() {
    //this.position.add(this.velocity);
    //this.velocity.add(this.acceleration);
  }
  void borders() {
  float x = this.position.x;
  float y = this.position.y;
  float radius = DIAMETER/2;
  boolean colision = false;
  PVector colisionPosition = new PVector(0,0);
    if (x-radius < 0){
      colisionPosition.x = x;
      this.position.x = 0+radius;
    }
    if (y-radius < 0){
      this.position.y = 0+radius;
      colisionPosition.y = y;
    }
    if (x+radius > width) {
    this.position.x = width-radius;}
    colisionPosition.x = width-radius;
    if (y+radius > height){
     this.position.y = height-radius;
     colisionPosition.x = height-radius;
    }
  }
  
  
  float K_density(float r_diff){
    return (315*pow((pow(R,2)-pow(r_diff,2)),3))/(65*PI*pow(R,9));
  }
  void updateDensity(ArrayList<Particle> neighborhood){
    float sum = 0;
    for(Particle b: neighborhood){
      PVector diff_r = this.position.sub(b.position);
      sum += m * K_density(diff_r.mag());
    }
    print("Density: ",this.density," \n"); 
    this.density = sum;
  }
  
  
  PVector getGravityF(){
    return g.mult(this.density);
  }
  float getPression(){
    return k*(this.density - density_initial);
  }
  float K_pressionForce(float r_diff){
    return -45*pow(R-r_diff,2)/(PI*pow(R,6));
  }
  PVector getPressionF(ArrayList<Particle> neighborhood){
    PVector sum = new PVector(0,0);
    for (Particle b: neighborhood){
      PVector r_diff = b.position.sub(this.position);
      PVector r_ij = r_diff.div(r_diff.mag());
      sum.add( r_ij.mult((-m)*K_pressionForce(r_diff.mag())*(this.getPression() + b.getPression())/(2*b.density)));
    }
    return sum;
  }
  
  float K_viscosity(float r_diff){
    return 45*(R-r_diff)/(PI*pow(R,6));
  }
  
  PVector getViscosity(ArrayList<Particle> neighborhood){
    PVector sum = new PVector(0,0);
    for(Particle b: neighborhood){
      PVector r_diff = b.position.sub(this.position);
      PVector v_diff = b.velocity.sub(this.velocity);
      sum.add(  v_diff.mult(m*K_viscosity(r_diff.mag())).div(this.density));
    }
    return sum;
  }
  
  
  void calculateNewVelocity(PVector F_total){
    this.velocity = this.velocity.add(F_total.mult(delta_t/this.density));
  }
  void calculateNewPosition(){
    this.position = this.position.add(this.velocity.mult(delta_t));
  }
  ArrayList<Particle> getNeighborhood(ArrayList<Particle> particles){//Cambiar por vicinity
    ArrayList<Particle> neighborhood = new ArrayList<Particle>();
    for(Particle b : particles){
      float distance = b.position.sub(this.position).mag();
      print("Distance: ",distance," ");
      if(distance < R){
        neighborhood.add(b);
      }
    }
    return neighborhood;
  }
}




//Actualizacion de fuerzas
void updateParticles(ArrayList<Particle> particles ){
  for( Particle a: particles){
    //a.borders();
    render(a);
    ArrayList<Particle> neighborhood = a.getNeighborhood(particles);
    a.updateDensity(neighborhood);//Se actualiza la densidad.
    PVector gravity = a.getGravityF();
    PVector pression = a.getPressionF(neighborhood);
    PVector viscosity = a.getViscosity(neighborhood);
    PVector F_total = gravity.add(pression).add(viscosity);
    a.calculateNewVelocity(F_total);
    a.calculateNewPosition();
    
  }
  
  
}


  //Dibujo del circulo que representa a un individuo
  void render(Particle particle) {
    fill(255,0,0);
    stroke(0);
    pushMatrix();
    ellipseMode(CENTER);
    ellipse(particle.position.x, particle.position.y, DIAMETER,DIAMETER);        
    popMatrix();
  }


void addParticle(float x, float y){
   fluid.add(new Particle(x,y));
}

void setInitialParticles(){
  PVector sizeBox = PVector.mult(numParticles,DIAMETER);
  PVector origin = new PVector(initialPosition.x - sizeBox.x/2, initialPosition.y);
  for (int x = 0; x<numParticles.x; x++){
    for(int y=0; y<numParticles.y;y++){
      PVector position = new PVector(origin.x + DIAMETER*(float)x+DIAMETER/2, origin.y + DIAMETER*(float)y+DIAMETER/2 );
      addParticle(position.x,position.y);
    }
  }
}

void setup() {
  size(800, 600);
  background(50);
  frameRate(1);
  fluid = new ArrayList<Particle>();
  setInitialParticles();
}

//void mousePressed() {
//  if(curr_n_fluid < N)
//    addParticle(mouseX,mouseY);
//  curr_n_fluid += 1;
//}

void draw() {
  scale(1,-1);
  translate(0,-height);
  background(50);
  updateParticles(fluid);
}
