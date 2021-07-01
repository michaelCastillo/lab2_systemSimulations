public static int curr_n_fluid = 0;
public static int SIZE_X = 800;
public static int SIZE_Y = 1500;
//Fluid
public static ArrayList<Particle> fluid;
public static float DIAMETER = 10;
public static PVector numParticles = new PVector(20,55);
public static PVector initialPosition = new PVector(400,100);
public static float R = 10;
//Symulation parameters
public static float M = 65;
public static PVector g = new PVector(0,-9.8*12000);
public static float delta_t = 8*pow(10,-4);
public static float density_initial = 100;
public static float K = 2000;
public static float mu = 250; //Viscosidad del aceite

class Particle{
  PVector position;
  int id;
  float density=density_initial;
  float pression = 0;
  PVector velocity;
  Particle(float x, float y) {
    this.id = curr_n_fluid;
    this.position = new PVector(x, y);
    this.velocity=new PVector(0,0);
  }
  
  void calculateDensity(ArrayList<Particle> vicinity){
    float acum=0;
    for(Particle b: vicinity){
      acum += M*densityKernel(PVector.sub(this.position, b.position).mag());
    }
    this.density = acum;
  }
  void calculateNewPression(){
    this.pression = K*(this.density - density_initial);
    //println("PRESSIOn: ",this.pression);
  }
  
}

//Kernels
float densityKernel(float r){
  return (float)315/(65*PI*pow(R,9))*pow(pow(R,2) - pow(r,2),3);
}
float pressureKernel(float r){
  return (float)-45/(PI*pow(R,6))*pow(R-r,2);
}
float viscosityKernel(float r){
  return (float)45/(PI*pow(R,6))*(R-r);
}
void calculateNewVelocity(Particle particle, PVector F_total){
  PVector F_totalpond = PVector.mult(F_total, delta_t/particle.density);
  particle.velocity.add(F_totalpond);//Suma a la velocidad de la particula
}
void calculateNewPosition(Particle particle){
  PVector velocityPond = PVector.mult(particle.velocity, delta_t);
  particle.position.add(velocityPond);
}

PVector getGravityForce(Particle particle){
  return PVector.mult(g,particle.density);
}
PVector getViscosityForce(Particle particle, ArrayList<Particle> vicinity){
  PVector acum = new PVector(0,0);
  for (Particle b: vicinity){
    float K = viscosityKernel(PVector.sub(particle.position,b.position).mag());
    PVector calculation =PVector.mult( PVector.div(  PVector.sub(b.velocity,particle.velocity), b.density )  ,M*K); 
    acum.add(calculation);
  }
  return PVector.mult(acum,mu);
}
PVector getPressionForce(Particle particle, ArrayList<Particle> vicinity){
  PVector acum = new PVector(0,0);
  for(Particle b: vicinity){
    PVector rij = new PVector(0,0);
    if(PVector.dist(b.position,particle.position) != 0){
      rij = PVector.sub(b.position, particle.position);
      rij = PVector.div( rij ,rij.mag());
    }
    float K = pressureKernel(PVector.sub(particle.position,b.position).mag());
    PVector calculation = PVector.mult(rij,K*M*(particle.pression + b.pression)/(2*b.density));
    acum.add(calculation);
  }
  return PVector.mult(acum,-1);
}


ArrayList<Particle> getVicinity(Particle a){
  ArrayList<Particle> vicinity = new ArrayList<Particle>();
  for(Particle b: fluid){
    if(PVector.dist(a.position,b.position) < R){
      vicinity.add(b);
    }
  }
  return vicinity;
}

void updatePressionsAndDensities(){
  for( Particle a: fluid){
    a.calculateDensity(getVicinity(a));
    a.calculateNewPression();
  }
}

//Actualizacion de fuerzas
void updateParticles( ){
  for( Particle a: fluid){
    borders(a);
    render(a);
    ArrayList<Particle> vicinity = getVicinity(a);
    a.calculateDensity(vicinity);
    PVector F_gravity = getGravityForce(a);
    PVector F_viscosity = getViscosityForce(a, vicinity);
    PVector F_pression = getPressionForce(a,vicinity);
    println("Gravity: ",F_gravity,"Viscosity:",F_viscosity,"Pression:",F_pression);
    PVector F_total = PVector.add(PVector.add(F_gravity,F_viscosity),F_pression);
    //PVector F_total = PVector.add(F_gravity,F_viscosity);
    calculateNewVelocity(a, F_total);
    calculateNewPosition(a);
  }
  updatePressionsAndDensities();
}

void borders(Particle a){
  float reduce = 0.25;
  if(a.position.y - DIAMETER/2 <= 0){
    a.velocity.y = -a.velocity.y*reduce;
    a.position.y = DIAMETER/2;
  }else if(a.position.y + DIAMETER/2 >= height){
    a.velocity.y = -a.velocity.y*reduce;
    a.position.y = height-DIAMETER/2;
  }
  if(a.position.x - DIAMETER/2 <= 0){
    a.velocity.x = -a.velocity.x*reduce;
    a.position.x = DIAMETER/2;
  }else if(a.position.x + DIAMETER/2 >= width){
    a.velocity.x = -a.velocity.x*reduce;
    a.position.x = width + DIAMETER/2;
  }
  
}

//Dibujo del circulo que representa a un individuo
void render(Particle particle) {
  fill(255,0,0);
  //stroke(0);
  pushMatrix();
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
      PVector position = new PVector(origin.x + DIAMETER*(float)x+DIAMETER/2 + random(-1,1), origin.y + DIAMETER*(float)y+DIAMETER/2);
      addParticle(position.x,position.y);
    }
  }
}

void setup() {
  size(800, 850);
  background(50);
  frameRate(15);
  ellipseMode(CENTER);
  fluid = new ArrayList<Particle>();
  setInitialParticles();
}

void mousePressed() {
  float block_size = 10*DIAMETER;
  float x_init = mouseX - block_size/2;
  float y_init = height - mouseY - block_size/2;
  print("MOUSE_X: ",mouseX,"  MOUSE_Y:",mouseY,"\n");
  if(x_init < 0)
    x_init = 0;
  else if(x_init > width)
    x_init = width - block_size - DIAMETER;
  if(y_init < 0)
    y_init = 0;
  else if(y_init > height)
    y_init = height - block_size- DIAMETER;
  
  for(int i = 0; i<10; i++){
    for(int j=0; j<10; j++){
      //println("x_init: ",x_init," y_init:",y_init);
      addParticle(x_init + DIAMETER*i,y_init + DIAMETER*j);
    }
  }
}

void draw() {
  scale(1,-1);
  translate(0,-height);
  background(50);
  updateParticles();
}
