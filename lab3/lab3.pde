//Constantes de simulacion
public static int r_i = 10;
public static int R = 80 ;
public static float A_i = 25;
public static float B_i = 0.08;
public static int k = 75;
public static int K = 3000;
public static float v_0 = 5;
public static float T_i = 0.5;
public static float M = 1;
//Constantes del ambiente
public static int numParticles = 10;
public static ArrayList<Individuo> multitud;
public static PVector Y_SUP = new PVector( 600, 226);
public static PVector Y_SUP1 = new PVector(0,0);
public static PVector Y_INF = new PVector(600, 274);
public static PVector Y_INF1 = new PVector(0, 500);
public static int ID = 0;


PVector getDistance( PVector l1, PVector l2, PVector p){
  PVector result = new PVector(); 
  
  float dx = l2.x - l1.x; 
  float dy = l2.y - l1.y; 
  float d = sqrt( dx*dx + dy*dy ); 
  float ca = dx/d; // cosine
  float sa = dy/d; // sine 
  
  float mX = (-l1.x+p.x)*ca + (-l1.y+p.y)*sa; 
  
  if( mX <= 0 ){
    result.x = l1.x; 
    result.y = l1.y; 
  }
  else if( mX >= d ){
    result.x = l2.x; 
    result.y = l2.y; 
  }
  else{
    result.x = l1.x + mX*ca; 
    result.y = l1.y + mX*sa; 
  }
  
  dx = p.x - result.x; 
  dy = p.y - result.y; 
  result.z = sqrt( dx*dx + dy*dy ); 
  
  return result;   
}




void addIndividuo(PVector position){
   multitud.add(new Individuo(position));
}

void setInitialMultitud(){ //<>//
  float DIAMETER = r_i*2;
  PVector numParticles = new PVector(10,8);
  PVector initialPosition = new PVector(2*r_i,250 );
  PVector sizeBox = PVector.mult(numParticles,r_i*2);
  PVector origin = new PVector(initialPosition.x, initialPosition.y-sizeBox.y/2);
  for (int y = 0; y<numParticles.y; y++){
    for(int x=0; x<numParticles.x;x++){
      PVector position = new PVector(origin.x + DIAMETER*(float)x+DIAMETER/2, origin.y + DIAMETER*(float)y+DIAMETER/2);
      addIndividuo(position.copy());
      origin.add(5,0);
    }
    origin.x = initialPosition.x;
    origin.add(0,5);
    
  }
}

void drawLines(){
  stroke(255);
  strokeWeight(4);
  line(0,0,600,226);
  stroke(255);
  line(600,274,0,500);

}
//Dibujo del circulo que representa a un individuo
void render(Individuo individuo) {

  fill(255,0,0);
  //stroke(0);
  //pushMatrix();
  stroke(0);
  strokeWeight(0);
  ellipse(individuo.position.x, individuo.position.y, individuo.r,individuo.r);        

}

//Actualizacion de fuerzas
void updateParticles( ){
  deleteIndividuos();
  for (Individuo a: multitud) {
    render(a);
  }
  for( Individuo a: multitud){
    a.updateForce();
  }
  for (Individuo a: multitud) {
    a.updateVelocity();
    a.updatePosition();
  }
}


void deleteIndividuos (){
  int index = 0;
  ArrayList<Individuo> multitudAux = (ArrayList<Individuo>)multitud.clone();
  for (Individuo i : multitud){
    if(i.position.x > 600 ){
      multitudAux.remove(index);
    }else{
      index ++;
    }
  }
  multitud = multitudAux;
}

void setup() {
  size(750, 500);
  background(50);
  frameRate(30);
  ellipseMode(RADIUS);
  multitud = new ArrayList<Individuo>();
  setInitialMultitud();
}

void draw() {
  
    pushMatrix();
  background(50);
    drawLines();

  updateParticles();
    popMatrix();

}
// Distancia a la muralla...
//PVector tangent = new PVector(nextV.y - currentV.y, currentV.x - nextV.x);
//tangent.normalize();
