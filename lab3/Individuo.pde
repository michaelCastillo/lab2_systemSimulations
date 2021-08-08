  PVector position;
  PVector velocity;
  float r;
  PVector e;
  PVector f_total;
  
  float fr_limit = 30;
  float fcorp_limit = 30;
  float ffric_limit = 10;
  float fr_limit_wall = 10;
  float fcorp_limit_wall = 50;
  float ffric_limit_wall = 5;
  
  
  public class Individuo {
    int id;
    PVector position;
     PVector velocity;
     PVector f_total;
     PVector e;
     float r;
    
  Individuo(PVector position){
     this.position = position;
     this.id = ID;
     this.velocity = new PVector(0,0);
     this.f_total = new PVector(0,0);
     this.e = new PVector(0,0);
     this.r = r_i;
     ID++;
  }
  ArrayList<Individuo> getVicinity(){
    ArrayList<Individuo> vicinity = new ArrayList<Individuo>();
    for(Individuo b: multitud){
      if(PVector.dist(this.position,b.position) < R  && this.id != b.id){
        vicinity.add(b);
      }
    }
    return vicinity;
  }
  
  ArrayList<Individuo> getContactGroup(){
    ArrayList<Individuo> contact = new ArrayList<Individuo>();
    for(Individuo b: multitud){
      if(PVector.dist(this.position,b.position) < (this.r + b.r) && this.id != b.id){
        contact.add(b);
      }
    }
    return contact;
  }
  
  void updatePosition(){
    PVector newPosition = PVector.mult(this.velocity, T_i); 
    this.position.add(newPosition);                                                                                                               
  }
  
  
  void updateVelocity(){
    
    this.velocity.add(PVector.mult(this.f_total,T_i));
    this.velocity.limit(v_0);
  }
   void updateForce(){
    
     this.f_total.set(0,0);
     this.calculatedesiredDirectionForce();
     calculateRepulsionForceWall(Y_SUP, Y_SUP1);
     calculateRepulsionForceWall(Y_INF, Y_INF1);
     calculateCorporalForceWall(Y_SUP, Y_SUP1);
     calculateCorporalForceWall(Y_INF, Y_INF1);
     calculateFrictionForceWall(Y_SUP, Y_SUP1);
     calculateFrictionForceWall(Y_INF, Y_INF1);
   
     
     
     ArrayList<Individuo> vicinity = this.getVicinity();
     ArrayList<Individuo> contact = this.getContactGroup();
     for (Individuo i: vicinity){
       this.calculateRepulsionForce(i);
     }
     for (Individuo i: contact){
       this.calculateCorporalForce(i);
       this.calculateFrictionForce(i);
     }
     
     
     //println("F: ",this.f_total);

     
     
     
  }
  
  
  
  //Fuerzas
 void calculatedesiredDirectionForce(){
   PVector supDesiredPosition = new PVector(600, 239);
   PVector infDesiredPosition = new PVector(600, 261);
   if(this.position.y-this.r >= supDesiredPosition.y && this.position.y+this.r <= infDesiredPosition.y   ){
     this.e = new PVector(1,0);
   }
   else{
     if(this.position.y-this.r <= supDesiredPosition.y){
         this.e =  PVector.sub(supDesiredPosition,this.position).normalize();
     }
     if(this.position.y+this.r >= infDesiredPosition.y){
       this.e = PVector.sub(infDesiredPosition,this.position).normalize();
     }
   }
   this.f_total.add(PVector.div(PVector.sub(PVector.mult(this.e, v_0), this.velocity), T_i));
 }
 
 void calculateRepulsionForce(Individuo i){
   //Distancia hacia el individuo
   float distance = PVector.dist(this.position, i.position);
   if(distance <=14){distance = 14;}
   PVector nij = PVector.div(PVector.sub(this.position, i.position),distance).normalize();
   PVector f_r =  PVector.mult(nij, A_i*exp((this.r+i.r - distance)/B_i));
   f_r.limit(fr_limit);
   this.f_total.add(f_r);
   
 }
 
 void calculateCorporalForce(Individuo i){
   float distance = PVector.dist(this.position, i.position);
   if(distance <=14){distance = 14;}
   PVector nij = PVector.div(PVector.sub(this.position, i.position),distance).normalize();
   PVector f = PVector.mult(nij, 2*k*(this.r+i.r - distance));
   f.limit(fcorp_limit);
   this.f_total.add(f);
 
 
 }
 
 void calculateFrictionForce(Individuo i ){
   float distance = PVector.dist(this.position, i.position);
   if(distance <=14){distance = 14;}

   PVector nij = PVector.div(PVector.sub(this.position, i.position),distance).normalize();
   PVector tang = new PVector(-nij.y,nij.x); 
   float deltaV = PVector.dot( PVector.sub(i.velocity, this.velocity), tang  );
   PVector f = PVector.mult(tang, K*deltaV*( (this.r + i.r)-distance ));
   f.limit(ffric_limit);
   this.f_total.add(f);
 }
 
  void calculateRepulsionForceWall(PVector WallInit, PVector WallEnd){
   //Distancia hacia el individuo
   PVector perpendicularVector =point_perpendicular(WallInit, WallEnd);
   float distance = PVector.dist(perpendicularVector, this.position);
      if(distance <=16){distance = 16;}

   PVector niw = PVector.div(PVector.sub(this.position, perpendicularVector),distance).normalize();
   
   PVector f_r =  PVector.mult(niw, A_i*exp( (this.r*2 - distance)/B_i ));
   f_r.limit(fr_limit_wall);
   
   //Distancia hasta la muralla
   this.f_total.add(f_r);
 }
 
  
  void calculateCorporalForceWall(PVector WallInit, PVector WallEnd){
    PVector perpendicularVector =point_perpendicular(WallInit,WallEnd);
    float distance = PVector.dist(this.position,perpendicularVector);
    if(distance <= 2*this.r){
      PVector niw = PVector.div(PVector.sub(this.position, perpendicularVector),distance).normalize();
      PVector f = PVector.mult(niw, 2*k*( this.r-distance ));
      //println("CONTACT FORCE: ",f, "DISTANCE: ",distance, "NIW: ",niw, "R:",r);
      f.limit(fcorp_limit_wall);
      this.f_total.add(f);
    }
    
  }
 
 void calculateFrictionForceWall(PVector WallInit, PVector WallEnd){
   PVector perpendicularVector =point_perpendicular(WallInit,WallEnd);
   float distance = PVector.dist(this.position,perpendicularVector);
   if(distance <= 2*this.r){
     PVector niw = PVector.div(PVector.sub(this.position, perpendicularVector),distance).normalize();
     PVector tang = new PVector(-niw.y,niw.x); 
     float deltaV = PVector.dot( this.velocity,tang);
     PVector f = PVector.mult(tang, deltaV*K*(r-distance));
     f.limit(ffric_limit_wall);
     this.f_total.add(f);
   }
   
 }

 PVector point_perpendicular(PVector p1, PVector p2) {
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;
  double mag = Math.sqrt(dx*dx + dy*dy);
  // Vector unitario
  dx = dx/mag;
  dy = dy/mag;

  // Obtiene el punto
  double lamda = (dx * (this.position.x - p1.x)) + (dy * (this.position.y - p1.y));
  float x4 = (float) ((dx * lamda) + p1.x);
  float y4 = (float) ((dy * lamda) + p1.y);
  return new PVector(x4, y4);
}
 
 
 PVector getPerpendicularVector(PVector wallInit, PVector wallEnd){
   float k = ((wallEnd.y-wallInit.y) * (this.position.x-wallInit.x) - (wallEnd.x-wallInit.x) * (this.position.y-wallInit.y)) / (pow(wallEnd.y-wallInit.y,2) + pow(wallEnd.x-wallInit.x,2));
   PVector perpendicularPosition = new PVector(this.position.x - k * (wallEnd.y-wallInit.y),this.position.y + k * (wallEnd.x-wallInit.x));
   return perpendicularPosition;
 }
 
  

}
