
  
  PVector position;
  PVector velocity;
  float r;
  PVector e;
  PVector f_total;
  
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
      if(PVector.dist(this.position,b.position) == (this.r + b.r) && this.id != b.id){
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
  }
   void updateForce(){
     this.f_total = new PVector(0,0);
     this.calculatedesiredDirectionForce();
     boolean debug = true;
     //if(debug){
       calculateRepulsionForceWall(Y_SUP, Y_SUP1);
       calculateRepulsionForceWall(Y_INF, Y_INF1);
       calculateCorporalForceWall(Y_SUP, Y_SUP1);
       calculateCorporalForceWall(Y_INF, Y_INF1);
       calculateFrictionForceWall(Y_SUP, Y_SUP1);
       calculateFrictionForceWall(Y_INF, Y_INF1);
     //}
     
     
     ArrayList<Individuo> vicinity = this.getVicinity();
     ArrayList<Individuo> contact = this.getContactGroup();
     for (Individuo i: vicinity){
       this.calculateRepulsionForce(i);
     }
     for (Individuo i: contact){
       this.calculateCorporalForce(i);
       this.calculateFrictionForce(i);
     }
     
     println("F: ",this.f_total);
     
  }
  
  
  
  //Fuerzas
 void calculatedesiredDirectionForce(){
   if(this.position.y-this.r > Y_SUP.y && this.position.y+this.r < Y_INF.y   ){
     this.e = new PVector(1,0);
   }
   else{
     float angle = 0;
     if(this.position.y-this.r <= Y_SUP.y){
       angle = PVector.angleBetween(Y_SUP,this.position);
     }
     if(this.position.y+this.r >= Y_INF.y){
       angle = -PVector.angleBetween(Y_INF, this.position);
     }
     this.e = new PVector(cos(angle), sin(angle));
   }
   this.f_total.add(PVector.div(PVector.sub(PVector.mult(this.e, v_0), this.velocity), T_i));
 }
 
 void calculateRepulsionForce(Individuo i){
   //Distancia hacia el individuo
   float distance = PVector.dist(this.position, i.position);
   PVector nij = PVector.div(PVector.sub(this.position, i.position),distance);
   PVector f_r =  PVector.mult(nij, A_i*exp( -(distance-(this.r+i.r))/B_i  ) );
   this.f_total.add(f_r);
 }
 
 void calculateCorporalForce(Individuo i){
   float distance = PVector.dist(this.position, i.position);
   PVector nij = PVector.div(PVector.sub(this.position, i.position),distance);
   PVector f = PVector.mult(nij, 2*k*(this.r+i.r - distance)) ;
   this.f_total.add(f);
 }
 
 void calculateFrictionForce(Individuo i ){
   float distance = PVector.dist(this.position, i.position);
   PVector nij = PVector.div(PVector.sub(this.position, i.position),distance);
   PVector tang = new PVector(-nij.y,nij.x); 
   float deltaV = PVector.dot( PVector.sub(i.velocity, this.velocity), tang  );
   PVector f = PVector.mult(tang, K*deltaV*( (this.r + i.r)-distance ));
   this.f_total.add(f);
 }
 
  void calculateRepulsionForceWall(PVector WallInit, PVector WallEnd){
   //Distancia hacia el individuo
   PVector perpendicularVector =getPerpendicularVector(WallInit, WallEnd);
   float distance = PVector.dist(perpendicularVector, this.position);
   PVector niw = PVector.div(PVector.sub(this.position, perpendicularVector),distance);
   PVector f_r =  PVector.mult(niw, A_i*exp( -(distance - this.r)/B_i  ) );
   //Distancia hasta la muralla
   
   this.f_total.add(f_r);
 }
 
  
  void calculateCorporalForceWall(PVector WallInit, PVector WallEnd){
    PVector perpendicularVector =getPerpendicularVector(WallInit,WallEnd);
    float distance = PVector.dist(this.position,perpendicularVector);
    if(distance == (this.r*2)){
      PVector niw = PVector.div(PVector.sub(this.position, perpendicularVector),distance);
      PVector f = PVector.mult(niw, 2*k*( this.r-distance ));
      //println("CONTACT FORCE: ",f, "DISTANCE: ",distance, "NIW: ",niw, "R:",r);
      this.f_total.add(f);
    }
    
  }
 
 void calculateFrictionForceWall(PVector WallInit, PVector WallEnd){
   PVector perpendicularVector =getPerpendicularVector(WallInit,WallEnd);
   float distance = PVector.dist(this.position,perpendicularVector);
   if(distance == (this.r*2)){
     PVector niw = PVector.div(PVector.sub(this.position, perpendicularVector),distance);
     PVector tang = new PVector(-niw.y,niw.x); 
     float deltaV = PVector.dot( this.velocity,tang);
     PVector f = PVector.mult(tang, deltaV*K*(r-distance));
     this.f_total.add(f);
   }
   
 }

 
 PVector getPerpendicularVector(PVector wallInit, PVector wallEnd){
   float k = ((wallEnd.y-wallInit.y) * (this.position.x-wallInit.x) - (wallEnd.x-wallInit.x) * (this.position.y-wallInit.y)) / (pow(wallEnd.y-wallInit.y,2) + pow(wallEnd.x-wallInit.x,2));
   PVector perpendicularPosition = new PVector(this.position.x - k * (wallEnd.y-wallInit.y),this.position.y + k * (wallEnd.x-wallInit.x));
   println("PERPENDICULAR: ",perpendicularPosition);
   return perpendicularPosition;
 }
 
  

}
