//Mrigank, 06-11-22
//5 bar parallel linkage inverse kinematics
//refer diagram for interpretation of symbols
/*Arm reference
    ^
b  / \  c
a  \_/  d
 φ2 e φ1 


   C ^
   b/ \c
ML /   \ MR
   \   /
   a\_/d
  Ol e Or
//Ol is the origin.
//diagram and math in NB.
*/
const double pi=3.14159265358979323846;
const double radtodeg=180/pi;
#include <Servo.h>
#include <math.h>
/*
const double a=5;//in cm 
const double d=5;//in cm
*/
const double a=4;//in cm 
const double d=4;//in cm
const double b=6;//in cm
const double c=6;//in cm
const double e=1;//in cm
/*
double theta2,theta1,
       delta2,delta1,
       gamma2,gamma1;
double x,y;
*/
//Servos
Servo ServoL,ServoR;


bool acosrangecheck(double num){
  return (num>=-1)&&(num<=1);
}
class Fivebar{
  public:
  double a,d,b,c,e;//in cm
  double x,y;//in cm
  double phi1,phi2;//in degrees
  int theta1,theta2;
  double range;
  void parameters(double a,double b, double c,double d,double e){
    this->a=a;
    this->b=b;
    this->c=c;
    this->d=d;
    this->e=e;
    range=max(a+b,c+d);
  }
  
  void inversekinematics(double x,double y){
      //here phi1 is CCW wrt. +ve x-axis while phi2 is CW wrt. -ve x-axis.
      this->x=x;
      this->y=y;
      //for phi2
      double aux1,aux2,sqr;
      sqr=sqrt((x*x)+(y*y));
      aux1=-x/sqr;
      aux2=(x*x+y*y-b*b+a*a)/(2*a*sqr);
      if (!acosrangecheck(aux2)){
        /*cout<<"Out of range phi2"<<endl;*/
        return;
    }
      aux1=acos(aux1);
      aux2=acos(aux2);
      phi2=aux1-aux2;
  
      //for phi1
      sqr=(y*y)+((x-e)*(x-e));
      aux1=(x-e)/sqrt(sqr);
      aux2=(sqr-c*c+d*d)/(2*d*sqrt(sqr));;
      if (!acosrangecheck(aux2)){
        /*cout<<"Out of range phi1"<<endl;*/
        return;
    }
      aux1=acos(aux1);
      aux2=acos(aux2);
      phi1=aux1-aux2;
  
      phi1*=radtodeg;
      theta1=phi1+90;
      phi2*=radtodeg;
      theta2=90-phi2;
  }
  //private:
  void forwardkinematicsphi(double phi1,double phi2){//in degrees
      //here phi1 and phi2 are both CCW wrt. positive x axis.
      //Position of Ml from Ol
      phi1*=pi/180;//deg to rad
      phi2*=pi/180;//deg to rad
      double xL=a*cos(phi2);
      double yL=a*sin(phi2);
      double xR=e+(d*cos(phi1));
      double yR=d*sin(phi1);
      double v1=(yL-yR)/(xR-xL);
      double v2=(yR*yR-yL*yL+b*b-c*c+xR*xR-xL*xL)/(2*(xR-xL));
      double A=1+v1*v1;
      double B=2*(v1*v2-v1*xL-yL);
      double C=(v2-xL)*(v2-xL)+yL*yL-b*b;
      y=(-B+sqrt(B*B-4*A*C))/(2*A);
      x=v1*y+v2;
  }

  public:
  void forwardkinematics(double theta1,double theta2){//in degrees
    forwardkinematicsphi(theta1-90,theta2+90);
  }
};


class Robot{
  private:
  //these will be later used for calculation but not relevant to the outside world.
  int i,N;//no. of iterations
  double change;
  double m;//slope
  double c;//constant
  public:
  Servo& ServoL;
  Servo& ServoR;
  Fivebar& FB1;
  double dl;//resolution in cm
  Robot(Servo& L,Servo& R,Fivebar& FB1,double dl=0.05):ServoL(L),ServoR(R),FB1(FB1),dl(dl){}
  void plotter(double x1,double y1,double x2,double y2){  
    if ((x1==x2)&&(y1==y2))return;
    else if (x1==x2){//m=inf
      N=abs(y2-y1)/dl;
      change=dl;
      if(y2<y1){change*=-1;}
      for(i=0;i<N;i++,y1+=change){
        FB1.inversekinematics(x1,y1);
        ServoL.write(FB1.theta2);
        ServoR.write(FB1.theta1);
        delay(10);//ms
      }
    }
    else if (y1==y2){//m=0
      N=abs(x2-x1)/dl;
      change=dl;
      if(x2<x1){change*=-1;}
      for(i=0;i<N;i++,x1+=change){
        FB1.inversekinematics(x1,y1);
        ServoL.write(FB1.theta2);
        ServoR.write(FB1.theta1);
        delay(10);//ms
      }
    }
    else{//y1#y2, x1#x2
      N=abs(x2-x1)/dl;
      m=(y2-y1)/(x1-x2);
      c=(y2-m*x2);
      change=dl;
      if(x2<x1){change*=-1;}
      for(i=0;i<N;i++,x1+=change){
        y1=(x1*m)+c;
        FB1.inversekinematics(x1,y1);
        ServoL.write(FB1.theta2);
        ServoR.write(FB1.theta1);
        delay(15);//ms
      }
      
    }
  
  }

};

void align(Servo& servo){
    int pos=servo.read();
    if (pos>90){
      for (; pos > 90; pos -= 1) { 
        servo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
    else if(pos<90){
      for (; pos < 90; pos += 1) { 
        servo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
}

//object constructions
Fivebar FB1;
Robot Arm(ServoL,ServoR,FB1,0.04);


//for demonstration
//curve to be plotted
//for M
/*
double curve_x[12]={-1,-1,0,0,0,1.5,3,3,3,4,4,-1};//in cm
double curve_y[12]={-1,0,0,1.5,3,1.5,3,1.5,0,0,-1,-1};//in cm*/
/*SQUARE
double curve_x[5]={0,0,4,4,0};
double curve_y[5]={0,4,4,0,0};
*/

//P
/*
double curve_x[4]={-2,0.5,3,-2};
double curve_y[4]={3,7,3,3};
double A=1;
double xo=0;//-2;
double yo=0;//3;
int i,n=4;*/

//line y
/*
double curve_x[3]={0.5,0.5,0.5};
double curve_y[3]={8,5,8};
double A=1;
double xo=0;//-2;
double yo=0;//3;
int i,n=4;
*/
/*
double xo=-1;//in cm
double A=1;
double yo=A*3;//starting point
int i,n=12;*/


double x_now;
double y_now;
 



void setup() {
  // put your setup code here, to run once:
  ServoL.attach(10);
  ServoR.attach(9);
  FB1.parameters(a,b,c,d,e);
  align(ServoL);
  align(ServoR);/*
  x_now=curve_x[0];
  y_now=curve_y[0];*/
  delay(3000);
}

void loop(){//for M
  /*
  for(i=0;i<n;i++){
    Arm.plotter(x_now*A+xo,y_now*A+yo,curve_x[i]*A+xo,curve_y[i]*A+yo);
    delay(50); 
    x_now=curve_x[i];
    y_now=curve_y[i];
  }*/
  
  //for horizonatal line
  Arm.plotter(-2.5,3,3.5,3);
  Arm.plotter(3.5,3,0.5,9);
  Arm.plotter(0.5,9,-2.5,3);
  
}
