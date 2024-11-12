#include <MatrixMath.h>


float altura=0.09-0.009;
float ang1=0;
float ang2=-15;
float rotacionZMotor1 = 120;
float rotacionZMotor2 = -120;

float Pp=0.045;
float Pb=0.04922;
float L1=0.06055;
float L2=0.05126;

float con=3.1416/180;

double Pbx, Pbz;



double A, B , C , D;

double a, b , c,d;


double xb1, xb2, zb1, zb2;

double angulo1;

mtx_type Tz[4][4] = { 
  {1,0,0,0},
  {0,1,0,0},
  {0,0,1,altura},
  {0,0,0,1}
};

mtx_type x[4][4] = { 
  {1,0,0,0},
  {0,cos(ang1*con),-sin(ang1*con),0},
  {0,sin(ang1*con),cos(ang1*con),0},
  {0,0,0,1}
};

mtx_type y[4][4] = { 
  {cos(ang2*con),0,sin(ang2*con),0},
  {0,1,0,0},
  {-sin(ang2*con),0,cos(ang2*con),0},
  {0,0,0,1}
};

mtx_type z1[4][4] = {
  {cos(120.0), -sin(120.0),0,0},
  {sin(120.0), cos(120.0), 0,0},
  {0,0,1,0},
  {0,0,0,1}

};
mtx_type z2[4][4] = {
  {cos(-120.0), -sin(-120.0),0,0},
  {sin(-120.0), cos(-120.0), 0,0},
  {0,0,1,0},
  {0,0,0,1}

};






mtx_type P_pB1[4][1] = { 
  {Pp},
  {0},
  {0},
  {1}
};




mtx_type P_bp[4][4];
mtx_type P_bp2[4][4];
mtx_type P_bB1[4][1];


double getAngle(int motor){
  mtx_type P_bB1RotadaEnZ[4][4];

  ///
  Matrix.Multiply((mtx_type*)Tz,(mtx_type*)x,4,4,4,(mtx_type*)P_bp);
  Matrix.Multiply((mtx_type*)P_bp,(mtx_type*)y,4,4,4,(mtx_type*)P_bp2);

  
  switch(motor){
    case 1:
      Matrix.Multiply((mtx_type*)P_bp2,(mtx_type*)P_pB1,4,4,1,(mtx_type*)P_bB1);
      break;
    case 2:
      Matrix.Multiply((mtx_type*)P_bp2,(mtx_type*)z1,4,4,4,(mtx_type*)P_bB1RotadaEnZ);
      Matrix.Multiply((mtx_type*)P_bB1RotadaEnZ,(mtx_type*)P_pB1,4,4,1,(mtx_type*)P_bB1);
      break;
    case 3:
      Matrix.Multiply((mtx_type*)P_bp2,(mtx_type*)z2,4,4,4,(mtx_type*)P_bB1RotadaEnZ);
      Matrix.Multiply((mtx_type*)P_bB1RotadaEnZ,(mtx_type*)P_pB1,4,4,1,(mtx_type*)P_bB1);
      break;
    default:
      break;

  }

  //Matrix.Print((mtx_type*)P_bB1,4,1,"P_bB1");
  Pbx=(P_bB1[0][0]);
  Pbz=(P_bB1[0][2]);
  A=(Pbx*Pbx+Pbz*Pbz-L2*L2);
  B=(Pb*Pb-L1*L1);
  C=(Pbx-Pb)/Pbz;
  D=(A-B)/(2*Pbz);
  a=(1+C*C);
  b=2*(Pb+D*C);
  c=(Pb*Pb+D*D-L1*L1);
  xb1=(-b+sqrt(b*b-4*a*c))/(2*a);
  xb2=(-b-sqrt(b*b-4*a*c))/(2*a);
  zb1=sqrt(L1*L1-(xb1+Pb)*(xb1+Pb));
  zb2=sqrt(L1*L1-(xb2+Pb)*(xb2+Pb));
  angulo1=(asin(zb1/L1))*180/3.1416;
  Serial.println("Respuesta motor ");
  Serial.println(motor);
  Serial.println(angulo1);
  return angulo1;
}

void setup() {
      Serial.begin(115200); 



  double anguloMotor1= getAngle(1);
  double anguloMotor2= getAngle(2);
  double anguloMotor3= getAngle(3);


}

void loop() {
  

}
