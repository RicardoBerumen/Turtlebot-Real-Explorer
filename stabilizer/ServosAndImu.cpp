/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <ESP32Servo.h>
#include <sensor_msgs/msg/imu.h>  
#include <geometry_msgs/msg/vector3.h>
#include <MatrixMath.h>



//parametros de filtro pasabajas
float fc = 5.0;  // Frecuencia de corte (Hz)
float dt = 0.01;  // Periodo de muestreo (s)
float tau = 1.0 / (2.0 * PI * fc);
float beta = dt / (dt + tau);
  float accel_x {};
  float accel_y {};
  float accel_z {};

  float accel_xFiltered {};
  float accel_yFiltered{};
  float accel_zFiltered {};




rcl_subscription_t imu_subscriber;
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_data_received;
//mensaje de prueba
geometry_msgs__msg__Vector3 imu_data_to_publish;
////
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
TaskHandle_t Task1;
TaskHandle_t Task2;

// LED pins
const int led1 = 2;
const int led2 = 4;
//Angles calculated from IMU topic and used to move servos
float angleDegrees1{};
float angleDegrees2{};
//servos declaration
Servo servo1{};
Servo servo2{};
Servo servo3{};
const int pinServo1 = 18; // Pin GPIO al que está conectado el servo
const int pinServo2 = 19; // Pin GPIO al que está conectado el servo
const int pinServo3 = 23; // Pin GPIO al que está conectado el servo

//all the kinematic math and functions
float altura=8-0.9;
float ang1=0;
float ang2=10;
float rotacionZMotor1 = 120;
float rotacionZMotor2 = -120;

float Pp=4.5;
float Pb=2.5;
float L1=6.055;
float L2=5.125;

float con=3.1416/180;

double Pbx, Pbz;



double A, B , C , D;

double a, b , c,d;


double xb1, xb2, zb1, zb2;

double angulo1;

double anguloMotor1;
double anguloMotor2;
double anguloMotor3;

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
  {cos(120.0*con), -sin(120.0*con),0,0},
  {sin(120.0*con), cos(120.0*con), 0,0},
  {0,0,1,0},
  {0,0,0,1}

};
mtx_type z2[4][4] = {
  {cos(-120.0*con), -sin(-120.0*con),0,0},
  {sin(-120.0*con), cos(-120.0*con), 0,0},
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


double getAngle(int motor,int ang1,int ang2){
  if (ang2 >20){
    ang2=20;
  }
  if (ang2<-20){
  ang2=-20;
  }
  if (ang1 >20){
    ang1=20;
  }
  if (ang1<-20){
    ang1=-20;
  }
 
  mtx_type P_bB1RotadaEnZ[4][4];
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

  Pbx=Pp;
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
  angulo1=(asin(zb2/L1))*180/3.1416;
  Serial.println("Respuesta motor ");
  Serial.println(motor);
  Serial.println(angulo1);
  Serial.println(ang2);

  return angulo1;
}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


void imu_subscription_callback(const void * msgin)
{
  const sensor_msgs__msg__Imu * msg = (const sensor_msgs__msg__Imu *)msgin;
   accel_x = msg->linear_acceleration.x;
   accel_y = msg->linear_acceleration.y;
   accel_z = msg->linear_acceleration.z;

   accel_xFiltered = accel_xFiltered +beta*(accel_x- accel_xFiltered);
   accel_yFiltered = accel_yFiltered +beta*(accel_y- accel_yFiltered);
   accel_zFiltered = accel_zFiltered +beta*(accel_z- accel_zFiltered);


  
  

   float angleRadians1 =atan(accel_xFiltered/sqrt(accel_yFiltered*accel_yFiltered+accel_zFiltered*accel_zFiltered));
angleDegrees1= -angleRadians1*180/PI; 

  
  float angleRadians2 =atan(accel_yFiltered/sqrt(accel_xFiltered*accel_xFiltered+accel_zFiltered*accel_zFiltered));
  angleDegrees2= -angleRadians2*180/PI; 
  auto angleServo1{getAngle(1,angleDegrees2,angleDegrees1)};
  auto angleServo2{getAngle(2,angleDegrees2,angleDegrees1)};
  auto angleServo3{getAngle(3,angleDegrees2,angleDegrees1)};
   servo1.write(angleServo1);
   servo2.write(angleServo2);
   servo3.write(angleServo3+6);

  imu_data_to_publish.x = angleServo1;
  imu_data_to_publish.y = angleServo2;
  imu_data_to_publish.z = angleServo3;
  rcl_publish(&imu_publisher, &imu_data_to_publish, NULL);




}

void setup() {
  Serial.begin(115200); 
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  // initialization of servos
    servo1.attach(pinServo1);
  servo2.attach(pinServo2);
  servo3.attach(pinServo3);
   // servo1.write(0);
  //servo2.write(0);    // Mover el servo a 0 grados
  //servo3.write(0);
  delay(3000);
  ////////////////////////////////////
  //todas las cosas que estaban en el loop
    set_microros_transports();
    allocator = rcl_get_default_allocator();

    // Crear opciones de inicialización e incluir el dominio
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 5);  // Configura el dominio ROS a 5


    // Crear soporte y nodo con las opciones personalizadas
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    rclc_node_init_default(&node, "imu_data", "", &support);

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &imu_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu"));
     // // create publisher
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "/imu_print"));
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription( &executor,&imu_subscriber,&imu_data_received,&imu_subscription_callback,ON_NEW_DATA));



}


void loop() {
  
  

  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

}