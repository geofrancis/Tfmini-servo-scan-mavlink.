#include "mavlink/common/mavlink.h"        // Mavlink interface
#include "mavlink/common/mavlink_msg_obstacle_distance.h"
#include <Servo.h>
#include <TFMPlus.h>  

#define TFMINI_BAUDRATE 115200 
#define TFMINI_DATARATE 10.0f // ms
#define POTI_PIN PA0
#define EXTERNAL_BAUDRATE 1500000 


int potiValue = 0;
int FOV = 120; //multiple of res and even(res is 3 degree for the TF02-pro)
int lidarAngle = 0;
int messageAngle = 0;
int res = 3;
uint16_t distances[72];
int posi = 0;    // variable to store the servo position  

TFMPlus tfmP;      
Servo myservo;  // create servo object to control a servo
HardwareSerial Serial2(USART2);
char serial_buffer[15];

void setup() {
  

  Serial1.begin(EXTERNAL_BAUDRATE); // USB
  Serial2.begin(TFMINI_BAUDRATE); // TF mini
  tfmP.begin(&Serial2);
  myservo.attach(PA1);  // attaches the servo on pin PA1 to the servo object
  flushSerial2();
  memset(distances, UINT16_MAX, sizeof(distances)); 
  
}


int16_t tfDist = 0;    // Distance to object in centimeters
uint16_t pulses = 0;

void loop() {
  
  
  potiValue = analogRead(POTI_PIN);
  lidarAngle = map(potiValue, 185, 815, -90, 90);          // Adjust for the poti you use
  messageAngle = map(lidarAngle, -FOV/2, FOV/2, 0, FOV);
  
  if (lidarAngle <= -FOV/2) 
  {
    
    for (posi = 0; posi <= 180; posi += 1) { // goes from 0 degrees to 180 degrees in steps of 1 degree
    myservo.write(posi);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15 ms for the servo to reach the position
  }
  }
  if (lidarAngle >= FOV/2)
  {
    pulses = 0;
    
    for (posi = 180; posi >= 0; posi -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(posi);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15 ms for the servo to reach the position
  }
  }
  
  
  if(lidarAngle%res == 0){ // get a distance reading for each res (3 degree) step
    tfmP.getData(tfDist);
     
    send_pos();
  }



}

void send_pos(){

//MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 158;    

  uint64_t time_usec = 0; /*< Time since system boot*/
  uint8_t sensor_type = 0;
  distances[messageAngle/res] = tfDist-2.0f; //UINT16_MAX gets updated with actual distance values
  uint8_t increment = 3;
  uint16_t min_distance = 5; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 4000; /*< Maximum distance the sensor can measure in centimeters*/
  float increment_f = 0;
  float angle_offset = -FOV/2;
  uint8_t frame = 12;
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_obstacle_distance_pack(sysid,compid,&msg,time_usec,sensor_type,distances,increment,min_distance,max_distance,increment_f,angle_offset,frame);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  //delay(1);
  Serial1.write(buf, len);

}


// Flushes the INPUT serial buffer
void flushSerial2(){
  while(Serial2.available()){Serial2.read();}
}
