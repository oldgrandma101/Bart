

//AVR Libraries to access registers
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// From MPU light library
#include "Wire.h"
#include <MPU6050_light.h>
#include "Servo.h"

//CONTROL CONSTANTS
#define TARGET_DISTANCE_RIGHT 45  // Target distance from right ultrasonic sensor to the wall was at 30
#define MIN_DISTANCE 30           // Target distance from front ultrasonic sensor to the wall
#define SLOW_DISTANCE 60          // Target distance from front ultrasonic sensor to the wall to slow lift fan(worked at90)



#define PB5 13  //trig US1
#define PD3 3   //echo US1
#define PB3 11  //trig US2
#define PD2 2   //echo US2

//define echo pin


const int trigPin1 = PB3;
const int echoPin1 = PD2;
float duration_us1, duration_cm1;

const int trigPin2 = PB5;
const int echoPin2 = PD3;
float duration_us2, duration_cm2;
float front_distance, right_distance;

float yaw;

float reference_yaw = 0.0;  //Will be used to keep our Hovercraft on track after turns

MPU6050 mpu(Wire);
unsigned long timer = 0;

Servo servo;           // Create a servo object;  //Servo-motor
int servoPin = 9;      // Define the pin where the servo is connected
int servo_angle = 90;  // Initialize the servo angle


//

void setup() {
  // From MPU light
  Serial.begin(9600);
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets();  // gyro and accelero
  Serial.println("Done!\n");

  // // //

  // Wire.setWireTimeout(3000, true);
  servo.attach(servoPin);  // Attach the servo to the specified pin

  // // //

  // Configs for other ranges
  Wire.write(0b00000000);  //Set the accelerometer range to ±2g
  // Wire.write(0b00010000); //Set the accelerometer range to ±4g
  // Wire.write(0b00100000); // Set the accelerometer range to ±8g
  // Wire.write(0b00110000); // Set the accelerometer range to ±16g

  // Wire.write(0b00000000);  //Set the gyroscope range to ±250 dps
  // Wire.write(0b00010000); // Set the gyroscope range to ±500 dps
   Wire.write(0b00100000); // Set the accelerometer range to ±1000 dps
  // Wire.write(0b00110000); // Set the accelerometer range to ±2000 dps

  //US sensor 1 connected to p6
  pinMode(trigPin1, OUTPUT);  //trig pin1 to output
  pinMode(echoPin1, INPUT);   //echo pin1 to input

  //US sensor 2 connected to p13
  pinMode(trigPin2, OUTPUT);  //trig pin1 to output
  pinMode(echoPin2, INPUT);   //echo pin1 to input

  // // //

  TCCR0A |= (1 << COM0A1);  //non-inverted pin operation
  TCCR0A |= (1 << COM0B1);  //non-inverted pin operation

  TCCR0A |= (1 << WGM00);  //PWM, Phase Correct

  TCCR0B |= ((1 << CS01) | (1 << CS00));  //Prescaler = 64. Start the timer

  TIMSK0 |= (1 << TOIE0);  //Enable overflow interupt

  // Enable global interrupts
  sei();

  DDRD |= ((1 << PD5));  //Set direction of pin5 to output OC0B
  DDRD |= ((1 << PD6));  //Set direction of pin6 to output OC0A
}  //end void setup()




//
void loop() {

  mpu.update();
  yaw = mpu.getAngleZ();
  
  //US SENSOR 1 AT P6
  // Clears the trigPin
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration_us1 = pulseIn(echoPin1, HIGH);
  // Calculating the distance
  duration_cm1 = 0.017 * duration_us1;

  //US SENSOR 2 AT P13
  // Clears the trigPin
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration_us2 = pulseIn(echoPin2, HIGH);
  // Calculating the distance
  duration_cm2 = 0.017 * duration_us2;

  front_distance = duration_cm1;
  right_distance = duration_cm2;


  //beginning of navigation logic



  if (isStraight() && front_distance > SLOW_DISTANCE && front_distance != 0)  //if we're within 10deg of reference_yaw we check the sensors
  {
    full_steam_ahead(); 
  } 
  else if (isStraight() && front_distance < SLOW_DISTANCE && front_distance > MIN_DISTANCE && front_distance != 0)  //if we're within 10deg of reference_yaw we check the sensors
  {
    slow_down();
  }
  else if (isStraight() && front_distance < SLOW_DISTANCE && front_distance < MIN_DISTANCE && front_distance != 0)  //if we're within 10deg of reference_yaw we check the sensors
  {
    stop();
    
    if (right_distance > TARGET_DISTANCE_RIGHT)  //turn logic, if right_distance has room to go right, turn right
    {
      
      turn_right();


      //after turn_right we want to get to the reference_yaw before checking for walls so that we don't turn unnecessarily
      
      mpu.update();
      yaw = mpu.getAngleZ();
      

      fix_yaw(); //make sure we're within 10deg of reference_yaw after a turn before checking sensors for walls



    } else  //if the distance on the right is smaller that the TARGET_DISTANCE_RIGHT turn left
    {
      
      //call left turn function
      turn_left();
      

      //after turn_right we want to get to the reference_yaw before checking for walls so that we don't turn too early
      
      mpu.update();
      yaw = mpu.getAngleZ();
      

      fix_yaw(); //make sure we're within 10deg of reference_yaw after a turn beofre checking the sensors for walls



    }
  } else fix_yaw();//if we get to this point we were not within 10deg of reference yaw so call fix_yaw() function


}  //end void loop

void turn_right() //Turns the hovercraft 90deg to the right when called
{

  //Turn the fans off
  OCR0A = 0;
  OCR0B = 0;
  
  delay(2000);  //wait 2 seconds before turning
  int servo_angle;

  //get new yaw values from IMU
  mpu.update();
  float current_yaw = mpu.getAngleZ(); 
  

  //a right turn is -90deg for this IMU
  float new_reference_yaw = reference_yaw - 90.0; 


  while (current_yaw > new_reference_yaw + 10)  //Turn right if Hovercraft is more than 10deg away from the new reference yaw
  {

    mpu.update();
    current_yaw = mpu.getAngleZ();  //update yaw of hovercraft as we turn
    
    if (current_yaw >= reference_yaw)  //in this case we're already rotated to the left so turn the fan all the way to the right
    {
      servo.write(170);
    } 
    else  //in this case we're rotated to the right already so we don't want to turn the fan too much to the right
    {
      servo_angle = map(current_yaw, (new_reference_yaw + 90), new_reference_yaw, 170, 90);  //if we're 90deg to the left of target_yaw, turn fan 80deg to the right, linearly change the fan angle the more we turn
      servo.write(servo_angle); 
    }

    //once target_yaw and the required servo_angle is calculated turn the fans on

    OCR0A = 225;  //Thrust fan
    OCR0B = 245;  //Lift fan

  }  // end while loop
  reference_yaw = new_reference_yaw;  //update the reference_yaw after the turn is complete
  

  //fans should turn off at this point
  OCR0A = 0;
  OCR0B = 0;
  delay(2000);


  return;  //return to void loop
}  //end turn_right()

void turn_left() //Turns the hovercraft 90deg to the left when called
{
  
  //Turn the fans off
  OCR0A = 0;
  OCR0B = 0;


  delay(2000);
  int servo_angle;

  //get new yaw values from IMU
  mpu.update();
  float current_yaw = mpu.getAngleZ();

  //A left turn is +90deg for this IMU
  float new_reference_yaw = reference_yaw + 90.0;


  while (current_yaw < new_reference_yaw - 10)  //Turn left if Hovercraft is more than 10deg away from the new reference yaw
  {

    mpu.update();
    current_yaw = mpu.getAngleZ();  //current yaw of hovercraft as we turn
    
    if (current_yaw <= reference_yaw)  //in this case we're already rotated to the right so turn the fan all the way to the left
    {
      servo.write(10);
    } 
    else  //in this case we're rotated to the left already so we don't want to turn the fan too much to the left
    {
      servo_angle = map(current_yaw, (new_reference_yaw - 90), new_reference_yaw, 10, 90);  //if we're 90deg to the right of target_yaw, turn fan 80deg to the left, linearly change the fan angle the more we turn
      servo.write(servo_angle);
    }

    //once target_yaw and the required servo_angle is calculated turn the fans on

    OCR0A = 225;  //Thrust 
    OCR0B = 245;  //Lift

  }  // End while loop

  //update reference_yaw after we complete a turn
  reference_yaw = new_reference_yaw;
  

  //fans should turn off at this point
  OCR0A = 0;
  OCR0B = 0;
  delay(2000);


  return;  //return to void loop
}

void full_steam_ahead() //Fastest speed 
{
  //Current yaw from IMU
  mpu.update();
  float current_yaw = mpu.getAngleZ();

  //Turn the thrust fan in the opposite direction of rotation to keep Hovercraft going straight
  servo_angle = map(current_yaw, (reference_yaw + 90), (reference_yaw - 90), 180, 0);  
  servo.write(servo_angle);

  OCR0A = 225;  //thrust
  OCR0B = 250;  //lift 
  return;       //return to void loop to reevaluate sensor data
}

void slow_down()  //Lower the fans to prepare to stop before a wall
{
  //Update yaw from IMU
  mpu.update();
  float current_yaw = mpu.getAngleZ();

  //Turn the thrust fan in the opposite direction of rotation to keep Hovercraft going straight
  servo_angle = map(current_yaw, (reference_yaw + 90), (reference_yaw - 90), 180, 0);  /
  servo.write(servo_angle);

  OCR0A = 200;  //thrust
  OCR0B = 240;  //lift
  return;       //return to void loop to reevaluate sensor data
}

void stop() //Stop the fans to stop in front of a wall
{
  
  mpu.update();
  float current_yaw = mpu.getAngleZ();  //this is the actual yaw rightnow

  //Turn the thrust fan in the opposite direction of rotation to keep Hovercraft going straight
  servo_angle = map(current_yaw, (reference_yaw + 90), (reference_yaw - 90), 180, 0); 
  servo.write(servo_angle);

  OCR0A = 0;  //thrust 
  OCR0B = 0;  //lift (worked at 240 twice)
  return;     //return to void loop to reevaluate sensor data
}

bool isStraight() //this function returns true if we're within 10deg of reference_yaw
{
  mpu.update();
  yaw = mpu.getAngleZ();
  
  return (yaw < reference_yaw + 10 && yaw > reference_yaw - 10);  
}


void fix_yaw() //this function straightens us out without looking at the sensors to avoid unneccessary turns
{
  while (true)  //this forces the program to enter the while loop
  {
    //Update yaw from IMU
    mpu.update();
    yaw = mpu.getAngleZ();
    //call slow_down() function that corrects hovercraft to within 10deg of reference_yaw 
    slow_down();          
    if (isStraight()) //if isStraight returns true we're within 10deg of refernce_yaw so break from while loop and return from fix_yaw() function
      break;
    
  }//end while

}//end fix_yaw
