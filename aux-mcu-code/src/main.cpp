#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <OLED.h>
#include <PID_v1.h>
#include <pins.h>

// Clearing confusion in dirction
#define FORWARD true;
#define BACKWARD false;

typedef struct Data {
  // Setpoint or currentpoint
  unsigned int point;
  // Brake
  bool brake;
  // Direction, true forward and false backward
  bool direction;
  // Error
  bool error;
} Data;

OLED OLED_screen;
/**
* Rules for messages below:
* 4MSB is for operands, 12 lower bits are for setpoints
* 4MSB are arranged as below: Brake, Direction, Error, Empty
*/
volatile unsigned int message_A_out;
volatile unsigned int message_A_in;
volatile unsigned int message_B_out;
volatile unsigned int message_B_in;
volatile bool brk;

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}

Data create_data(unsigned int point, bool brake, bool direction, bool error)
{
  Data current_data;
  current_data.point = point;
  current_data.brake = brake;
  current_data.direction = direction;
  current_data.error = error;

  return current_data;
}

Data decode_data(unsigned int message)
{
  unsigned int point = message & 0xFFF; // Take 12 LSB
  bool brake = message >> 15; // Take MSB
  bool direction = (message >> 14) & 1; // Take 2nd MSB
  bool error = (message >> 13) & 1; // Take 3rd MSB
  Data current_data = create_data(point, brake, direction, error);
  return current_data;
}

unsigned int encode_data(Data current_data)
{
  // Set the point
  unsigned int message = current_data.point & 0xFFF;

  // Set the brake
  if (current_data.brake)
  {
    message |= (1 << 15);
  }

  // Set the direction
  if (current_data.direction)
  {
    message |= (1 << 14);
  }

  // Set the error
  if (current_data.error)
  {
    message |= (1 << 13);
  }

  return message;
}
