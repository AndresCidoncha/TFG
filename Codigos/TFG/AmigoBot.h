/* AmigoBot class, for easy interaction with an Arduino differential robot.
By: Andrés Cidoncha Carballo <andrescidonchacarballo@gmail.com>
LICENSED UNDER GNU v3 */

#include "MotorEncoder.h"

/* PINES */
#define ENC1_P1   0
#define ENC1_P2   1
#define ENC2_P1   2
#define ENC2_P2   3
#define IN4       6
#define IN3       7
#define IN2       8
#define IN1       9
#define ENB       10
#define ENA       11
#define ON_LED    13
#define BUSY_LED  A0
#define ERROR_LED   A1
#define SW2o      A2

class AmigoBot{
public:
    leftMotor = MotorEncoder(IN1, IN2, ENA, ENC1_P1, ENC1_P2);  //M1
    rightMotor = MotorEncoder(IN3, IN4, ENB, ENC2_P1, ENC2_P2); //M2
    AmigoBot(){
        pinMode (SW2o, INPUT);
        pinMode (ON_LED, OUTPUT);
        pinMode (BUSY_LED, OUTPUT);
        pinMode (RED_LED, OUTPUT);
        digitalWrite (ON_LED, LOW);
        digitalWrite (BUSY_LED, LOW);
        digitalWrite (ERROR_LED, HIGH);
    }
    /* LED */
    void ready();
    void busy();
    void error();
    /* MOTORS */
    void resetEncoders();
    void setSpeeds(uint8_t leftSpeed, uint8_t rightSpeed);
    void stopMotors();
}

void AmigoBot::ready(void){
    digitalWrite (ON_LED, HIGH);
}

void AmigoBot::busy(void){
    digitalWrite (BUSY_LED, HIGH);
}

void AmigoBot::error(void){
    digitalWrite (ON_LED, LOW);
    digitalWrite (BUSY_LED, LOW);
    digitalWrite (ERROR_LED, HIGH);
}

void AmigoBot::resetEncoders(void){
    leftMotor.reset();
    rightMotor.reset();
}

void AmigoBot::setSpeeds(uint8_t leftSpeed, uint8_t rightSpeed){
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);
}

void AmigoBot::stopMotors(void){
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
}
