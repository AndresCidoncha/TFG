/***********************************************************************************
*
* AmigoBot.h
*
***********************************************************************************
*
* AUTORES
* Andrés Cidoncha Carballo
*
* FECHA
* 25/06/2016
*
* DESCRIPCION
* Clase para representar la electrónica del AmigoBot de cara a simplificar la
* interaccion en el sketch principal
************************************************************************************/

#include "MotorEncoder.h"

/* PINES */
#define ENC1_P1   18
#define ENC1_P2   19
#define ENC2_P1   20
#define ENC2_P2   21
#define M1_IN1    2
#define M1_IN2    3
#define M1_ENA    4
#define M2_IN1    5
#define M2_IN2    6
#define M2_ENA    7
#define SW2o      10
#define BUSY_LED  11
#define ERROR_LED 12
#define ON_LED    13

class AmigoBot{
public:
    MotorEncoder* leftMotor = new MotorEncoder(M1_IN1, M1_IN2, M2_ENA, ENC1_P1, ENC1_P2);  //M1
    MotorEncoder* rightMotor = new MotorEncoder(M2_IN1, M2_IN2, M2_ENA, ENC2_P1, ENC2_P2); //M2
    AmigoBot(){
        pinMode (SW2o, INPUT);
        pinMode (ON_LED, OUTPUT);
        pinMode (BUSY_LED, OUTPUT);
        pinMode (ERROR_LED, OUTPUT);
        digitalWrite (ON_LED, LOW);
        digitalWrite (BUSY_LED, HIGH);
        digitalWrite (ERROR_LED, LOW);
    }
    /* LED */
    void ready();
    void busy();
    void error();
    /* MOTORS */
    void resetEncoders();
    void setSpeeds(uint8_t leftSpeed, uint8_t rightSpeed);
    void stopMotors();
};

void AmigoBot::ready(void){
    digitalWrite (BUSY_LED, HIGH);
    digitalWrite (ON_LED, HIGH);
}

void AmigoBot::busy(void){
    digitalWrite (ON_LED, HIGH);
    digitalWrite (BUSY_LED, LOW);
}

void AmigoBot::error(void){
    digitalWrite (ON_LED, LOW);
    digitalWrite (BUSY_LED, LOW);
    digitalWrite (ERROR_LED, HIGH);
}

void AmigoBot::resetEncoders(void){
    leftMotor->reset();
    rightMotor->reset();
}

void AmigoBot::setSpeeds(uint8_t leftSpeed, uint8_t rightSpeed){
    leftMotor->setSpeed(leftSpeed);
    rightMotor->setSpeed(rightSpeed);
}

void AmigoBot::stopMotors(void){
    leftMotor->setSpeed(0);
    rightMotor->setSpeed(0);
}
