/***********************************************************************************
*
* MotorEncoder.h
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
* Clase para representar el conjunto motor+encoder. Utiliza la libreria Encoder
* http://www.pjrc.com/teensy/td_libs_Encoder.html
* LICENSED UNDER GNU v3
************************************************************************************/

#include <Encoder.h>

class MotorEncoder{
    private:
        uint8_t IN1, IN2, ENABLE;
        Encoder* encoder;
    public:
        MotorEncoder(uint8_t IN1PIN, uint8_t IN2PIN, uint8_t ENABLEPIN,
            uint8_t ENCPIN1, uint8_t ENCPIN2){
                encoder = new Encoder(ENCPIN1, ENCPIN2);
                IN1 = IN1PIN;
                IN2 = IN2PIN;
                ENABLE = ENABLEPIN;
                pinMode (ENABLE, OUTPUT);
                pinMode (IN1, OUTPUT);
                pinMode (IN2, OUTPUT);
                digitalWrite (IN1, HIGH);
                digitalWrite (IN2, LOW);
                analogWrite (ENABLE, 0);
        }
        int32_t read();
        void write(int32_t position);
        void reset();
        void setSpeed(uint8_t speed);
        void reverse();

};

int32_t MotorEncoder::read(void){
    return encoder->read();
}

void MotorEncoder::write(int32_t position){
    encoder->write(position);
}

void MotorEncoder::reset(void){
    encoder->write(0);
}

void MotorEncoder::setSpeed(uint8_t speed){
    analogWrite(ENABLE, speed);
}

void MotorEncoder::reverse(void){
    digitalWrite (IN1, HIGH-digitalRead(IN1));
    digitalWrite (IN2, HIGH-digitalRead(IN2));
}
