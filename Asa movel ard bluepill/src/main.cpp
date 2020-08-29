#include <Arduino.h>
#include <Servo.h>
//#include "Wire.h"


Servo servo1;  // cria um objeto para controlar o servo
Servo servo2;  // cria um objeto para controlar o servo

#define manual_auto PA11
#define botao_asa PA12
#define pot1 PA0
#define pot2 PA1
//#define pot3 PB1

int max_pot1 = 0, min_pot1 = 1022;
int max_pot2 = 0, min_pot2 = 1022;
int pos_pot1, pos_pot2;

int max_servo1 = 0, min_servo1 = 180;
int max_servo2 = 0, min_servo2 = 180;
int pos_servo1, pos_servo2;

void config_limit();
void init_mpu();
void init_servo();

void setup() {
        
    //Wire.begin();
      
    Serial.begin(115200);
    
    pinMode(pot1, INPUT_ANALOG); //inicia esses pinos de entrada adc
    pinMode(pot2, INPUT_ANALOG);
    //pinMode(pot3, INPUT_ANALOG);

    pinMode(manual_auto, INPUT_PULLUP); // chave manual automático
    pinMode(botao_asa, INPUT_PULLUP); // botão abre asa


    init_mpu();
    init_servo();

    // config_limit(); // usado para ver o valor dos potenciometros
}

void loop() {
    
    /*
    if (digitalRead(manual_auto)) {
        if (a.acceleration.x < -5 || a.acceleration.y > 5 || a.acceleration.y < -5) {
            servo1.write(max_servo1);
            servo2.write(min_servo2);
        } else {
            pos_pot1 = analogRead(pot1);
            Serial.print(" pos_pot1: ");
            Serial.println(pos_pot1);

            pos_servo1 = map(pos_pot1, min_pot1, max_pot1, min_servo1, max_servo2);
            pos_servo2 = map(pos_pot1, min_pot1, max_pot1, max_servo2, min_servo2);
            Serial.print(" pos_servo1: ");
            Serial.println(pos_servo1);
            Serial.println("");

            servo1.write(pos_servo1);
            servo2.write(pos_servo2);
        }
    } else {
        if (digitalRead(botao_asa)){
            servo1.write(max_servo1);
            servo2.write(min_servo2);
        } else {
            servo1.write(min_servo1);
            servo2.write(max_servo2);
        }
    }
    //*/
    Serial.println("teste");

    delay(500);
}

void init_servo() {
    servo1.attach(PA15); //PA15 PB8
    servo2.attach(PA8);

    //move servos
    servo1.write(max_servo1);
    servo2.write(min_servo2);
    delay(1000);
    servo1.write(min_servo1);
    servo2.write(max_servo2);
    delay(1000);
}

void init_mpu() {
    // initialize device
    Serial.println("Initializing I2C devices...");
    //accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    
}