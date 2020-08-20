#include <Arduino.h>
#include <Servo.h>

Servo servo1;
Servo servo2;

bool blinkState = false;
#define LED_PIN PC13

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

void setup()
{
    Serial.begin(9600);

    pinMode(LED_PIN, OUTPUT);
    pinMode(pot1, INPUT_ANALOG); //inicia esses pinos de entrada adc
    pinMode(pot2, INPUT_ANALOG);
    pinMode(manual_auto, INPUT_PULLUP); // chave manual automático
    pinMode(botao_asa, INPUT_PULLUP);   // botão abre asa

    init_mpu();
    init_servo();

    // config_limit(); // usado para ver o valor dos potenciometros
}

void loop()
{
    //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    /*
    // display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.print("\t");
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(gy);
    Serial.print("\t");
    Serial.println(gz);
    //*/
    /*
    if (digitalRead(manual_auto)) {
        if (a.acceleration.x < -10 || a.acceleration.y > 10 || a.acceleration.y < -10) {
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
    Serial.println("posicao servo");
    //Serial.println(servo1.read());
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(500);
}

void config_limit()
{
    int delay_value = 2000;

    Serial.println("leitura do potenciometro 1 ");
    while (!digitalRead(botao_asa))
    {
        Serial.print("pot1:");
        Serial.println(analogRead(pot1));
        delay(delay_value);
    }
    Serial.println("OK");
    delay(delay_value);

    Serial.println("leitura do potenciometro 2 ");
    while (!digitalRead(botao_asa))
    {
        Serial.print("pot2:");
        Serial.println(analogRead(pot2));
        delay(delay_value);
    }
    Serial.println("OK");
    delay(delay_value);

    Serial.println("leitura do servo1");
    while (!digitalRead(botao_asa))
    {
        Serial.print("servo1:");
        //Serial.println(servo1.read());
        delay(delay_value);
    }
    Serial.println("OK");
    delay(delay_value);

    Serial.println("leitura do servo2");
    while (!digitalRead(botao_asa))
    {
        Serial.print("servo2:");
        //Serial.println(servo2.read());
        delay(delay_value);
    }
    Serial.println("OK");
    delay(delay_value);

    Serial.println("Anote os valores e mude no código");
    delay(delay_value);
}

void init_servo()
{
    //servo1.attach(PA15); //PA15 PB8
    //servo2.attach(PA8);

    //move servos
    //servo1.write(max_servo1);
    //servo2.write(min_servo2);
    delay(1000);
    //servo1.write(min_servo1);
    //servo2.write(max_servo2);
    delay(1000);
}

void init_mpu()
{
    
}