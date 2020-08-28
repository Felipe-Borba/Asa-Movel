#include <Servo.h>


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

bool led_state = false;

void init_servo();
void config_limit();

void setup() {
    Serial.begin(115200);
    
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(pot1, INPUT_ANALOG); //inicia esses pinos de entrada adc
    pinMode(pot2, INPUT_ANALOG);
    //pinMode(pot3, INPUT_ANALOG);

    pinMode(manual_auto, INPUT_PULLUP); // chave manual automático
    pinMode(botao_asa, INPUT_PULLUP); // botão abre asa

    init_servo();
    
    // config_limit(); // usado para ver o valor dos potenciometros
}

void loop() {
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
    
    //blink led to display activity
    led_state != led_state;
    digitalWrite(LED_BUILTIN, led_state);

    //teste
    Serial.print("pot1:");
    Serial.println(analogRead(pot1));
    Serial.print("pot2:");
    Serial.println(analogRead(pot2));
}
/*
void config_limit() {
    int delay_value = 2000;

    Serial.println("leitura do potenciometro 1 ");
    while (!digitalRead(botao_asa)) {
        Serial.print("pot1:");
        Serial.println(analogRead(pot1));
        delay(delay_value);
    }
    Serial.println("OK");
    delay(delay_value);

    Serial.println("leitura do potenciometro 2 ");
    while (!digitalRead(botao_asa)) {
        Serial.print("pot2:");
        Serial.println(analogRead(pot2));
        delay(delay_value);
    }
    Serial.println("OK");
    delay(delay_value);

    Serial.println("leitura do servo1");
    while (!digitalRead(botao_asa)) {
        Serial.print("servo1:");
        Serial.println (servo1.read());
        delay(delay_value);
    }
    Serial.println("OK");
    delay(delay_value);

    Serial.println("leitura do servo2");
    while (!digitalRead(botao_asa)) {
        Serial.print("servo2:");
        Serial.println (servo2.read());
        delay(delay_value);
    }
    Serial.println("OK");
    delay(delay_value);

    Serial.println("Anote os valores e mude no código");
    delay(delay_value);
}
//*/

void init_servo() {
    servo1.attach(PA15); //mudar  
    servo2.attach(PA8); //mudar

    //move servos
    servo1.write(max_servo1);
    servo2.write(min_servo2);
    delay(1000);
    servo1.write(min_servo1);
    servo2.write(max_servo2);
    delay(1000);
}
/*
void init_mpu() {
    Serial.println("Adafruit MPU6050 test!");
    delay(1000);
    // Try to initialize!
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
        Serial.println("+-2G");
        break;
    case MPU6050_RANGE_4_G:
        Serial.println("+-4G");
        break;
    case MPU6050_RANGE_8_G:
        Serial.println("+-8G");
        break;
    case MPU6050_RANGE_16_G:
        Serial.println("+-16G");
        break;
    }

    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.print("Gyro range set to: ");
    switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
        Serial.println("+- 250 deg/s");
        break;
    case MPU6050_RANGE_500_DEG:
        Serial.println("+- 500 deg/s");
        break;
    case MPU6050_RANGE_1000_DEG:
        Serial.println("+- 1000 deg/s");
        break;
    case MPU6050_RANGE_2000_DEG:
        Serial.println("+- 2000 deg/s");
        break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    Serial.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
        Serial.println("260 Hz");
        break;
    case MPU6050_BAND_184_HZ:
        Serial.println("184 Hz");
        break;
    case MPU6050_BAND_94_HZ:
        Serial.println("94 Hz");
        break;
    case MPU6050_BAND_44_HZ:
        Serial.println("44 Hz");
        break;
    case MPU6050_BAND_21_HZ:
        Serial.println("21 Hz");
        break;
    case MPU6050_BAND_10_HZ:
        Serial.println("10 Hz");
        break;
    case MPU6050_BAND_5_HZ:
        Serial.println("5 Hz");
        break;
    }    
}
//*/
