#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;
#define INTERRUPT_PIN PB11
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ---- //
Servo servo1;  // cria um objeto para controlar o servo
Servo servo2;  // cria um objeto para controlar o servo

#define manual_auto PB4 //Não posso usar o PA11 e PA12 pq eles são usaedos pelo USB
#define botao_asa PB3
#define pot1 PA0
#define pot2 PA1
//#define pot3 PB1

// Potentiometer parameter //
int max_pot1 = 0, min_pot1 = 1022;
int max_pot2 = 0, min_pot2 = 1022;
int pos_pot1, pos_pot2;

// Servo parameter //
int max_servo1 = 0, min_servo1 = 180;
int max_servo2 = 0, min_servo2 = 180;
int pos_servo1, pos_servo2;

// Threshold limit //
#define RCornering 800    //  800 para fordKa
#define LCornering -700   // -700 para fordKa
#define Brake -1700       //-1700 para fordKa
#define Acceleration 550  // 0 ~ 1022

// Functions //
void init_servo();
void init_mpu();
void dispay_info();
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.begin(115200);
  //while (!Serial); // espera uma conexão pela serial

  //configure potentiometers
  pinMode(pot1, INPUT_ANALOG); //inicia esses pinos de entrada adc
  pinMode(pot2, INPUT_ANALOG);

  //configure buttons
  pinMode(manual_auto, INPUT_PULLUP); // chave manual automático
  pinMode(botao_asa, INPUT_PULLUP); // botão abre asa

  // configure LED for output
  pinMode(LED_BUILTIN, OUTPUT);

  init_mpu();



  /*/ wait for ready                               //0 = +/- 2g
    Serial.print("escala:");                         //1 = +/- 4g
    Serial.println(mpu.getFullScaleAccelRange());    //2 = +/- 8g
                                                   //3 = +/- 16g

    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
    //*/

  init_servo();

  //Serial.flush();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    //pos_pot1 = analogRead(pot1);
    float alfa = 0.1;
    pos_pot1 = ( alfa * pos_pot1 ) + ( 1.0f - alfa ) * analogRead(pot1);

    dispay_info();
  }
  //*
  if (digitalRead(manual_auto)) {
    if (((aaWorld.x - Brake) <= 1  ) ||
        (aaWorld.y >= RCornering ) ||
        (aaWorld.y <= LCornering ) ) {
      servo1.write(max_servo1);
      servo2.write(min_servo2);
      while (analogRead(pot1) < Acceleration && digitalRead(manual_auto)) {
        //bloqueia enquanto não detectar uma aceleração pelo potenciometro no acelerador
        dispay_info();


      }
    } else {

      // Serial.print(" pos_pot1: ");
      // Serial.println(pos_pot1);

      pos_servo1 = map(pos_pot1, min_pot1, max_pot1, min_servo1, max_servo2);
      pos_servo2 = map(pos_pot1, min_pot1, max_pot1, max_servo2, min_servo2);
      // Serial.print(" pos_servo1: ");
      // Serial.println(pos_servo1);
      // Serial.println("");

      servo1.write(pos_servo1);
      servo2.write(pos_servo2);
    }
  } else {
    if (digitalRead(botao_asa)) {
      servo1.write(max_servo1);
      servo2.write(min_servo2);
    } else {
      servo1.write(min_servo1);
      servo2.write(max_servo2);
    }
  }
  //*/

  //blink led to display activity
  if (blinkState)
    blinkState = false;
  else
    blinkState = true;

  digitalWrite(LED_BUILTIN, blinkState);
}

// ================================================================
// ===                        Functions                         ===
// ================================================================
void dispay_info() {
  //Pode visualizar os dados pelo Plotter serial (Ctrl+Shift+L)
  //Sensibilidade é 2G então divide por 8192?
  //
  Serial.print("aaWorld.x:");
  Serial.print(aaWorld.x);

  Serial.print("\t");
  Serial.print("aaWorld.y:");
  Serial.print(aaWorld.y);

  Serial.print("\t");
  Serial.print("aaWorld.z:");
  Serial.print(aaWorld.z);

  //
  Serial.print("\t");
  Serial.print("RCornering:");
  Serial.print(RCornering);

  Serial.print("\t");
  Serial.print("LCornering:");
  Serial.print(LCornering);

  Serial.print("\t");
  Serial.print("Brake:");
  Serial.println(Brake);
  /*//

    Serial.print("\t");
    Serial.print("GetAccel.x:");
    Serial.print(aa.x);

    Serial.print("\t");
    Serial.print("GetAccel.y:");
    Serial.print(aa.y);

    Serial.print("\t");
    Serial.print("GetAccel.z:");
    Serial.print(aa.z);
    ///
    Serial.print("\t");
    Serial.print("Acceleration:");
    Serial.println(Acceleration);

    Serial.print("\t");
    Serial.print("pot1:");
    Serial.println(pos_pot1);

    Serial.print("\t");
    Serial.print("pos_pot2:");
    Serial.println(analogRead(pot2));
    //*/
  Serial.print("\n");
}
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

void init_mpu() {

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  /*/ wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
    //*/

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));

  }
}
