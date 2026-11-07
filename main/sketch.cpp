/**
 * Torque control example using voltage control loop.

 *Full control HELP -> https://docs.simplefoc.com/commander_motor

 *SimpleFOCStudio
 Link:
  https://github.com/JorgeMaker/SimpleFOCStudio/tree/main

  Parancsok:

  cd C:\Users\kises\Documents\SimpleFOCStudio-main\SimpleFOCStudio-main
  conda activate simplefoc
  python simpleFOCStudio.py
  
 * Most of the low-end BLDC driver boards doesn't have current measurement therefore SimpleFOC offers 
 * you a way to control motor torque by setting the voltage to the motor instead hte current. 
 * 
 * This makes the BLDC motor effectively a DC motor, and you can use it in a same way.
 */
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <SPI.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <Preferences.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

//Preferences preferences;
// ************************************ PINOUT ************************************************
  
  #define SERVO1_PIN  21   //szervo
  #define SERVO2_PIN  22

  #define M1_in1 32
  #define M1_in2 33
  #define M1_in3 25
  #define M1_en 26
  #define M2_in1 27
  #define M2_in2 14
  #define M2_in3 12
  #define M2_en 13

  //I2C pinek
  #define SDA 16  //Wire
  #define SCL 15

  //SPI pinek
  #define MISO 19
  #define MOSI 23
  #define SCK 18
  #define CS1 4   //BMI160
  #define CS2 17  //H2
  #define CS3 5   //H3

  //Egyéb pinek
  #define LED_pin 2   // kék LED
  #define BATT_pin 36 // akku fesz. mérés 
  

// ************************************ SERVO ***********************************************
  
  Servo servo;

  //bool servoDirection = 0;        //Servo irány
  //bool lastServoDirection = 0;    //irányváltás segéd változó

  int MinServoValue = 1000;      //SERVO ~180°
  int MaxServoValue = 2000;

  int ParkingPos = 1500;
  int RunningPos = 1000;
  int ServoPos = ParkingPos;
  int LastServoPos = 0; 

//-------------------------------------------------------------------------------------------------
  unsigned long tLast = 0;
  unsigned long tLastMs = 0;
  unsigned long fallingResetTimer = 0;
  int resetTime = 2000;                   // felállítás után eltelt idő, hogy nyissa a kitámasztó kart

  #define dT_MICROSECONDS 50000
  #define dT dT_MICROSECONDS/1000000.0

  unsigned long lastUpdateMicros = 0;

  unsigned long loopCount = 0;
  unsigned long lastHzTime = 0;
  float loopHz = 0;

// ************************************* IMU ***************************************************
  // Időzítő változók az IMU mintavételezéshez
  float pitch = 0.0; 
  
  struct IMUData {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
  } imu;

  unsigned long lastBMItime = 0;
  const unsigned long BMI_INTERVAL = 5; // 5 ezredmásodpercenkénti lekérdezés (200 Hz)

  unsigned long lastPrintTime = 0;
  const unsigned long PRINT_INTERVAL = 200; // 200 ms-onkénti kiírás a monitorra (5 Hz)


  // Segédfüggvény az előjel meghatározásához
  //#define _sign(val) ((val > 0) - (val < 0))


//

//  ************************************* CONTROLLER ***************************************
  bool motorDirection;
  bool lastMotorDirection = false;

  bool steeringDirection = 0;        //Servo irány
  bool lastSteeringDirection = 0;    //irányváltás segéd változó

  int MinsteeringValue = -1000;      //kormányzás maximum érték
  int MaxsteeringValue = 1000;

  int MinthrottleValue = -1500;      //gáz maximum érték  /100
  int MaxthrottleValue = 1500;

  float throttle;
  float steering;

  //ANALÓG
  int LeftX;
  int LeftY;
  int RightX;
  int RightY;

  int L2;
  int R2;

  //Nyomógombok
  bool buttonPS;
  bool lastButtonPS= 0;
  bool buttonMic;
  bool buttonThumbL;

  bool dpad_up;
  bool dpad_down;
  bool dpad_left;
  bool dpad_right;

  //Gombnyomások észlelése
  const int LONG_PRESS_TIME  = 3000; // 3000 milliseconds

  //különböző gombok időzítői
  unsigned long releasedTime_up = 0;    //Dpad "up" felengedés időpontja
  unsigned long releasedTime_left = 0;  //Dpad "left"......

  unsigned long pressedTime_PS=0;

  //nyomógomb segédválltozók
  bool firstpressed_PS = 0;
//
//  ******************************** Egyéb változók **************************************
  bool startBalancing = 0;      // robot egyensúlyozás indítása
  bool falling = false;         // eldőlt ha igaz
  bool stands_again = false;    // igaz ha újra felállítva
  bool swapped = false;
  bool steering_swapped = false;

  bool connected = false;   //failsafe

  float balanceOut;
  float motor1Speed;
  float motor2Speed;
  float setpoint_angle;
  float motor1_velocity;
  float motor2_velocity;

//

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            connected = true;
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            connected = false;
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    /*Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d, battery:%4d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ(),        // Accelerometer Z
        ctl->battery() 
        
    );
    */
}

void processGamepad(ControllerPtr ctl) {
    
    if (ctl->a()) {      // X 
        static int colorIdx = 0;
        // Some gamepads like DS4 and DualSense support changing the color LED.
        // It is possible to change it by calling:
        switch (colorIdx % 3) {
            case 0:
                // Red
                ctl->setColorLED(255, 0, 0);
                break;
            case 1:
                // Green
                ctl->setColorLED(0, 255, 0);
                break;
            case 2:
                // Blue
                ctl->setColorLED(0, 0, 255);
                break;
        }
        colorIdx++;
    }

    if (ctl->b()) {       //KÖR
        // Turn on the 4 LED. Each bit represents one LED.
        static int led = 0;
        led++;
        ctl->setPlayerLEDs(led & 0x0f);
    }

    if (ctl->x()) {   //négyzet
        ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0/* weakMagnitude */,
                            255 /* strongMagnitude */);
        
    }

    if (ctl->y()) {     //Háromszög

    }

    //if (ctl->dpad() == 0X01)Serial.println("UP");
    //if (ctl->dpad() == 0X02)Serial.println("DOWN");
    //if (ctl->dpad() == 0X08)Serial.println("LEFT");
    //if (ctl->dpad() == 0X04)Serial.println("RIGHT");

    //if (ctl->thumbL()) Serial.println("LEFT JOYBUTTON");
    //if (ctl->thumbR()) Serial.println("RIGHT JOYBUTTON");

    //  \|/ -gomb
    //if (ctl->miscSelect()) Serial.println("SELECT");  
    //if (ctl->miscBack()) Serial.println("BACK"); 

    //  PS -gomb
    //if (ctl->miscSystem()) Serial.println("SYSTEM"); 
    
    //  MENU -gomb
    //if (ctl->miscStart()) Serial.println("START");    
    //if (ctl->miscHome()) Serial.println("HOME");
   
    //  MIC -gomb
    //if (ctl->miscCapture()) Serial.println("CAPTURE");
    buttonPS = ctl->miscSystem();
    buttonMic = ctl->miscCapture();
    buttonThumbL = ctl->thumbL();

    //dpad_up
    if (ctl->dpad() == 0X01) dpad_up = true;
    else dpad_up = false;

    //dpad_left
    if (ctl->dpad() == 0X08) dpad_left = true;
    else dpad_left = false;

    LeftX = ctl->axisX();
    LeftY = ctl->axisY();
    L2 =    ctl->brake();
    RightX = ctl->axisRX();
    RightY = ctl->axisRY();
    R2 = ctl->throttle();

  // SWAP Motor direction playRumble
  if(lastMotorDirection != motorDirection) ctl->playDualRumble(0, 500, 0, 150);
  // SWAP SERVO direction playRumble
  if(lastSteeringDirection != steeringDirection) ctl->playDualRumble(0, 500, 0, 150);
}

void processControllers() {
  for (auto myController : myControllers) {
    if(myController && myController->isConnected() && myController->hasData()) {
      if(myController->isGamepad()) {
        processGamepad(myController);
      }else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

//  **************************** READ IMU *********************************************
bool updatePitchIMU() {
  
  // --- IDŐZÍTETT BLOKK A BMI160 SZÁMÁRA ---
  // Csak 5ms-onként fut le, így nem terheli túl az SPI buszt
  if (millis() - lastBMItime >= BMI_INTERVAL) {
    lastBMItime = millis();

    // Biztonsági mikroszekundumos szünet az utolsó SimpleFOC SPI lefutás után
    delayMicroseconds(1); 

    // ÁTVÁLTUNK SPI_MODE3-RA és megnyitjuk a tranzakciót (max 10MHz a BMI160-nak)
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3));
    digitalWrite(CS1, LOW);

    // Regiszter címzés: 0x0C (DATA_0 gyorsulásmérő regiszter) + Olvasás flag (0x80)
    SPI.transfer(0x0C | 0x80); 
    
    // A BMI160 adatlap szerinti VALÓDI SORRENDJE: 6 bájt Gyro, majd 6 bájt Accel!
    uint8_t gx_l = SPI.transfer(0x00); uint8_t gx_h = SPI.transfer(0x00);
    uint8_t gy_l = SPI.transfer(0x00); uint8_t gy_h = SPI.transfer(0x00);
    uint8_t gz_l = SPI.transfer(0x00); uint8_t gz_h = SPI.transfer(0x00);
    
    uint8_t ax_l = SPI.transfer(0x00); uint8_t ax_h = SPI.transfer(0x00);
    uint8_t ay_l = SPI.transfer(0x00); uint8_t ay_h = SPI.transfer(0x00);
    uint8_t az_l = SPI.transfer(0x00); uint8_t az_h = SPI.transfer(0x00);

    digitalWrite(CS1, HIGH); 
    SPI.endTransaction(); // Tranzakció lezárása, MISO felszabadult!

    // Bájtok helyes összefűzése (16-bites előjeles egészekké)
    imu.gx = (int16_t)((gx_h << 8) | gx_l);
    imu.gy = (int16_t)((gy_h << 8) | gy_l);
    imu.gz = (int16_t)((gz_h << 8) | gz_l);
    
    imu.ax = (int16_t)((ax_h << 8) | ax_l);
    imu.ay = (int16_t)((ay_h << 8) | ay_l);
    imu.az = (int16_t)((az_h << 8) | az_l);

  
  
    // 2. Tényleges időkülönbség (dt) kiszámítása másodpercben
    unsigned long now = micros();
    if (lastUpdateMicros == 0) lastUpdateMicros = now;
    float dt = (now - lastUpdateMicros) / 1000000.0; 
    lastUpdateMicros = now;

    // Biztonsági gát (ha a Bluetooth vagy valami más megakasztaná a loopot)
    if (dt > 0.05) dt = 0.005; 

    // 3. Giroszkóp dőlési sebességének átváltása radián/másodpercbe
    // Gyroszkóp Y tengely (pitch). 131.07-tel osztunk fok/mp-ért.
    // Ha ±250°/s-ra konfiguráltad, akkor használd a 131.07-et!
    float gyroY_rad = ((float)imu.gy / 131.07) * (PI / 180.0);
  
    // 4. Gyorsulásmérő adatok skálázása (+/-2g tartománynál az osztó 16384.0)
    float ax = (float)imu.ax/ 16384.0;
    float az = (float)imu.az / 16384.0;
  
    // Szög kiszámítása tisztán a gyorsulásmérőből (radiánban)
    // Az X és Z tengelyek határozzák meg a pitch szöget a függőlegeshez képest
    float accel_pitch = atan2(-ax, az);
  
    // 5. KOMPLEMENTER SZŰRŐ (Szenzorfúzió)
    // Integráljuk a giroszkópot, és finoman hozzáigazítjuk a gyorsulásmérő abszolút szögét
    float pitch_new = 0.98 * (pitch + gyroY_rad * dt) + 0.02 * accel_pitch;    //0.02
    
    pitch = pitch_new;
    

    
  
    if(abs(pitch) > 0.70){
      falling = true;
      stands_again = false;
    }
    else{
      if(stands_again == false){
        stands_again = true;
        fallingResetTimer = millis();
      }

      if(millis() - fallingResetTimer > resetTime){    // várunk egy kicsit, mire reseteljük a felállítást
        falling = false;
      }
    }
    return true;
  } 
  return false;
}

void convertInValue(){

  //Servo vezérlés, irány meghatározás
  int steeringValue;
  if(steeringDirection) steeringValue = map(LeftX, -512, 512, MinsteeringValue, MaxsteeringValue);
  else steeringValue = map(LeftX, -512, 512, MaxsteeringValue, MinsteeringValue);
  steering = float(steeringValue)*0.001;
  //Serial.print("steering"); Serial.print(steering); 

  /*
  // 1. Joystick bemenet tisztítása (Holtjáték / Deadband kezelés)
  float raw_steering = steering_joystick_input; // A távirányítóról jövő nyers érték
  float deadband = 0.08; // 8%-os holtjáték, ezt szükség szerint finomíthatod

  if (abs(raw_steering) < deadband) {
      steering = 0.0; // Ha a holtjátékon belül van, garantáltan tiszta nulla
  } else {
      // Opcionális finomítás: skálázzuk újra a maradék tartományt, 
      // hogy a kormányzás finoman, nulláról induljon a holtjáték határától
      steering = _sign(raw_steering) * (abs(raw_steering) - deadband) / (1.0 - deadband);
}
  */
  
  //Gáz, fék, hátramenet vezérlés
  int throttleValue;
  if(motorDirection) throttleValue = map(RightY, -512, 512, MinthrottleValue, MaxthrottleValue);
  else throttleValue = map(RightY, -512, 512, MaxthrottleValue, MinthrottleValue);
  throttle = float(throttleValue)*0.01;
  //Serial.print(" throttle"); Serial.println(throttle); 
  
}

void failSafe(){
  steering = 0.0;
  throttle = 0.0;
  startBalancing = 0;
}

void processButtons(){

  //**********************************SWAP Motor Direction*************************************************************
  if(buttonMic && dpad_up){                                 //Microfon és FEL gomb egyszerre nyomva 
    if(millis() - releasedTime_up >= LONG_PRESS_TIME){
      if (swapped == false){                                //Ha még nem volt csak akkor fordítjuk
        motorDirection = !motorDirection;
        swapped = true;
        //saveData
        //preferences.putBool("motorDirection", motorDirection);  // állapot mentése
        //delay(100);
        //Serial.print("motorDirection");Serial.println(motorDirection);
        //myMotor.swapDirection(motorDirection);
      } 
    }
  }
  else{
    releasedTime_up = millis();
    swapped = false;                                      //fordítási rutin reset
    lastMotorDirection = motorDirection;
  }
//
//****************************************** SWAP Steering Direction ************************************************************
  if(buttonMic && dpad_left){                                 //Microfon és Bal gomb egyszerre nyomva 
    if(millis() - releasedTime_left >= LONG_PRESS_TIME){
      if (steering_swapped == false){                                //Ha még nem volt csak akkor fordítjuk
        steeringDirection = !steeringDirection;
        steering_swapped = true;
        //saveData
        //preferences.putBool("steeringDirection", steeringDirection);  // állapot mentése
        //delay(100);
        //Serial.print("steeringDirection");Serial.println(steeringDirection);
      } 
    }
  }
  else{
    releasedTime_left = millis();
    steering_swapped = false;                                      //fordítási rutin reset
    lastSteeringDirection = steeringDirection;
  }
//
//******************************************* Start-stop balancing ***********************************
  if(buttonPS){                          //megnyomva
    if(millis() - pressedTime_PS > 50){   //debounce
      if(firstpressed_PS == 0){
        startBalancing = !startBalancing;
        firstpressed_PS = 1;
      }              
    }
  }

  if(!buttonPS){
    pressedTime_PS = millis();
    firstpressed_PS = 0;
  }   

}

//  ****************************************** MOTOR ********************************************

  MagneticSensorSPI sensor1 = MagneticSensorSPI(AS5047_SPI, CS2);
  MagneticSensorSPI sensor2 = MagneticSensorSPI(AS5047_SPI, CS3);
  

  // BLDC motor & driver instance
  // BLDCMotor(pole pair number, phase resistance [Ohms], KV rating [rpm/V])
  BLDCMotor motor1 = BLDCMotor(7, 12.5, 100, 0.015);
  BLDCMotor motor2 = BLDCMotor(7, 12.5, 100, 0.015);

  BLDCDriver3PWM driver1 = BLDCDriver3PWM(M1_in1, M1_in2, M1_in3, M1_en);
  BLDCDriver3PWM driver2 = BLDCDriver3PWM(M2_in1, M2_in2, M2_in3, M2_en);

  float U_supply = 8;          // tápfesz 
  float U_limit = 8;       // motor feszlimit
  float I_limit = 1;     // motor áramlimit
//
  //  **************************************** PID & LPF ******************************************
  
  // 1. SEBESSÉG PID: A kerekek sebességét finoman igazítja a kívánt gázhoz (throttle)
  // P=0.5 jó indulásnak, de az I tagot picit emelhetjük, a limit legyen magasabb!
  PIDController PID_speed{0.01, 0.0, 0.0, 100, 0.2};        // S

  // 2. EGYENSÚLYOZÓ PID (A robot magja): A dőlési szögből (pitch) számol keréksebességet.
  // Mivel a motor közvetlen hajtású, a P tagot fel kell vinni, a limitet pedig 15-20 rad/s-ra emelni!
  PIDController PID_balance{1, 0.0, 0.0, 100000, 1.0};     // B

  // 3. MOTOR PID-ek: Mivel a SimpleFOC VELOCITY módban belsőleg már szabályozza a sebességet,
  // a kódodban lévő külső PID-et Transzparensre (Átengedőre) kell állítani, hogy ne akadjon össze!
  PIDController PID_motor1{1.0, 0.0, 0.0, 10000, 1.0}; 
  PIDController PID_motor2{1.0, 0.0, 0.0, 10000, 1.0}; 

  // 4. LOW-PASS SZŰRŐK: A Tf értékeket milliszekundumos tartományba kell hozni!
  LowPassFilter lpf_speed{0.05};    // 5 ms-os szűrés a sebességre          Vf..

  LowPassFilter lpf_motor1{0.02};   // 20 ms-os szűrés a keréksebességre
  LowPassFilter lpf_motor2{0.02};   // 20 ms-os szűrés a keréksebességre

  // A távirányító bemenetek (joystick) maradhatnak lassabbak a sima vezetéshez
  LowPassFilter lpf_throttle{0.02};   // 20 ms-os simítás a gázra           Tf
  LowPassFilter lpf_steering{0.02};   // 20 ms-os simítás a kormányzásra    Sf

  //
//  *************************************** COMMANDER *****************************************
  // instantiate the commander
  Commander command = Commander(Serial);
  void onMotor(char* cmd){ command.motor(&motor1, cmd); }

  void pidBal(char* cmd) {  command.pid(&PID_balance, cmd);}
  void pidSpd(char* cmd) {  command.pid(&PID_speed, cmd);}

  void pidMotor1(char* cmd) {  command.pid(&PID_motor1, cmd);}
  void pidMotor2(char* cmd) {  command.pid(&PID_motor2, cmd);}

  void lpfSpeed(char* cmd) {  command.lpf(&lpf_speed, cmd);}
  void lpfSteering(char* cmd) {  command.lpf(&lpf_steering, cmd);}
  void lpfThrottle(char* cmd) {  command.lpf(&lpf_throttle, cmd);}


  void lpfMotor1(char* cmd) {  command.lpf(&lpf_motor1, cmd);}
  void lpfMotor2(char* cmd) {  command.lpf(&lpf_motor2, cmd);}

  //void angleOffs(char* cmd) {angleOffset = atof(cmd);}
  //void gyroFilter(char* cmd) {gyroFilterConstant = atof(cmd);}

//...
//  ******************************************* SETUP *****************************************
void setup() {

  Serial.begin(115200);
  delay(200);

  // initialise I2C hardware
  //Wire.setClock(400000);
  Wire.begin(SDA, SCL); 
  
//Init IMU
  // --- HARDVERES CS LÁB BEÁLLÍTÁSOK ---
  // Az összes CS lábat OUTPUT-ra tesszük és HIGH szintre emeljük (MISO leválasztás!)
  //pinMode(CS2, OUTPUT); digitalWrite(CS2, HIGH);
  //pinMode(CS3, OUTPUT); digitalWrite(CS3, HIGH);
  pinMode(CS1, OUTPUT); digitalWrite(CS1, HIGH);

  // Adjunk időt a szenzoroknak a belső Power-On-Reset-re (POR)
  delay(50); 

  // --- SPI BUSZ INDÍTÁSA ---
  SPI.begin(SCK, MISO, MOSI);

  // --- BMI160 SPI MÓDBA KÉNYSZERÍTÉSE ---
  // Az adatlap szerint indítás után egy CSB felfutó él kell az SPI üzemmód aktiválásához
  digitalWrite(CS1, LOW);   delayMicroseconds(10);
  digitalWrite(CS1, HIGH);  delayMicroseconds(10);

  // Végrehajtunk egy kézi tranzakciót és egy vak olvasást a BMI160-on SPI_MODE3-ban
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS1, LOW);
  SPI.transfer(0x00 | 0x80); // CHIP_ID regiszter (0x00) + Olvasás maszk (0x80)
  uint8_t chip_id = SPI.transfer(0x00);
  digitalWrite(CS1, HIGH);
  SPI.endTransaction();

  Serial.print("BMI160 Chip ID keresese... ");
  if(chip_id == 0xD1) {
    Serial.println("SIKERES! (0xD1, SPI Mode 3 aktív)");
  } else {
    Serial.print("HIBA! Kapott ID: 0x"); Serial.println(chip_id, HEX);
    Serial.println("Ellenorizd a bekotest vagy a tápfeszültséget!");
  }
  // --- BMI160 FELÉBRESZTÉSE (POWER MODES) ---
  // Az adatlap szerint a 0x7E regiszterbe (CMD) küldött parancsokkal ébresztjük fel a chipeket
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  
  // 1. Gyorsulásmérő bekapcsolása normál módba (0x11 parancs a 0x7E regiszterbe)
  digitalWrite(CS1, LOW);
  SPI.transfer(0x7E & 0x7F); // Írás parancs (MSB = 0)
  SPI.transfer(0x11);
  digitalWrite(CS1, HIGH);
  delay(5); // Biztosítsunk időt az indításra

  // 2. Giroszkóp bekapcsolása normál módba (0x15 parancs a 0x7E regiszterbe)
  digitalWrite(CS1, LOW);
  SPI.transfer(0x7E & 0x7F);
  SPI.transfer(0x15);
  digitalWrite(CS1, HIGH);
  delay(80); // A giroszkópnak kb. 80ms kell, hogy teljesen stabilizálódjon az első mérés előtt

  SPI.endTransaction();
  Serial.println("BMI160 felébresztve!");

  // --- GYROSZKÓP ÁTCONFIGURÁLÁSA ±250°/s TARTOMÁNYRA ---
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  
  digitalWrite(CS1, LOW);
  SPI.transfer(0x43 & 0x7F); // 0x43 regiszter címzése írásra (MSB = 0)
  SPI.transfer(0x03);        // 0x03 érték = ±250°/s méréshatár
  digitalWrite(CS1, HIGH);
  delay(5);                  // Kis idő a regiszter mentésére

  SPI.endTransaction();
  Serial.println("BMI160 Gyro beallitva: +-250 deg/s");

// SERVO------------------------------------------------------------------------------------------------------------------------------
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);    // standard 50 hz servo
  servo.attach(SERVO1_PIN, MinServoValue, MaxServoValue); //
  
  servo.writeMicroseconds(ServoPos);
  delay(500);

//
// Controller
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);
  delay(300);

// MOTOR / SENSOR
  SimpleFOCDebug::enable(&Serial);

  sensor1.spi_mode = SPI_MODE1;
  sensor1.clock_speed = 10000000;
  sensor1.init();

  sensor2.spi_mode = SPI_MODE1;
  sensor2.clock_speed = 10000000;
  sensor2.init();

  delay(100);

  // link the motor to the sensor
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);
  delay(100);

  driver1.voltage_power_supply = U_supply;
  driver1.init();
  motor1.linkDriver(&driver1);

  driver2.voltage_power_supply = U_supply;
  driver2.init();
  motor2.linkDriver(&driver2);

  // aligning voltage 
  motor1.voltage_limit = U_limit;
  motor2.voltage_limit = U_limit;
  
  motor1.current_limit = I_limit;
  motor2.current_limit = I_limit;

  // choose FOC modulation (optional)
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.torque_controller = TorqueControlType::estimated_current;
  motor1.controller = MotionControlType::torque;

  motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor2.torque_controller = TorqueControlType::estimated_current;
  motor2.controller = MotionControlType::torque;


  // initialize motor                     
  motor1.init();
  motor1.sensor_direction = CW;
  motor1.zero_electric_angle = 4.66;
  
  motor2.init();
  motor2.sensor_direction = CCW;
  motor2.zero_electric_angle = 5.43;

  //motor1.LPF_velocity.Tf = 0.02;
  //motor2.LPF_velocity.Tf = 0.02;
  
  motor1.initFOC();
  motor2.initFOC();
  Serial.println("motor initFOC");

  Serial.print("motor 1 zero angle: ");Serial.println(motor1.zero_electric_angle);
  Serial.print("motor 2 zero angle: ");Serial.println(motor2.zero_electric_angle);
//
// COMMANDER
  // define the motor id
  //command.add('M', onMotor, "motor");
  command.add('B', pidBal, "PID_balance");    // BP0.5, BI0.001, ...
  command.add('S', pidSpd, "PID_speed");

  command.add('L', pidMotor1, "PID_motor1");
  command.add('R', pidMotor2, "PID_motor2");

  command.add('A', lpfMotor1, "lpf_motor1");
  command.add('C', lpfMotor2, "lpf_motor2");

  command.add('V', lpfSpeed, "lpf speed");    // VF0.07
  command.add('T', lpfThrottle, "lpf throttle");
  command.add('I', lpfSteering, "lpf steering");

  //command.add('O', angleOffs, "angleOffset");   //O...
  //command.add('G', gyroFilter, "gyroFilterConstant");   //G...

  motor1.useMonitoring(Serial);
  motor1.monitor_downsample = 300; //
  //MONITORING https://docs.simplefoc.com/monitoring
  motor1.monitor_variables = _MON_VEL;

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  //Serial.println(F("Motor commands sketch | Initial motion control > torque/voltage : target 2V."));
  delay(500);
//
}
//  ******************************************* LOOP ******************************************  
void loop() {
  /*
  - PID értékek mentése
  - PID_balance.P = 0.2; kipróbálni preferenceshez.
  - 
  - offset érték automatikus kalibrálása;
  - Motorok feszültségének emelése (6V / 0.6A kicsit gyenge.) *
  - egyenes vonal tartás finomhangolása
  - sztender beépítése, servo könyvtár, pwm időzítők kipróbálása
  - enable kimenet a mrgfelelő lábra átkötni
  - felborulás érzékelése*
  - joystik deadzone
  
  */
  motor1.loopFOC();
  motor2.loopFOC();
 
  loopCount++;

  unsigned long tNow = micros();    // micros itőzítő
  unsigned long tNowMs = millis();  // milis időzítő


  if(tNow-tLast > dT_MICROSECONDS) {    //Azok a dolgok amiknek nem kell minden hurokban lefutni 
    bool dataUpdated = BP32.update();   //A bluetoth adatok frissítése
    processControllers();               //A controller függvényei 
    
    motor1.move();
    motor2.move();

    if(connected){                      // Ha csatlakoztatva van a controller....
      convertInValue();
      processButtons();

      
      if(ServoPos != LastServoPos){
        LastServoPos = ServoPos;
        servo.writeMicroseconds(ServoPos);
      }
    }
    else failSafe();

    tLast = tNow;
  }

  if(updatePitchIMU()){ 

    if(falling == true) startBalancing = false;       // Ha eldőlt, akkor leállítjuk a ciklust, és csak gombnyomásra indul
    if(falling == false && startBalancing == false){  // ha nem dőlt el, de nincs indítva a kitámasztót kinyitja
      ServoPos = ParkingPos;   
    } 
    
    if(falling == false && startBalancing == true){
      if(ServoPos == ParkingPos) ServoPos = RunningPos;  // ha indult a robot, a kitámasztót behúzza

      motor1_velocity = motor1.shaft_velocity;  //lpf_motor1(motor1.shaft_velocity); 
      motor2_velocity = motor2.shaft_velocity;  //lpf_motor2(motor2.shaft_velocity);
      //cél dőlésszőg meghatározása, a keréksebbesség és akívánt sebesség alapján    tartomány:~ -80 / 80

      // A két motor átlagsebessége szűrve (rad/s)
      float current_robot_speed = lpf_speed((motor1_velocity + motor2_velocity) / 2.0); //*-1 hogy az előjel megfelelő
      
      // Cél dőlésszög (A joystick throttle és a kerekek jelenlegi sebességének különbsége (rad)
      setpoint_angle = PID_speed(current_robot_speed - lpf_throttle(throttle));  

      //balance out Amperben
      balanceOut = PID_balance(setpoint_angle - pitch);

      //Amperben, mint a balance limit kb. 20-30%-a (pl. max 0.2 - 0.3 Amper),
      float filteredSteering = lpf_steering(steering);

      motor1Speed = balanceOut - filteredSteering; 
      motor2Speed = balanceOut + filteredSteering; 

      motor1.target = motor1Speed;// 
      motor2.target = motor2Speed;// 

      /*
      // 2. EGYENESVONAL TARTÁS (Yaw Compensation)
      float yaw_error = 0.0;
      if (steering == 0.0) {  
        // Ha a pilóta nem kormányoz, a két kerék sebességének meg kell egyeznie.
        // Ha a motor1 gyorsabb, a különbség pozitív -> a PID-nek csökkentenie kell a motor1 nyomatékát.
        yaw_error = motor1_velocity - motor2_velocity;
      } else {
        // Ha szándékosan kanyarodunk, kikapcsoljuk az egyenesen tartást
        yaw_error = 0.0; 
        PID_yaw.reset();  // Reseteljük a PID integrátorát, hogy ne halmozódjon fel a hiba
      }

      // A yaw PID kiszámolja a korrekciós áramot (Amperben)
      float yaw_correction = PID_yaw(yaw_error);

      // 3. Kormányzás és korrekció összevonása
      // Ha nem kormányzol, a filteredSteering 0, és a yaw_correction tartja egyenesen.
      float total_steering = lpf_steering(steering) + yaw_correction;

      // 4. Végső motoráramok (SimpleFOC target)
      motor1.target = balanceOut - total_steering; 
      motor2.target = balanceOut + total_steering;
      */

    }else{
      motor1.target = 0;
      motor2.target = 0;
      PID_speed.reset();
      PID_balance.reset();
    }
  }
    
  // --- LOOP FREKVENCIA SZÁMÍTÁSA (Minden 1 másodpercben) ---
  if (tNowMs - lastHzTime >= 1000) {
    loopHz = loopCount; // Mivel 1000ms telt el, a loopCount pontosan a Hz-et jelenti
    loopCount = 0;
    lastHzTime = tNowMs;
  }
  
  //Serial Monitor
  if (tNowMs-tLastMs > 300){
    //Serial.print("  Loop FOC sebesség: "); Serial.print(loopHz); Serial.print(" Hz");
    //Serial.print("Vel1: ");Serial.print(motor1.shaft_velocity);
    //Serial.print("  Vel2: ");Serial.print(motor2.shaft_velocity);
    //Serial.print("  setpoint_angle: ");Serial.print(setpoint_angle);
    //Serial.print("  pitch:");Serial.print(pitch);
    //Serial.print("  balanceOut: ");Serial.print(balanceOut);
    //Serial.print("  motor1Speed ");Serial.print(motor1Speed);
    //Serial.print("  connected: "); Serial.print(connected);
    //Serial.print("  steering: "); Serial.print(steering);
    //Serial.print("  throttle: "); Serial.print(throttle);
    //Serial.print(" falling: "); Serial.print(falling);
    //Serial.print(" start: "); Serial.print(startBalancing);
    //Serial.print(" servoPos: "); Serial.print(ServoPos);
    //motor1.monitor();
    //Serial.println();
    tLastMs = tNowMs;
  }
  
  //motor1.monitor();
  //user communication
  command.run();
}



