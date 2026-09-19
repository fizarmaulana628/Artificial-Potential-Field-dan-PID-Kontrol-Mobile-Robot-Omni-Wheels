#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WebSocketsServer.h>

// --- KONFIGURASI WIFI & WEBSOCKET ---
const char* ssid = "Robot_Omni_AP"; 
const char* password = "robot12345"; 
WebSocketsServer webSocket = WebSocketsServer(81);

Adafruit_MCP23X17 mcp;
LiquidCrystal_I2C lcd(0x27, 16, 2); 
Adafruit_MPU6050 mpu;

// --- PIN MAPPING ---
const int PIN_MCP_IN1[] = { -1, 0, 2, 4, 6 }; 
const int PIN_MCP_IN2[] = { -1, 1, 3, 5, 7 }; 
const int PIN_PWM[]     = { -1, 12, 13, 14, 15 }; 
const int PIN_ENC_A[]   = { -1, 16, 18, 23, 26 }; 
const int PIN_ENC_B[]   = { -1, 17, 19, 25, 27 };
const int BTN_UP = 8, BTN_DOWN = 9, BTN_OK = 10, BTN_BACK = 11;
const int MUX_S0 = 32, MUX_S1 = 33, MUX_S2 = 4, MUX_S3 = 2, MUX_SIG_PIN = 35, TRIG_ALL_PIN = 5; 

// --- PARAMETER FISIK ---
const float R_RODA = 0.041, LX_LY = 0.15, PPR = 616.0;
const int SAMPLING_MS = 50;   

// --- TAMBAHAN BARU: FAKTOR KOREKSI ODOMETRI --- 
float KOREKSI_X = 0.873; //Koreksi = 131 / 150 = 0.873
float KOREKSI_Y = 0.873;

// --- PARAMETER APF ---
float K_ATT = 5.0, K_REP = 80.0, DIST_DETECT_M = 0.25, MAX_SPEED_RPM = 100.0; 

// TAMBAHKAN DUA BARIS INI:
float DIST_DETECT_CM = 25.0; 
float DIST_PANIC_CM  = 15.0;

// --- PARAMETER PID HEADING ---
float KP_HEADING = 15.0, targetHeading = 0, currentHeading = 0, gyroZ_offset = 0;

// --- VARIABEL STATE, ODOMETRI & WAKTU ---
float posX = 0, posY = 0, posTheta = 0;
float targetX = 0, targetY = 0;
volatile long encCount[5] = {0,0,0,0,0};
long lastEncCount[5] = {0,0,0,0,0};
unsigned long lastTime = 0;
// Faktor kalibrasi motor (Index 0 dibiarkan kosong, Index 1-4 untuk M1-M4)
// Karena robot lari ke KANAN, kita "lemahkan" motor KIRI (M1 dan M3) menjadi 0.90
float motorScale[5] = {0.0, 1.0, 1.0, 1.0, 1.0};

unsigned long startTimeRobot = 0;
float travelTime = 0;             
int currentPWM[5] = {0,0,0,0,0};

enum RobotState { SET_X, SET_Y, READY, RUNNING, FINISHED };
RobotState state = SET_X;

// --- VARIABEL RINTANGAN VIRTUAL ---
const int MAX_VIRTUAL_OBS = 10;
float virtualObsX[MAX_VIRTUAL_OBS];
float virtualObsY[MAX_VIRTUAL_OBS];
int virtualObsCount = 0;

// --- ISR ENCODER ---
void IRAM_ATTR isr_M1() { if(digitalRead(PIN_ENC_B[1])) encCount[1]--; else encCount[1]++; }
void IRAM_ATTR isr_M2() { if(digitalRead(PIN_ENC_B[2])) encCount[2]--; else encCount[2]++; }
void IRAM_ATTR isr_M3() { if(digitalRead(PIN_ENC_B[3])) encCount[3]++; else encCount[3]--; }
void IRAM_ATTR isr_M4() { if(digitalRead(PIN_ENC_B[4])) encCount[4]++; else encCount[4]--; }

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    String msg = String((char*)payload);
    if (msg == "START") {
      calibrateMPU();
      for(int i=1; i<=4; i++) lastEncCount[i] = encCount[i];
      currentHeading = 0; posX = 0; posY = 0; travelTime = 0;
      startTimeRobot = millis(); state = RUNNING; lastTime = millis();
      updateLCDMenu();
    }
    else if (msg == "RESET") {
      stopMotors(); state = SET_X; virtualObsCount = 0;
      posX = 0; posY = 0; currentHeading = 0;
      updateLCDMenu();
    }
    else if (msg.startsWith("TARGET,")) {
      int firstComma = msg.indexOf(',');
      int secondComma = msg.indexOf(',', firstComma + 1);
      targetX = msg.substring(firstComma + 1, secondComma).toFloat();
      targetY = msg.substring(secondComma + 1).toFloat();
      stopMotors(); state = READY; updateLCDMenu();
    }
    else if (msg.startsWith("OBS,")) {
      int firstComma = msg.indexOf(',');
      int secondComma = msg.indexOf(',', firstComma + 1);
      if (virtualObsCount < MAX_VIRTUAL_OBS) {
        virtualObsX[virtualObsCount] = msg.substring(firstComma + 1, secondComma).toFloat();
        virtualObsY[virtualObsCount] = msg.substring(secondComma + 1).toFloat();
        virtualObsCount++;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); 
  lcd.init(); lcd.backlight();
  WiFi.softAP(ssid, password);
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  if (!mcp.begin_I2C()) { while(1); }
  if (!mpu.begin()) { lcd.print("MPU NOT FOUND!"); while(1); }

  for(int i=1; i<=4; i++) {
    mcp.pinMode(PIN_MCP_IN1[i], OUTPUT); mcp.pinMode(PIN_MCP_IN2[i], OUTPUT);
    pinMode(PIN_PWM[i], OUTPUT);
    pinMode(PIN_ENC_A[i], INPUT_PULLUP); pinMode(PIN_ENC_B[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A[i]), (i==1?isr_M1:i==2?isr_M2:i==3?isr_M3:isr_M4), RISING);
  }
  
  pinMode(MUX_S0, OUTPUT); pinMode(MUX_S1, OUTPUT); pinMode(MUX_S2, OUTPUT); 
  pinMode(MUX_S3, OUTPUT); pinMode(TRIG_ALL_PIN, OUTPUT); pinMode(MUX_SIG_PIN, INPUT);
  mcp.pinMode(BTN_UP, INPUT_PULLUP); mcp.pinMode(BTN_DOWN, INPUT_PULLUP);
  mcp.pinMode(BTN_OK, INPUT_PULLUP); mcp.pinMode(BTN_BACK, INPUT_PULLUP);

  updateLCDMenu();
}

void loop() {
  webSocket.loop();
  handleButtons(); 

  if (state == RUNNING) {
    unsigned long now = millis();
    if (now - lastTime >= SAMPLING_MS) {
      float dt = (now - lastTime) / 1000.0;
      lastTime = now;
      travelTime = (now - startTimeRobot) / 1000.0;

      // 1. UPDATE HEADING (Tanpa Deadband agar tidak drift)
      sensors_event_t a, g, temp; 
      mpu.getEvent(&a, &g, &temp);
      float gyroZ = g.gyro.z - gyroZ_offset;
      currentHeading += (gyroZ * 180.0 / PI) * dt;

      // Konversi currentHeading (Derajat) menjadi Radian untuk trigonometri odometri
      posTheta = currentHeading * (PI / 180.0); 
      // -------------------------------------------------

      // 2. ODOMETRI
      float vWheel[5];
      for(int i=1; i<=4; i++) {
        long dC = encCount[i] - lastEncCount[i];
        lastEncCount[i] = encCount[i]; 
        vWheel[i] = (dC / PPR) * (2 * PI * R_RODA) / dt;
      }
      float vRobotX = (vWheel[1] - vWheel[2] - vWheel[3] + vWheel[4]) / 4.0 * sqrt(2);
      float vRobotY = -1.0 * (vWheel[1] + vWheel[2] + vWheel[3] + vWheel[4]) / 4.0 * sqrt(2);
       
      posX += ((vRobotX * cos(posTheta) - vRobotY * sin(posTheta)) * dt) * KOREKSI_X;
      posY += ((vRobotX * sin(posTheta) + vRobotY * cos(posTheta)) * dt) * KOREKSI_Y;

      // 3. APF CALCULATIONS
      float dx = targetX - posX, dy = targetY - posY;
      float distGoal = sqrt(dx*dx + dy*dy);
      float F_att_x = K_ATT * dx, F_att_y = -1.0 * K_ATT * dy;
      float F_rep_x = 0, F_rep_y = 0;
      bool isObstacle = false;

      // Sensor Fisik (LOGIKA BARU SESUAI KODING KEDUA)
      float readings[8];
      float closestDist = 999.0;

      // Baca semua sensor dulu dan cari yang paling dekat
      for (int i = 0; i < 8; i++) {
          readings[i] = readSingleSensor(i);
          if (readings[i] > 2.0 && readings[i] < closestDist) {
              closestDist = readings[i];
          }
      }

      bool panicMode = (closestDist < DIST_PANIC_CM);

      for (int i = 0; i < 8; i++) {
          float d_cm = readings[i];
          
          // Abaikan data sampah atau di luar jangkauan deteksi
          if (d_cm < 2.0 || d_cm > DIST_DETECT_CM) continue; 
          // Jika sedang panik, abaikan rintangan lain yang masih jauh
          if (panicMode && d_cm > (DIST_PANIC_CM + 10.0)) continue;

          isObstacle = true;

          // Konversi ke Meter KHUSUS UNTUK RUMUS GAYA agar K_REP 200 tetap kuat
          float d_meter = d_cm / 100.0;
          float detect_meter = DIST_DETECT_CM / 100.0;

          float magnitude = 0;
          if (d_cm < DIST_PANIC_CM) {
              magnitude = 250.0; // Panic Force Max
          } else {
              magnitude = K_REP * (1.0/d_meter - 1.0/detect_meter) / (d_meter * d_meter);
          }

          // --- TAMBAHAN BARU: Boost Gaya untuk Sensor Diagonal ---
          // i ganjil (1, 3, 5, 7) adalah sensor diagonal (45, 135, 225, 315)
          if (i % 2 != 0) { 
              magnitude *= 1.5; // Berikan gaya 1.5x lebih kuat agar tidak loyo
          }
          // --------------------------------------------------------

          // Batasi magnitude agar tidak merusak perhitungan (tetap pakai batas 400 sesuai koding pertama)
          if (magnitude > 400.0) magnitude = 400.0; 
          
          float angleSensor = i * (45.0 * PI / 180.0); 
          F_rep_x -= magnitude * sin(angleSensor); 
          F_rep_y -= magnitude * cos(angleSensor);
      }

      // Rintangan Virtual
      for (int i = 0; i < virtualObsCount; i++) {
        float dx_obs = posX - virtualObsX[i], dy_obs = posY - virtualObsY[i];
        float d_m = sqrt(dx_obs*dx_obs + dy_obs*dy_obs);
        if (d_m > 0.05 && d_m < DIST_DETECT_M) { 
          isObstacle = true;
          float magnitude = K_REP * (1.0/d_m - 1.0/DIST_DETECT_M) / (d_m * d_m);
          if (magnitude > 400.0) magnitude = 400.0; 
          float angleObs = atan2(dy_obs, dx_obs); 
          F_rep_x += magnitude * cos(angleObs); F_rep_y += magnitude * sin(angleObs);
        }
      }
      float Vx = F_att_x + F_rep_y, Vy = F_att_y + F_rep_x;

      // 4. PID HEADING (DENGAN WRAP-AROUND)
      float errorH = targetHeading - currentHeading;
      if (errorH > 180) errorH -= 360;
      if (errorH < -180) errorH += 360;
      if (abs(errorH) < 1.0) errorH = 0; 

      float W = (-KP_HEADING * errorH) * (PI / 180.0); 
      if (W > 1.5) W = 1.5; if (W < -1.5) W = -1.5; // Limit kembali ke 1.5 agar kuat

      // 5. FINISH & KINEMATIKA (MENGGUNAKAN RUMUS KODING 1)
      if (distGoal < 0.02) {
        stopMotors(); state = FINISHED; updateLCDMenu();
      } else {
        float magV = sqrt(Vx*Vx + Vy*Vy);
        float maxMps = (MAX_SPEED_RPM / 60.0) * (2*PI*R_RODA); 
        if (magV > maxMps) { Vx = (Vx / magV) * maxMps; Vy = (Vy / magV) * maxMps; }
        //if (distGoal < 0.15) W *= 0.3;

        float rpmTarget[5];
        // RUMUS KINEMATIKA DIKEMBALIKAN KE KODING 1
        rpmTarget[1] = ((Vy + Vx + W*LX_LY) / (2*PI*R_RODA)) * 60; 
        rpmTarget[2] = ((Vy - Vx - W*LX_LY) / (2*PI*R_RODA)) * 60; 
        rpmTarget[3] = ((Vy - Vx + W*LX_LY) / (2*PI*R_RODA)) * 60; 
        rpmTarget[4] = ((Vy + Vx - W*LX_LY) / (2*PI*R_RODA)) * 60; 
        for(int i=1; i<=4; i++) runOpenLoop(i, rpmTarget[i]);
      }

      // 6. KIRIM DATA
      String dataStr = String(posX) + "," + String(posY) + "," + String(currentPWM[1]) + "," + String(currentPWM[2]) + "," + String(currentPWM[3]) + "," + String(currentPWM[4]) + "," + String(travelTime) + "," + String(currentHeading);
      webSocket.broadcastTXT(dataStr);
    }
  }
}

// Fungsi bantu agar koding lebih rapi
void calibrateMPU() {
  lcd.clear(); lcd.print("CALIBRATING...");
  float sum = 0;
  for(int i=0; i<400; i++) { 
    sensors_event_t a, g, temp; mpu.getEvent(&a, &g, &temp); 
    sum += g.gyro.z; delay(3); 
  }
  gyroZ_offset = sum / 400.0;
}

float readSingleSensor(int ch) {
    digitalWrite(MUX_S0, (ch & 0x01) ? HIGH : LOW); 
    digitalWrite(MUX_S1, (ch & 0x02) ? HIGH : LOW);
    digitalWrite(MUX_S2, (ch & 0x04) ? HIGH : LOW); 
    digitalWrite(MUX_S3, (ch & 0x08) ? HIGH : LOW);
    delayMicroseconds(50); 
    digitalWrite(TRIG_ALL_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_ALL_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_ALL_PIN, LOW);

    // Timeout dipersingkat menjadi 10000 agar loop tidak lambat
    long duration = pulseIn(MUX_SIG_PIN, HIGH, 10000);
    
    delay(5);

    if (duration == 0) return 999.0; // Jika tidak ada halangan, kembalikan 999 CM
    return (duration * 0.034) / 2.0; // Return dalam bentuk Centimeter (CM)
}

void runOpenLoop(int id, float targetRPM) {
  // UBAH DI SINI: Kalikan target dengan faktor skala motor
   float balancedRPM = targetRPM * motorScale[id];
   int pwm = constrain(abs(targetRPM) / MAX_SPEED_RPM * 255.0, 0, 255);
   
   if (pwm > 0 && pwm < 100) pwm = 110; 
   currentPWM[id] = pwm;
   bool maju = (targetRPM >= 0);
   if (id == 1) { mcp.digitalWrite(0, maju?HIGH:LOW); mcp.digitalWrite(1, maju?LOW:HIGH); }
   else if (id == 2) { mcp.digitalWrite(2, maju?LOW:HIGH); mcp.digitalWrite(3, maju?HIGH:LOW); }
   else if (id == 3) { mcp.digitalWrite(4, maju?LOW:HIGH); mcp.digitalWrite(5, maju?HIGH:LOW); }
   else if (id == 4) { mcp.digitalWrite(6, maju?HIGH:LOW); mcp.digitalWrite(7, maju?LOW:HIGH); }
   analogWrite(PIN_PWM[id], pwm);
}

void stopMotors() {
  for(int i=1; i<=4; i++) { 
    analogWrite(PIN_PWM[i], 0); 
    mcp.digitalWrite(PIN_MCP_IN1[i], LOW); mcp.digitalWrite(PIN_MCP_IN2[i], LOW); 
    currentPWM[i] = 0;
  }
}

void handleButtons() {
  static unsigned long lastP = 0; if (millis()-lastP < 200) return;
  if(mcp.digitalRead(BTN_UP)==LOW){ if(state==SET_X) targetX+=0.3; else if(state==SET_Y) targetY+=0.3; updateLCDMenu(); lastP=millis(); }
  if(mcp.digitalRead(BTN_DOWN)==LOW){ if(state==SET_X) targetX-=0.3; else if(state==SET_Y) targetY-=0.3; updateLCDMenu(); lastP=millis(); }
  if(mcp.digitalRead(BTN_OK)==LOW){ 
    if(state==SET_X) state=SET_Y;
    else if(state==SET_Y) state=READY;
    else if(state==READY) {
      calibrateMPU();
      for(int i=1; i<=4; i++) lastEncCount[i] = encCount[i];
      currentHeading = 0; posX = 0; posY = 0; travelTime = 0;
      startTimeRobot = millis(); state = RUNNING; lastTime = millis();
    } else if(state==FINISHED) state=SET_X;
    updateLCDMenu(); lastP=millis(); 
  }
}

void updateLCDMenu() {
  lcd.clear();
  if (state == SET_X) { lcd.print("Set X: "); lcd.print(targetX); }
  else if (state == SET_Y) { lcd.print("Set Y: "); lcd.print(targetY); }
  else if (state == READY) { lcd.print("Ready? OK"); }
  else if (state == RUNNING) { lcd.setCursor(0,0); lcd.print("T:"); lcd.print(travelTime); lcd.setCursor(0,1); lcd.print("H:"); lcd.print(currentHeading); }
  else if (state == FINISHED) { lcd.print("Done! T:"); lcd.print(travelTime); }
}