#include <MatrixMiniR4.h>
int i;
int PWR_LY;  // Power for left joystick (LY axis)
int STR_LX;  // Steering for left joystick (LX axis)
int PWR_RY;  // Power for right joystick (RY axis)
int STR_RX;  // Steering for right joystick (RX axis)
#define head_Kp 1.8f
#define head_Kd 1.0f
#define head_Ki 0.0f
float head_error, head_pError, head_w, head_d, head_i;
float angle;
float angle2;

float previous_error = 0.0;
float integral = 0.0;
unsigned long previous_time = 0;
const float MIN_SPEED =22.0;

void setup() {
  MiniR4.begin();
  Serial.begin(115200);
  angle = 100;
  angle2 = 0;
  MiniR4.RC1.setAngle(angle);
  MiniR4.RC3.setAngle(angle2);
  start();
  MiniR4.Motion.resetIMUValues();
  //  delay(500);

  ////////////////////////////////////JOY
  MiniR4.PWR.setBattCell(2);
  MiniR4.M1.setReverse(true);   // กำหนดทิศทางของมอเตอร์ M1
  MiniR4.M2.setReverse(false);  // กำหนดทิศทางของมอเตอร์ M2
  MiniR4.M1.setSpeed(0);        // ตั้งความเร็วของมอเตอร์ M1 เป็น 0
  MiniR4.M2.setSpeed(0);        // ตั้งความเร็วของมอเตอร์ M2 เป็น 0

  delay(500);


  while (true) {
     MiniR4.PS2.read_gamepad(false, 0);
  if (MiniR4.PS2.Button(PSB_TRIANGLE)) {
     MiniR4.Motion.resetIMUValues();
     resetPID(); 
     MiniR4.I2C1.MXLaser.begin();
     tone();
    //  encordimu(230, 0.1, 1, 1);
    //  backencordd(320);
    //  encordimu(230, 0.1, 1, 1);
     
      // getb();
      // MiniR4.RC1.setAngle(25);
      // delay(500);
      // MiniR4.RC3.setAngle(180);
      // delay(500);
      // angle2 = 180;
      // angle = 90;
    //TaskI2CLaser();

    // fwimu(130);
    // delay(500);
    // rimu(70);
    // MiniR4.RC1.setAngle(20);
    //  fwimu(350);
    //  MiniR4.RC1.setAngle(180);
    break;
  }
  }
  
  // angle = 80;
  // delay(700);


  //////////////////////////////////JOY
  // fwimu(); delay(1000);
  // rimu(); delay(1000);
  // fwimu(); delay(1000);
  // rimu(); delay(1000);
  // fwimu(); delay(1000);
  // rimu(); delay(1000);
  // fwimu(); delay(1000);
  // rimu(); delay(1000);
  // fwimu(); delay(1000);
  // rimu(); delay(1000);
  // fw();
  // delay(1000);
  // rr();




  // encordd(); delay(500);
  // backencordd(); delay(500);
  // encordd(); delay(500);
  // backencordd(); delay(500);
  // encordd(); delay(500);
  // backencordd(); delay(500);
  // encordd(); delay(500);
  // backencordd(); delay(500);
  // encordd(); delay(500);
  // backencordd(); delay(500);

  // square();
  //Tasksquare();
  //fw();delay(200);
  //rr();delay(200);
}
void loop() {
  // TaskGrayscale();
  //TaskPowerView();
  //fwimupid(5, 0.5, 1, 1);
  //TaskIMU();
  //heading(5, 0);
  //TaskIMU();
  //TaskI2CLaser();
  /////////////////////////////////////////////////JOYY
  ///*
 // อ่านข้อมูลจาก PS2 controller
  MiniR4.PS2.read_gamepad(false, 0);  // อ่านข้อมูลจาก PS2 controller
  if (MiniR4.PS2.Button(PSB_L2)) {
    if (angle > 6) {
      angle = angle + -5;
      delay(2);
    }
  }
  if (MiniR4.PS2.Button(PSB_L1)) {
    if (angle < 100) {
      angle = angle + 5;
      delay(2);
    } else {angle = 100;}
  }
  

if (MiniR4.PS2.Button(PSB_R2)) {

      angle2 = 180;
      delay(2);
    
  }
  if (MiniR4.PS2.Button(PSB_R1)) {

      angle2 = 0;
      delay(2);
  }

  if (MiniR4.PS2.Button(PSB_CIRCLE)) {
      MiniR4.RC1.setAngle(15);
      delay(500);
      MiniR4.RC3.setAngle(0);
      delay(500);
      MiniR4.RC1.setAngle(100);
      delay(500);
      angle2 = 180;
      delay(2);
      angle2 = 0;
      delay(2);
  }
  if (MiniR4.PS2.Button(PSB_CROSS)) {
    int dis = MiniR4.I2C1.MXLaser.getDistance();
    if (dis == 8191) {
      MiniR4.M1.setSpeed(-60);
      MiniR4.M2.setSpeed(-60);
    } else if (dis > 143) {
      MiniR4.M1.setSpeed(-40);
      MiniR4.M2.setSpeed(-40);
    } else if (dis < 130) {
      MiniR4.M1.setSpeed(40);
      MiniR4.M2.setSpeed(40);
    } else {
      MiniR4.M1.setSpeed(0);
      MiniR4.M2.setSpeed(0);
      delay(100);
    }
  }
  
  if (MiniR4.PS2.Button(PSB_SQUARE)) {
    int dis = MiniR4.I2C1.MXLaser.getDistance();
    if (dis == 8191) {
      MiniR4.M1.setSpeed(-90);
      MiniR4.M2.setSpeed(-90);
    } else if (dis > 143) {
      MiniR4.M1.setSpeed(-70);
      MiniR4.M2.setSpeed(-70);
    } else if (dis < 130) {
      MiniR4.M1.setSpeed(70);
      MiniR4.M2.setSpeed(70);
    } else {
      MiniR4.M1.setSpeed(0);
      MiniR4.M2.setSpeed(0);
      MiniR4.M1.setBrake(true);
      MiniR4.M2.setBrake(true);
      delay(100);
      MiniR4.RC1.setAngle(23);
      delay(500);
      MiniR4.RC3.setAngle(180);
      delay(500);
      angle2 = 180;
      angle = 90;

    }
  }
    if (MiniR4.PS2.Button(PSB_PAD_RIGHT)) {
    MiniR4.M1.setSpeed(-30);  // มอเตอร์ M1
    MiniR4.M2.setSpeed(30);

  }
    if (MiniR4.PS2.Button(PSB_PAD_LEFT)) {
    MiniR4.M1.setSpeed(30);  // มอเตอร์ M1
    MiniR4.M2.setSpeed(-30);

  }
  if (MiniR4.PS2.Button(PSB_PAD_UP)) {
    MiniR4.M1.setSpeed(-40);  // มอเตอร์ M1
    MiniR4.M2.setSpeed(-40);

  }
    if (MiniR4.PS2.Button(PSB_PAD_DOWN)) {
    MiniR4.M1.setSpeed(30);  // มอเตอร์ M1
    MiniR4.M2.setSpeed(30);

  }



  // อ่านค่า Analog จาก Joystick ซ้าย
  PWR_LY = map(MiniR4.PS2.Analog(PSS_LY), 0, 255, -100, 100);  // แกน Y ของ Joystick ซ้าย (พลังงาน)

  // // อ่านค่า Analog จาก Joystick ขวา
  PWR_RY = map(MiniR4.PS2.Analog(PSS_RY), 0, 255, -100, 100);  // แกน Y ของ Joystick ขวา (พลังงาน)

  // การใช้ปุ่ม L1 และ R1


  // ควบคุมมอเตอร์ตามค่า PWR และ STR
  MiniR4.RC1.setAngle(angle);
  MiniR4.RC3.setAngle(angle2);
  
  MiniR4.M1.setSpeed((PWR_LY)*2);  // มอเตอร์ M1
  MiniR4.M2.setSpeed((PWR_RY)*2);  // มอเตอร์ M2

  delay(10);  // หน่วงเวลาเล็กน้อย
  //*/

  ///////////////////////////////////////////////////JOYYYYY
  // TaskIMU();
  // delay(1);
  // MiniR4.M1.setSpeed(50); delay(1000);
  // MiniR4.M1.setSpeed(0); delay(500);
  // MiniR4.M1.setSpeed(-50); delay(1000);
  // MiniR4.M1.setSpeed(0); delay(500);
  // if (MiniR4.BTN_UP.getState()) {
  //   MiniR4.LED.setColor(1, 255, 0, 0);  // Red
  //   MiniR4.LED.setColor(2, 0, 0, 255);  // Blue
  // }
  // if (MiniR4.BTN_DOWN.getState()) {
  //   MiniR4.LED.setColor(1, 0, 0, 255);  // Blue
  //   MiniR4.LED.setColor(2, 255, 0, 0);  // Red
  // }
}

void start() {
  while (true) {
    if (MiniR4.BTN_UP.getState()) {
      MiniR4.Buzzer.Tone(NOTE_C4, 1000);
      delay(200);
      MiniR4.Buzzer.NoTone();
      delay(50);
      MiniR4.Buzzer.Tone(NOTE_C4, 1000);
      delay(200);
      MiniR4.Buzzer.NoTone();
      delay(50);
      MiniR4.Buzzer.Tone(NOTE_C4, 1000);
      delay(200);
      MiniR4.Buzzer.NoTone();
      delay(500);
      break;
    } else {
      delay(50);
    }
  }
}

void TaskEncoder() {
  MiniR4.M1.resetCounter();
  MiniR4.M2.resetCounter();

  while (true) {
    Serial.print("M1: ");
    Serial.print(MiniR4.M1.getDegrees());
    Serial.print(", M2: ");
    Serial.print(MiniR4.M2.getDegrees());


    MiniR4.M1.setSpeed(50);
    MiniR4.M2.setSpeed(50);
    delay(1500);

    delay(500);
    MiniR4.M1.setBrake(true);
    MiniR4.M2.setBrake(true);
    delay(5000);
  }
}

void fw() {
  MiniR4.M1.setSpeed(50);
  MiniR4.M2.setSpeed(-50);
  delay(1300);
  MiniR4.M1.setSpeed(0);
  MiniR4.M2.setSpeed(0);
  delay(300);
  MiniR4.M1.setBrake(true);
  MiniR4.M2.setBrake(true);
  delay(1000);
  delay(1000);
}

void bk(int a) {
  MiniR4.M1.setSpeed(50);
  MiniR4.M2.setSpeed(50);
  delay(a);
}

void rr() {
  delay(650);
  MiniR4.M1.setSpeed(70);
  MiniR4.M2.setSpeed(70);
  delay(350);
  MiniR4.M1.setSpeed(0);
  MiniR4.M2.setSpeed(0);
  delay(600);
  MiniR4.M1.setBrake(true);
  MiniR4.M2.setBrake(true);
  delay(1000);
  delay(1000);
}

void ll(int a) {
  MiniR4.M3.setSpeed(50);
  MiniR4.M2.setSpeed(-50);
  delay(a);
}

void square() {
  fw();
  rr();
  fw();
  rr();
  fw();
  rr();
  fw();
  rr();
  delay(500);
  MiniR4.M1.setSpeed(0);
  MiniR4.M2.setSpeed(0);
}

void encord(int a) {
  MiniR4.M1.resetCounter();
  MiniR4.M2.resetCounter();
  while (true) {
    if (MiniR4.M1.getDegrees() < a && MiniR4.M2.getDegrees() < a) {
      MiniR4.M1.setSpeed(-40);
      MiniR4.M2.setSpeed(-40);
      delay(100);
    } else {
      MiniR4.M1.setBrake(true);
      MiniR4.M2.setBrake(true);
      delay(200);
      break;
    }
  }
}

void encordd(int a) {
  MiniR4.M1.resetCounter();
  MiniR4.M2.resetCounter();
  while (true) {
    if (MiniR4.M1.getDegrees() < a && MiniR4.M2.getDegrees() < a) {
      MiniR4.M1.setSpeed(-40);
      MiniR4.M2.setSpeed(-40);
      delay(100);
    } else {
      delay(20);
      break;
    }
  }
}

void backencordd(int a) {
  MiniR4.M1.resetCounter();
  MiniR4.M2.resetCounter();
  while (true) {
    if (MiniR4.M1.getDegrees() < a && MiniR4.M2.getDegrees() < a) {
      MiniR4.M1.setSpeed(-50);
      MiniR4.M2.setSpeed(50);
      delay(100);
    } else {
      MiniR4.M1.setSpeed(0);
      MiniR4.M2.setSpeed(0);
      delay(100);
      break;
    }
  }
}

void TaskIMU() {
  int roll = MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Roll);
  int pitch = MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Pitch);
  int yaw = MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw);

  char buff[80];
  sprintf(buff, "Euler: r=%3d, p=%3d, y=%3d\t\n", roll, pitch, yaw);
  Serial.println(buff);

  delay(100);
}

void rimu(int a) {
  MiniR4.Motion.resetIMUValues();
  while (true) {
    TaskIMU();
    if (MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw) > a * -1) {
      MiniR4.M1.setSpeed(-8);
      MiniR4.M2.setSpeed(8);
      delay(5);
    } else {
      MiniR4.M1.setSpeed(0);
      MiniR4.M2.setSpeed(0);
      delay(100);
      break;
    }
  }
}

void limu(int a) {
  MiniR4.Motion.resetIMUValues();
  while (true) {
    TaskIMU();
    if (MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw) > a) {
      MiniR4.M1.setSpeed(8);
      MiniR4.M2.setSpeed(-8);
      delay(5);
    } else {
      MiniR4.M1.setSpeed(0);
      MiniR4.M2.setSpeed(0);
      delay(100);
      break;
    }
  }
}

void fwimu(int b) {
  MiniR4.Motion.resetIMUValues();
  while (true) {
    if (2 > MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw) > -2) {
      encordd(b);
      delay(500);
      break;
    }
    if (2 < MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw) > -2) {
      MiniR4.M1.setSpeed(-8);
      MiniR4.M2.setSpeed(8);
      delay(5);
    }
    if (2 > MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw) < -2) {
      MiniR4.M1.setSpeed(8);
      MiniR4.M2.setSpeed(-8);
      delay(5);
    }
  }
}

void TaskI2CLaser() {
  if (MiniR4.I2C1.MXLaser.begin()) {
    Serial.println("MXLaser init success");
  } else {
    Serial.println("MXLaser init failed");
  }

  while (true) {
    int dis = MiniR4.I2C1.MXLaser.getDistance();

    if (dis == 8191) {
      Serial.println("TIMEOUT");
    } else {
      Serial.print("Distance = ");
      Serial.print(dis);
      Serial.println("mm");
    }
    delay(100);
  }
}


void TaskI2CMotion() {
  if (MiniR4.I2C4.MXMotion.begin()) {
    Serial.println("MXMotion init success");
  } else {
    Serial.println("MXMotion init failed");
  }

  while (true) {
    int roll = MiniR4.I2C2.MXMotion.getRoll();
    int pitch = MiniR4.I2C2.MXMotion.getPitch();
    int yaw = MiniR4.I2C2.MXMotion.getYaw();

    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print(" , Pitch: ");
    Serial.print(pitch);
    Serial.print(" , Yaw: ");
    Serial.println(yaw);

    delay(100);
  }
}

void resetPID() {
  previous_error = 0.0;
  integral = 0.0;
  previous_time = millis();
}

// ฟังก์ชันคำนวณค่า PID
float calculatePID(float current_error, float Kp, float Ki, float Kd) {
  unsigned long current_time = millis();
  float delta_time = (current_time - previous_time) / 1000.0;  // Time difference in seconds

  // Proportional term
  float proportional = Kp * current_error;

  // Integral term
  integral += current_error * delta_time;
  float integral_term = Ki * integral;

  // Derivative term
  float derivative = (current_error - previous_error) / delta_time;
  float derivative_term = Kd * derivative;

  // Store the current error for the next cycle
  previous_error = current_error;
  previous_time = current_time;

  // Return the combined PID output
  return proportional + integral_term + derivative_term;
}

void fwimupid(int b, float Kp, float Ki, float Kd) {

  resetPID();  // Reset PID calculations at the start

  while (true) {
    // Get the current yaw value from the IMU (assuming yaw is in degrees)
    float current_yaw = MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw);

    // Calculate error (target yaw is 0)
    float error = 0 - current_yaw;

    // If error is small enough, stop
    if (abs(error) < 2) {
      MiniR4.M1.setSpeed(-85);
      MiniR4.M2.setSpeed(-85);
      delay(b);
    }

    // Use the PID controller to calculate the speed adjustments
    float pid_output = calculatePID(error, Kp, Ki, Kd);
/*()
    if (abs(pid_output) < MIN_SPEED) {
      pid_output = (pid_output > 0) ? MIN_SPEED : -MIN_SPEED;  // กำหนดให้มอเตอร์หมุนด้วยความเร็วขั้นต่ำ
    }
*/
    // Based on the PID output, adjust the motor speeds
    // Positive error means turn counter-clockwise, negative error means turn clockwise
    MiniR4.M1.setSpeed(pid_output);   // Negative speed for left motor
    MiniR4.M2.setSpeed(-pid_output);  // Positive speed for right motor

    delay(5);  // Small delay to allow motor adjustments
  }
}

void TaskPowerView() {
  float voltage = MiniR4.PWR.getBattVoltage();
  float percentage = MiniR4.PWR.getBattPercentage();

  MiniR4.OLED.clearDisplay();
  MiniR4.OLED.setCursor(5, 5);
  MiniR4.OLED.print("Vol:" + String(voltage) + "V");
  MiniR4.OLED.setCursor(5, 20);
  MiniR4.OLED.print("Per:" + String(percentage) + "%");
  MiniR4.OLED.display();

  delay(100);
}

void tone() {
  while (true) {
      MiniR4.Buzzer.Tone(NOTE_C4, 1000);
      delay(100);
      MiniR4.Buzzer.NoTone();
      delay(50);
      MiniR4.Buzzer.Tone(NOTE_C4, 1000);
      delay(100);
      MiniR4.Buzzer.NoTone();
      delay(50);
      MiniR4.Buzzer.Tone(NOTE_C4, 1000);
      delay(100);
      MiniR4.Buzzer.NoTone();
      delay(50);
      break;
  }
}
void TaskGrayscale() {
  int AI = MiniR4.A1.getAIL();  // Analog read
  int DI = MiniR4.A1.getR();    // Digital read

  char buff[50];
  sprintf(buff, "AI: %d, DI: %d", AI, DI);
  Serial.println(buff);

  delay(100);
}