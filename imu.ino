int pvYaw;
void heading(float spd,float spYaw){
  head_error = spYaw-(MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw));
  head_i=head_i+head_error;
  head_i=constrain(head_i,-180,180);
  if(head_i<-180)head_i=-180;
  else if(head_i>180)head_i=180;
  head_d=head_error-head_pError;
  head_w=(head_error*head_Kp)+(head_i*head_Ki)+(head_Kd);
  //head_w=contrain(head_w,-100,100);
  if(head_w<-100)head_w=-100;
  else if(head_w>100)head_w=100;
  MiniR4.M1.setSpeed(spd+head_w);
  MiniR4.M2.setSpeed(spd-head_w);
  head_pError=head_error;
}

void getb() {
   if (MiniR4.I2C1.MXLaser.begin()) {
    Serial.println("MXLaser init success");
  } else {
    Serial.println("MXLaser init failed");
  }

  while (true) {
    int dis = MiniR4.I2C1.MXLaser.getDistance();

    if (dis == 8191) {
      MiniR4.M1.setSpeed(-50);
      MiniR4.M2.setSpeed(-50);
    } else if (dis > 143) {
      MiniR4.M1.setSpeed(-30);
      MiniR4.M2.setSpeed(-30);
    } else if (dis < 130) {
      MiniR4.M1.setSpeed(30);
      MiniR4.M2.setSpeed(30);
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
      break;

    }
    delay(100);
  }
}
  

void getbbtn() {
  if (MiniR4.I2C1.MXLaser.begin()) {
    Serial.println("MXLaser init success");
  } else {
    Serial.println("MXLaser init failed");
  }

  while (MiniR4.PS2.Button(PSB_SQUARE)) {
    int dis = MiniR4.I2C1.MXLaser.getDistance();

    if (dis == 8191) {
      Serial.println("TIMEOUT");
      MiniR4.M1.setSpeed(-50);
      MiniR4.M2.setSpeed(-50);
    } else if (dis > 143) {
      MiniR4.M1.setSpeed(-30);
      MiniR4.M2.setSpeed(-30);
      Serial.print("Distance = ");
      Serial.print(dis);
      Serial.println("mm");
    } else if (dis < 130) {
      MiniR4.M1.setSpeed(30);
      MiniR4.M2.setSpeed(30);
      Serial.print("Distance = ");
      Serial.print(dis);
      Serial.println("mm");
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

    }
    delay(100);
  }
}

void encordimu(int a, float Kp, float Ki, float Kd) {
   MiniR4.M1.resetCounter();
  MiniR4.M2.resetCounter();
  while (true) {
    if (MiniR4.M1.getDegrees() < a && MiniR4.M2.getDegrees() < a) {
    float current_yaw = MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw);

    // Calculate error (target yaw is 0)
    float error = 0 - current_yaw;

    // If error is small enough, stop
    if (abs(error) < 3) {
      MiniR4.M1.setSpeed(-82);
      MiniR4.M2.setSpeed(-85);
      delay(10);
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
    } else {
      delay(20);
      break;
    }
  }
}
