#define rmf 3
#define rmb 4
#define lmf 6
#define lmb 5

#define rm_pwm 2
#define lm_pwm 7

#define irs0 A10
#define irs1 A9
#define irs2 A11
#define irs3 A12
#define irs4 A13
#define irs5 A14
#define irs6 A8
#define irs7 A15

int i = 0; // i - line(white) , j - no line(black)
int j = 1; // to test code in white background and black lines
int end_point = 0;

int ir_array[8] = {0 ,0, 0, 0, 0, 0, 0, 0};
int error_arr[8] = {-20, -6.8, -2.4, -1,1, 2.4, 6.8, 20};
char path_array[5] = {};
int path_length;
int read_length;

int max_speed = 100;
int mid_speed_R = 45;
int mid_speed_L = 65;
int lms = mid_speed_L;
int rms = mid_speed_R;

float P , I , D , error;
float prev_error = 0;
float Kp = 0.75;
float Ki = 0;
float Kd = 9.5;

  
void setup() {
  Serial.begin(9600);
  pinMode(lmf, OUTPUT);
  pinMode(lmb, OUTPUT);
  pinMode(rmf, OUTPUT);
  pinMode(rmb, OUTPUT);
  pinMode(lm_pwm, OUTPUT);
  pinMode(rm_pwm, OUTPUT);
  pinMode(irs0, INPUT);
  pinMode(irs1, INPUT);
  pinMode(irs2, INPUT);
  pinMode(irs3, INPUT);
  pinMode(irs4, INPUT);
  pinMode(irs5, INPUT);
  pinMode(irs6, INPUT);
  pinMode(irs7, INPUT);
 

  motor_speed();
  forward();
  delay(500);

  
  
}

void loop() {
  stop();
  delay(0.1);
  
  read_sensors();  
  PID_control();
  forward();
  motor_speed();
  Serial.println("following line");
  
  //about turn
  if ((ir_array[0]==i) && (ir_array[2]==i) && (ir_array[2]==i) && (ir_array[2]==i) && (ir_array[3]==i) && (ir_array[4]==i) && (ir_array[5]==i)&&(ir_array[7]==i)){
    stop();
    delay(150);
    turn_right();
    rms = 80;
    lms = 90;
    motor_speed();
    delay(600);
    stop();
    delay(1000);
    forward();
    rms=70;
    lms=80;
    motor_speed();
    delay(200);
    read_sensors();
  }
  if (((ir_array[0] == 1)&&((ir_array[1] == 1)||(ir_array[2] == 1)||(ir_array[3] == 1)))  || ((ir_array[7] == 1)&&((ir_array[4] == 1)||(ir_array[5] == 1)||(ir_array[6] == 1)))){
    junction();
    }
 
}

void forward() {
  digitalWrite(rmf, HIGH);
  digitalWrite(rmb, LOW);
  digitalWrite(lmf, HIGH);
  digitalWrite(lmb, LOW);
}

void turn_right() {
  digitalWrite(lmf, HIGH);
  digitalWrite(lmb, LOW);
  digitalWrite(rmf, LOW);
  digitalWrite(rmb, HIGH);
}

void turn_left() {
  digitalWrite(lmf, LOW);
  digitalWrite(lmb, HIGH);
  digitalWrite(rmf, HIGH);
  digitalWrite(rmb, LOW);
}

void stop () {
  digitalWrite(rmf, LOW);
  digitalWrite(rmb, LOW);
  digitalWrite(lmf, LOW);
  digitalWrite(lmb, LOW);
}

void junction() //at a cross
{
  Serial.println("junction");
 
  stop();
  delay(500);
  read_sensors();
  
  
  int right_path = 0;
  int left_path = 0;
  int straight_path = 0;

  if (((ir_array[0] == j)||(ir_array[1] == j)) && ((ir_array[7]==i) || (ir_array[6] == i))){    
    left_path = 1;
    right_path = 0;
    }
    
  if (((ir_array[0] == j)||(ir_array[1] == j)) && ((ir_array[7] == j) || (ir_array[6] == j))){
    left_path = 1;
    right_path = 0;
    }
    
  if (((ir_array[0] == i)||(ir_array[1] == i)) && ((ir_array[7]==j) || (ir_array[6] == j))){
    right_path = 1;
    left_path = 0;
    }
    
    forward();
    rms=80;
    lms=90;
    motor_speed();
    delay(200);

   read_sensors();
   if ((ir_array[0]==j) && (ir_array[1]==j) && (ir_array[2]==j) && (ir_array[3]==j) && (ir_array[4]==j) && (ir_array[5]==j) && (ir_array[6]==j)&&(ir_array[7]==i)){
    stop();
    delay(1000);
    end_point = 1;
    Serial.println("Finished");
    Serial.println(path_array);
    if (end_point == 1)
      {
      turn_right();
      rms = 80;
      lms = 90;
      motor_speed();
      delay(565);
      /*while(true)
        {
        turn_right();
        rms=40;
        lms=40;
        motor_speed();
        read_sensors();
        if((ir_array[2]==j)|| (ir_array[3]==j) || (ir_array[4]==j)){
          stop();
          delay(100);
          break;
        }*/
      stop();
      delay(500);
      forward();
      rms = 80;
      lms = 90;
      motor_speed();
      delay(100);
      
      while (path_length == -1)
        {
        read_sensors();  
        PID_control();
        forward();
        motor_speed();
        if ((ir_array[0] == 1) || (ir_array[7] == 1))
          {
          stop();
          delay(500);
          if (path_array[path_length] == 'D')
            {
              path_length = path_length - 1;
            }
          turning();
          path_length = path_length - 1;
          }
        }
      end_point = 0;
      }
      stop();
      while(1)
      {

      }

   }
   
  if ((ir_array[1]==j) || (ir_array[2]==j) || (ir_array[3]==j) || (ir_array[4]==j) || (ir_array[5]==j)||(ir_array[6]==j)) {
   straight_path = 1;
   }
  
  if (left_path == 1){
    path_length++;
    if (path_array[path_length-2] == 'B'){
      short_path();
    }
    turn_left();
    rms=70;
    lms=60;
    motor_speed();
    delay(50);

    while(true){
      turn_left();
      rms=60;
      lms=40;
      motor_speed();
      read_sensors();
      if((ir_array[2]==j)|| (ir_array[3]==j) || (ir_array[4]==j) || (ir_array[5]==j)){
        stop();
        delay(500);
        break;
      }
    }
    Serial.println("left");
    path_array[path_length] = 'L';
    prt_arr();
    forward();
    rms = 70;
    lms = 70;
    motor_speed();
    delay(50);
    }
    

   else if (straight_path == 1){
    path_length++;
    if (path_array[path_length-2] == 'B'){
      short_path();
    }
    Serial.println("Straight");
    path_array[path_length] = 'S';
    prt_arr();
    forward();
    rms=80;
    lms=90;
    motor_speed();
    delay(100);
    }

  else if (right_path == 1){    
    path_length++;
    if (path_array[path_length-2] == 'B'){
      short_path();
    }
    
    turn_right();
    rms=80;
    lms=80;
    motor_speed();
    while(true){
      turn_right();
      rms=40;
      lms=90;
      motor_speed();
      read_sensors();
      if((ir_array[2]==j)|| (ir_array[3]==j) || (ir_array[4]==j) || (ir_array[5]==j)){
        stop();
        delay(500);
        break;
      }
    }
    Serial.println("Right");
    path_array[path_length] = 'R';
    prt_arr();
    forward();
    rms = 70;
    lms = 70;
    motor_speed();
    delay(50);
    
    }
    
    
    
}


void motor_speed() {
  analogWrite(rm_pwm, rms);
  analogWrite(lm_pwm, lms);
  }
  
void PID_control() {
  error = 0 ;

  for (int a=0; a<=7; a++) 
    {
    error = error + ir_array[a]*error_arr[a];
    }

  P = error;
  I = I + error;
  D = error - prev_error;
  prev_error = error;

  float PID_value = (P*Kp + I*Ki + D*Kd);
  int PID_value_int = round(PID_value);

  rms = mid_speed_R - PID_value;
  lms = mid_speed_L + PID_value;



  if (rms < 0)         {rms = 0; }
  if (rms > max_speed) {rms = max_speed; }
  if (lms < 0)          {lms = 0; }
  if (lms > max_speed)  {lms = max_speed; }
 
  }

 void read_sensors(){
 ir_array[0] = !digitalRead(irs0);
//  if (ir_array[0] == 0 ) {ir_array[0] = 1; }
//  else {ir_array[0] = 0 ;}
  
  ir_array[1] = !digitalRead(irs1);
//  if (ir_array[1] == 0 ) {ir_array[1] = 1; }
//  else {ir_array[1] = 0 ;}
  
  ir_array[2] = !digitalRead(irs2);
//  if (ir_array[2] == 0 ) {ir_array[2] = 1; }
//  else {ir_array[2] = 0 ;}
  
  ir_array[3] = !digitalRead(irs3);
//  if (ir_array[3] == 0 ) {ir_array[3] = 1; }
//  else {ir_array[3] = 0 ;}
  
  ir_array[4] = !digitalRead(irs4);
//  if (ir_array[4] == 0 ) {ir_array[4] = 1; }
//  else {ir_array[4] = 0 ;}
  
  ir_array[5] = !digitalRead(irs5);
//  if (ir_array[5] == 0 ) {ir_array[5] = 1; }
//  else {ir_array[5] = 0 ;}

  ir_array[6] = !digitalRead(irs6);
//  if (ir_array[6] == 0 ) {ir_array[6] = 1; }
//  else {ir_array[6] = 0 ;}
ir_array[7] = !digitalRead(irs7);
//  if (ir_array[6] == 0 ) {ir_array[6] = 1; }
//  else {ir_array[6] = 0 ;}
 }


void short_path(){
  
  int shortDone = 0;

  if ((path_array[path_length-3] == 'L') && (path_array[path_length-1] == 'R')){
    path_length = path_length - 3;
    path_array[path_length] = 'B';
    shortDone = 1;
  }

  if ((path_array[path_length-3] == 'L') && (path_array[path_length-1] == 'S') && (shortDone == 0)){
    path_length = path_length - 3;
    path_array[path_length] = 'R';
    shortDone = 1;
  }

  if ((path_array[path_length-3] == 'R') && (path_array[path_length-1] == 'L') && (shortDone == 0)){
    path_length = path_length - 3;
    path_array[path_length] = 'B';
    shortDone = 1;
  }

  if ((path_array[path_length-3] == 'S') && (path_array[path_length-1] == 'L') && (shortDone == 0)){
    path_length = path_length - 3;
    path_array[path_length] = 'R';
    shortDone = 1;
  }

  if ((path_array[path_length-3] == 'S') && (path_array[path_length-1] == 'S') && (shortDone == 0)){
    path_length = path_length - 3;
    path_array[path_length] = 'B';
    shortDone = 1;
  }

  if ((path_array[path_length-3] == 'L') && (path_array[path_length-1] == 'L') && (shortDone == 0)) {
    path_length = path_length - 3;
    path_array[path_length] = 'S';
    shortDone = 1;
  }
  path_array[path_length+1] = 'D';
  path_array[path_length+2] = 'D';
  path_length++;

}

void turning()
{
  if (path_array[path_length] == 'R')
          {
          turn_left();
          rms=80;
          lms=90;
          motor_speed();
          delay(300);

          while(true)
            {
            turn_left();
            rms=40;
            lms=45;
            motor_speed();
            read_sensors();
            if((ir_array[2]==j)|| (ir_array[3]==j) || (ir_array[4]==j)||(ir_array[5]==j))
              {
              stop();
              delay(100);
              break;
              }
            }
        
          }
        else if (path_array[path_length] == 'S')
          {
          forward();
          rms=90;
          lms=60;
          motor_speed();
          delay(100);
          }

        else if (path_array[path_length] == 'L')
          {
          turn_right();
          rms=80;
          lms=80;
          motor_speed();
          delay(300);

          while(true)
            {
            turn_right();
            rms=40;
            lms=40;
            motor_speed();
            read_sensors();
            if((ir_array[2]==j)|| (ir_array[3]==j) || (ir_array[4]==j)){
              stop();
              delay(100);
              break;
              }
            }
        
          }  

}

void prt_arr(){

  int a = 0;
  while (a < path_length)
    {
      Serial.print(path_array[a]);
      a = a+1;
    }
  Serial.println();
}
