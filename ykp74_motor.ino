// Prototyping 과정
// Auto Parking system and Remote control with BT module
// SW : 박영기 책임 (System SW 개발2그룹/무선)
// HW : 성민철 책임 (PC개발그룹/무선)
// 2016.6.17

#include <SoftwareSerial.h>
#include <Servo.h>
#include <ykpFunction.h>

//grobal
//#define USE_SOFT_SREIAL_PORT
#define DGB_MSG(a) Serial.print("LOG : "); Serial.println(a)
 
//Output indicator
#define OUTPUT_LED  13
#define OUTPUT_SPK  10      //PWM only

//Sonic Sensor
#define SENSOR_TRIG 11
//#define SENSOR_ECHO 10    //Input Only
#define SENSOR_ECHO 13      //Input Only

#define SENSOR_TRIG_SIDE 12
#define SENSOR_ECHO_SIDE 8    //Input Only

#define SENSOR_TRIG_FRONT 2
#define SENSOR_ECHO_FRONT 5   //Input Only

//BT
#define TX      5
#define RX      2

//servo motor
#define SERVO_PIN     9  //PMW only
#define INIT_DGR      20
#define RIGHT         (INIT_DGR + 19)
#define LEFT          (INIT_DGR - 19)
#define HALF_RIGHT    (INIT_DGR + 10)
#define HALF_LEFT     (INIT_DGR - 10)

//main motor
#define MOTOR_A_1A    3   //PWM only
#define MOTOR_A_1B    4  
#define MOTOR_B_1A    6   //PWM only
#define MOTOR_B_1B    7

#define SPEED       255
#define LOW_SPEED   100

Servo servo;
#ifdef USE_SOFT_SREIAL_PORT
SoftwareSerial BTSerial( TX,RX );
#endif

enum{
  DIR_INIT = 0,
  DIR_LEFT,
  DIR_HALF_LEFT,
  DIR_RIGHT,
  DIR_HALF_RIGHT
};

enum {
  IDLE_S = 0,
  STOP_S = 0,
  //FORWORD_S,
  //REVERS_S,
  
  T_PARKING_S,
      T_IDLE,
      T_DONE,

  P_COURSE_S,
      P_IDLE,
      P_FORWARD,
      P_BUS_DETECT_DONE,
      P_REVERS,
      P_REVERS_2,
      P_ADJUST,
      P_ADJUST_2,
      P_DONE,
};

int device_state = STOP_S;
int p_course_state = P_IDLE;
int t_course_state = T_IDLE;

void motor_forward( int spd){
  analogWrite( MOTOR_A_1A, spd);
  digitalWrite( MOTOR_A_1B, 0 );
  analogWrite( MOTOR_B_1A, 255 - spd);
  digitalWrite( MOTOR_B_1B, 1 );
  //device_state = FORWORD_S;
  led_ctl( OUTPUT_LED, ON );
  Serial.println("motor_forward ");
}

void motor_reverse( int spd ){
  analogWrite( MOTOR_A_1A, 255 - spd);
  digitalWrite( MOTOR_A_1B, 1 );
  analogWrite( MOTOR_B_1A, spd);
  digitalWrite( MOTOR_B_1B, 0 );
  //device_state = REVERS_S;
  led_ctl( OUTPUT_LED, ON );
  Serial.println("motor_reverse");
}

void motor_stop(){
  analogWrite( MOTOR_A_1A, 0);
  digitalWrite( MOTOR_A_1B, 0 );
  analogWrite( MOTOR_B_1A, 0);
  digitalWrite( MOTOR_B_1B, 0 );
  //device_state = STOP_S;
  led_ctl( OUTPUT_LED, OFF );
  Serial.println("motor_stop");
}

void direction_ctrl( int dir ){
  switch( dir ){
    case DIR_INIT:
      servo.write(INIT_DGR+2);  //서보 지터노이즈 개선을 위함
      servo.write(INIT_DGR);
      break;    
    
    case DIR_LEFT:
      servo.write(INIT_DGR);  //서보 지터노이즈 개선을 위함
      delay(20);
      servo.write(LEFT);
      break;

    case DIR_HALF_LEFT:
      servo.write(INIT_DGR);  //서보 지터노이즈 개선을 위함
      delay(20);
      servo.write(HALF_LEFT);
      break;
 
    case DIR_RIGHT:
      servo.write(INIT_DGR);  //서보 지터노이즈 개선을 위함
      delay(20);
      servo.write(RIGHT);
      break;

    case DIR_HALF_RIGHT:
      servo.write(INIT_DGR);  //서보 지터노이즈 개선을 위함
      delay(20);
      servo.write(HALF_RIGHT);
      break;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// 초음파 센서 거리계산
/////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int get_distance_sonic_rear()
{
  unsigned int tmp_value[5];
  unsigned int min = 10000;
  unsigned int max = 0;  unsigned int avg = 0;  unsigned int ret = 0;
  
  Serial.print("get_distance_sonic_rear : ");
  
  pinMode(SENSOR_TRIG, OUTPUT);
  pinMode(SENSOR_ECHO, INPUT);
 
  for(int i=0; i<5; i++)
  {
    digitalWrite(SENSOR_TRIG, LOW);
    digitalWrite(SENSOR_ECHO, LOW);
    delayMicroseconds(2);
    digitalWrite(SENSOR_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(SENSOR_TRIG, LOW);

    unsigned long duration = pulseIn (SENSOR_ECHO, HIGH);
    tmp_value[i] = duration / 29 / 2;
    
    if (tmp_value[i] < min)
      min = tmp_value[i];
    if (tmp_value[i] > max)
      max = tmp_value[i];
    avg += tmp_value[i];
    delay(10);
  }
  ret = ((avg - min - max) / 3);
  
  Serial.println(ret);
  return ret;
}

unsigned int get_distance_sonic_front()
{
  unsigned int tmp_value[5];
  unsigned int min = 10000;
  unsigned int max = 0;  unsigned int avg = 0;  unsigned int ret = 0;
  
  Serial.print("get_distance_sonic_front : ");
  
  pinMode(SENSOR_TRIG_FRONT, OUTPUT);
  pinMode(SENSOR_ECHO_FRONT, INPUT);
 
  for(int i=0; i<5; i++)
  {
    digitalWrite(SENSOR_TRIG_FRONT, LOW);
    digitalWrite(SENSOR_ECHO_FRONT, LOW);
    delayMicroseconds(2);
    digitalWrite(SENSOR_TRIG_FRONT, HIGH);
    delayMicroseconds(10);
    digitalWrite(SENSOR_TRIG_FRONT, LOW);

    unsigned long duration = pulseIn (SENSOR_ECHO_FRONT, HIGH);
    tmp_value[i] = duration / 29 / 2;
    
    if (tmp_value[i] < min)
      min = tmp_value[i];
    if (tmp_value[i] > max)
      max = tmp_value[i];
    avg += tmp_value[i];
    delay(10);
  }
  ret = ((avg - min - max) / 3);
  
  Serial.println(ret);
  return ret;
}

unsigned int get_distance_sonic_side()
{
  unsigned int tmp_value[5];
  unsigned int min = 10000;
  unsigned int max = 0;  unsigned int avg = 0;  unsigned int ret = 0;
  
  Serial.print("get_distance_sonic_side : ");
  
  pinMode(SENSOR_TRIG_SIDE, OUTPUT);
  pinMode(SENSOR_ECHO_SIDE, INPUT);
 
  for(int i=0; i<5; i++)
  {
    digitalWrite(SENSOR_TRIG_SIDE, LOW);
    digitalWrite(SENSOR_ECHO_SIDE, LOW);
    delayMicroseconds(2);
    digitalWrite(SENSOR_TRIG_SIDE, HIGH);
    delayMicroseconds(10);
    digitalWrite(SENSOR_TRIG_SIDE, LOW);

    unsigned long duration = pulseIn (SENSOR_ECHO_SIDE, HIGH);
    tmp_value[i] = duration / 29 / 2;
    
    if (tmp_value[i] < min)
      min = tmp_value[i];
    if (tmp_value[i] > max)
      max = tmp_value[i];
    avg += tmp_value[i];
    delay(10);
  }
  ret = ((avg - min - max) / 3);
  
  Serial.println(ret);
  return ret;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// T 주차 실행  (순차 처리방식 이용)
/////////////////////////////////////////////////////////////////////////////////////////////////
void t_corse_run(){
  int side_bus = 0;
  int delta1 = 0;
  int before = 0;
  int detect = 0;
  int cnt = 0;

  //Start motor 출발
  motor_forward(SPEED);

  //사이드센서를 이용하여 두번째 버스까지 체크함.
  while( side_bus <= 1 ){
      delay(20);
      delta1 = get_distance_sonic_side();
      if(  delta1 < 3){  // 노이즈에 의한 0 - 3값은 쓰레기값으로 버림 안정화 처리
        continue;  
      }
      
      if( delta1 < 20 ){
         detect = 1;
         cnt++;
      } else {
         detect = 0;
         cnt = 0;
      }

      if( before == 0 && detect == 1 && cnt > 5){
         side_bus += 1;
         before = 1;
      } else if (  before == 1 && detect == 0 ){
         //side_bus += 1;
         before = 0;
         cnt = 0;
      }
      Serial.println(side_bus );
  }
  
  direction_ctrl( DIR_LEFT );
  delay(500);
  motor_stop();
  make_tone(SUCCESS_TONE);
  delay(1000);

  direction_ctrl( DIR_INIT );
  motor_reverse(LOW_SPEED);  //후진후
  delay(100);
  direction_ctrl( DIR_INIT );
  delay(300);
  direction_ctrl( DIR_RIGHT );

  //후방 거리가 점점 가까워 지면 핸들를 풀고
  while(get_distance_sonic_rear() > 10 ){
    ;
  }
  direction_ctrl( DIR_INIT );

  //후방 거리가 뒤에 붙으면 가까워지면 엔진 스톱
  while(get_distance_sonic_rear() > 3 ){
    ;
  }
  motor_stop();
  delay(1000);

  //보정작업 사이드 센서
  motor_forward(SPEED);
  while(get_distance_sonic_side()  > 15 ){
    ;
  }
  direction_ctrl( DIR_HALF_LEFT);
  delay(200);
  motor_stop();
  delay(100);
  motor_reverse(LOW_SPEED);  //후진후
  direction_ctrl( DIR_INIT); 

   //후방 거리가 뒤에 붙으면 가까워지면 엔진 스톱
  while(get_distance_sonic_rear() > 3 ){
    ;
  }
  motor_stop();
  delay(1000);

  //주차완료
  make_tone(ALERT_TONE);
  device_state = IDLE_S;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// 평행 주차 실행   (스테이트 머신 처리방식 이용)
/////////////////////////////////////////////////////////////////////////////////////////////////
int sd = 0, fd = 0, rd = 0;
int onBus1 = 0, onBus2 = 0;
int passBus1 = 0;

void p_init(){
  onBus1 = 0;
  onBus2 = 0;
  passBus1 = 0;

  sd = 0;
  fd = 0;
  rd = 0;
  p_course_state = P_IDLE;
}

void p_course_run()
{
  switch(  p_course_state ){
    case P_IDLE:
      motor_forward(SPEED);
      p_course_state = P_FORWARD;
      break;

    case P_FORWARD:
      sd = get_distance_sonic_side();
      if(  sd < 20 ){  //사이드센서 값이 20이내이면 첫번째 버스 인식
        Serial.println("onBus1!!");
        onBus1 = 1;
      }

      if( onBus1 && sd > 20 ){  //센서값이 멀어지면 첫번쨰 버스는 지나침
        Serial.println("passBus1!!");
        passBus1 = 1;
      }

      if( passBus1 && sd < 20 ){  //두번째 버스가 인식확인됨 다음루틴 준비
        motor_forward(SPEED);
        delay(500);
        motor_stop();
        p_course_state = P_BUS_DETECT_DONE;
        make_tone(SUCCESS_TONE);
        Serial.println("P_BUS_DETECT_DONE!!");
        break;
      }
      break;

   case P_BUS_DETECT_DONE:
      direction_ctrl( DIR_RIGHT );  //오른쪽
      motor_reverse(LOW_SPEED);     //후진 이동으로 오른쪽 뒤로 이동함.
 
      if (get_distance_sonic_side() < 20){
        onBus2 = 1;
        Serial.println("onBus2!!");
      }

      //버스와 가이드 사이의 구명을 사이드센서가 체크하면 일단 진입성공
      if( onBus2 == 1 &&  get_distance_sonic_side() > 100 ){
         delay(300);
         direction_ctrl( DIR_LEFT );
         motor_stop();
         make_tone(SUCCESS_TONE);
         p_course_state = P_REVERS;
      }
      break;

    case P_REVERS:
      motor_reverse(LOW_SPEED);  //후진 뒷쪽 센서가 뒷가이드랑 붙을때까지 후진
      if (get_distance_sonic_rear() < 5){
          motor_stop();
          direction_ctrl( DIR_INIT );
          make_tone(SUCCESS_TONE);
          p_course_state = P_ADJUST;
      }
      break;
  
    case P_ADJUST:   //각도 보정함..
          direction_ctrl( DIR_HALF_RIGHT );
          motor_forward(LOW_SPEED);
          if (get_distance_sonic_front() < 10){
            direction_ctrl( DIR_INIT );
            motor_stop();
            p_course_state = P_DONE; //파킹 스테이스이동 
          }
         break;

    case P_REVERS_2:  //자세보정을 위한 후진2
      motor_reverse(LOW_SPEED);  //후진
      if (get_distance_sonic_rear() < 5){
          motor_stop();
          direction_ctrl( DIR_INIT );
          p_course_state = P_ADJUST_2;
      }
      break;
 
    case P_ADJUST_2:  //자세보정2
         direction_ctrl( DIR_LEFT );
         motor_forward(LOW_SPEED);  
         if (get_distance_sonic_front() < 5){
            direction_ctrl( DIR_LEFT );
            motor_stop();
            p_course_state = P_REVERS_2;
         }
         break;

    case P_DONE:
         fd = get_distance_sonic_front();
         rd = get_distance_sonic_rear();

         //앞뒤거리가 같아지도록 모터를 조정함
         if (abs( fd - rd ) >5){
            if( fd > rd){
              motor_forward(LOW_SPEED);  
            } else {
              motor_reverse(LOW_SPEED);
            }
         } else {
             motor_stop();
             make_tone(ALERT_TONE);
             device_state = IDLE_S;
         }
         break;
  }
}

void init_device(){
  //setup the main motor
  pinMode(MOTOR_A_1A, OUTPUT);
  pinMode(MOTOR_A_1B, OUTPUT);
  pinMode(MOTOR_B_1A, OUTPUT);
  pinMode(MOTOR_B_1B, OUTPUT);

  pinMode(0, INPUT);
  pinMode(1, OUTPUT);
 
  //servo
  servo.attach(SERVO_PIN);
  direction_ctrl( DIR_INIT );

  //LED and SPK Output
  pinMode(OUTPUT_LED, OUTPUT);

  start_tone();
  Serial.println("Init Device success!!");
}

void setup() {
  // put your setup code here, to run once:
#ifdef USE_SOFT_SREIAL_PORT
  BTSerial.begin(9600);
#endif
  Serial.begin(9600);

  /*  Initinal the RC Car Device */
  init_device();
}

void loop() {
  // put your main code here, to run repeatedly:
  int input = 0;

   //standby ouput
  led_ctl( OUTPUT_LED, BLINK );
  
  //use state machine
  if( device_state == P_COURSE_S ){
    p_course_run();
  }
  
#ifdef USE_SOFT_SREIAL_PORT
  if( BTSerial.available() > 0  )
#else
  if( Serial.available() > 0  )
#endif
  {
#ifdef USE_SOFT_SREIAL_PORT
    input = BTSerial.read();
#else
    input = Serial.read();
#endif
    led_ctl( OUTPUT_LED, ON ); 
    
    switch( input ){
       case '1':
            //device_state =  T_COURSE_S;  //no used state machine
            t_corse_run(); 
            Serial.println("T Course run!!");
            break;
       
       case '2':  
            p_init();
            device_state =  P_COURSE_S;
            Serial.println("P Course run!!");
            break;
       
       case 's': 
            (void)get_distance_sonic_side(); 
            (void)get_distance_sonic_front();
            (void)get_distance_sonic_rear();
            break;
       
       case '6':  motor_forward(SPEED);  break;
       case '7':  motor_reverse(SPEED);  break;
       case '0':  motor_stop(); break;
       
       case 'f':  direction_ctrl( DIR_INIT ); break;
       case 'r':  direction_ctrl( DIR_RIGHT ); break;
       case 'l':  direction_ctrl( DIR_LEFT );  break;
       case 'n':  direction_ctrl( DIR_HALF_RIGHT ); break;
       case 'm':  direction_ctrl( DIR_HALF_LEFT );  break;
 
       default:
             Serial.println( "InValid input !! : " );
             break;
    }
  }
}
