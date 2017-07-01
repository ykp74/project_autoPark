//ykp74 Arduino function lib
#define BUZZER 10   //fixed buzzer gpio
#define ON 1
#define OFF 0
#define BLINK 2

typedef struct _toneDB{
	int hz;
	int duration;
	int count;
}_toneDB;

enum{
	START_TONE,
	ERROR_TONE,
	SUCCESS_TONE,
	ALERT_TONE,
	MAX_TONE
};

_toneDB sound[MAX_TONE] = { 
	{ 1000, 100, 1 },
	{ 1000, 100, 3 },
	{ 1000, 100, 2 },
	{ 1000, 50, 5 },
};

void make_tone( int type ){
  int i = 0;
  pinMode(BUZZER, OUTPUT);

  for(i=0;i<sound[type].count;i++){
       tone(BUZZER, sound[type].hz ,sound[type].duration); //1kHz Sound
       delay(sound[type].duration);
       noTone(BUZZER);
       delay(sound[type].duration);
  }
}

void start_tone(){
    make_tone(START_TONE);
}

void led_ctl(int port, int enable ){
	int i = 0;
	
	pinMode(port, OUTPUT);

	if( enable == BLINK){
		for( i = 0; i < 2; i++){
			digitalWrite( port,1);
			delay(50);
			digitalWrite( port,0);
			delay(50);
		}
		return;
	}
	digitalWrite( port, enable);
}
#if 0
int get_distance_sonic_front(){
  int duration = 0;
  int distance = 0;
  int array[2] = {0,};
  int index = 0;
  int tmp = 0;

  Serial.print("get_distance_sonic_front : ");
  array[0] = 0;
  array[1] = 0;

  pinMode(SENSOR_TRIG_FRONT, OUTPUT);
  pinMode(SENSOR_ECHO_FRONT, INPUT);
  
  while( 1 ){
      //init
      digitalWrite(SENSOR_TRIG_FRONT, 0);
      digitalWrite(SENSOR_ECHO_FRONT, 0);
      delayMicroseconds(2);
    
      //set triger to go
      digitalWrite(SENSOR_TRIG_FRONT, 1);
      delayMicroseconds(10);
      digitalWrite(SENSOR_TRIG_FRONT, 0);
    
       //get the time
      duration = pulseIn(SENSOR_ECHO_FRONT, HIGH);
      distance = duration / 29 / 2;

      delayMicroseconds(10);
      if( distance > 0 ){
        array[index] = distance;
        index++;
      }
      if( index == 2 ){
        tmp = abs(  array[0] - array[1]  );

        if( tmp <= 2 ){
          break;
        } else {
          index = 0;
        }
      } 
  }
  Serial.println(array[0]);
  return array[0];
}
#endif