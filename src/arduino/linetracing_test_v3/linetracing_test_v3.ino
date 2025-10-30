// ===== 모터 드라이버 핀 =====
const int DIR1_A=22, PWM1_A=9;    // Left group
const int DIR2_A=23, PWM2_A=10;
const int DIR1_B=28, PWM1_B=11;   // Right group
const int DIR2_B=29, PWM2_B=12;

// ===== 센서 핀 =====
const int IR[5] = {A0, A1, A2, A3, A4}; // 왼→오
const int TRIG=26, ECHO=27;             // 초음파

// ===== LED =====
const int LED = 13;

// ===== 기본 파라미터 =====
int   BASE_PWM     = 60;   // 초저속
float KP            = 0.18; // 코너에서 조금 더 과감히
int   OBST_MM       = 140;  // 오검지 줄이려고 140mm로 완화
int   IR_MIN        = 120;
int   IR_MAX        = 880;
bool  INVERT        = true; // 검정에서 값이 커지면 true

// ===== 코너/복구 파라미터(업데이트) =====
const int   PIVOT_SPD       = 55;   // 피벗 회전 속도 ↑
const int   PIVOT_SPD_SOFT  = 50;   // 초반 살살
const int   PIVOT_TIMEOUT   = 1400; // 피벗 최대 유지 시간 ↑
const float EDGE_HI         = 0.75; // 끝 센서 강검출 기준
const float MID_LO          = 0.35; // 중앙 약 기준
const float CENTER_OK       = 0.45; // 중앙 재획득 기준 완화
const uint16_t LOST_TOTAL_MS= 2800; // 상실 총 타임아웃 살짝 ↑
const int   PIVOT_CREEP     = 20;   // ★ 피벗 중 전진 크리프

// ===== 유틸 =====
int clamp(int v,int lo,int hi){ return v<lo?lo:(v>hi?hi:v); }

void setSide(int dir1,int pwm1,int dir2,int pwm2,int pwm,bool forward=true){
  int mag = abs(pwm);
  bool dir = (pwm>=0) ? forward : !forward;
  digitalWrite(dir1, dir); digitalWrite(dir2, dir);
  analogWrite(pwm1, clamp(mag,0,255));
  analogWrite(pwm2, clamp(mag,0,255));
}
void drive(int L,int R){
  setSide(DIR1_A,PWM1_A,DIR2_A,PWM2_A,L,true);
  setSide(DIR1_B,PWM1_B,DIR2_B,PWM2_B,R,true);
}
void stopAll(){ drive(0,0); }

long readUS(){
  digitalWrite(TRIG,LOW); delayMicroseconds(2);
  digitalWrite(TRIG,HIGH); delayMicroseconds(10);
  digitalWrite(TRIG,LOW);
  long us = pulseIn(ECHO,HIGH,30000);
  if(!us) return -1;
  return (long)(us*0.343/2.0); // mm
}

float normOne(int raw){
  if(INVERT) raw = 1023 - raw;
  raw = clamp(raw, IR_MIN, IR_MAX - 1);
  return (float)(raw - IR_MIN) / (float)(IR_MAX - IR_MIN);
}

// LED 패턴
void led_following(){ digitalWrite(LED, HIGH); }
void led_searching_slow(){ static unsigned long t; if(millis()-t>300){ t=millis(); digitalWrite(LED, !digitalRead(LED)); } }
void led_searching_fast(){ static unsigned long t; if(millis()-t>120){ t=millis(); digitalWrite(LED, !digitalRead(LED)); } }
void led_obstacle(){ static unsigned long t; if(millis()-t>150){ t=millis(); digitalWrite(LED, !digitalRead(LED)); } }

// 상태머신
enum Mode { FOLLOW, CORNER_LEFT, CORNER_RIGHT, LOST_SEARCH };
Mode mode = FOLLOW;
unsigned long modeStart = 0, lostStart = 0;
int lastSign = 0;

// 감속: |err| 클수록 더 느리게
int speedWithErr(int base, float err){
  float mag = fabs(err);
  float k = 1.0f - clamp((int)(mag*40), 0, 55)/100.0f; // 0.45~1.0
  int spd = (int)(base * k);
  return clamp(spd, 22, base);
}

// 코너 감지/재획득
bool isLeftCorner(float n0,float n1,float n2,float n3,float n4){
  float midAvg = (n1 + n2 + n3) / 3.0f;
  return (n0 > EDGE_HI) && (midAvg < MID_LO);
}
bool isRightCorner(float n0,float n1,float n2,float n3,float n4){
  float midAvg = (n1 + n2 + n3) / 3.0f;
  return (n4 > EDGE_HI) && (midAvg < MID_LO);
}
bool centerAcquired(float n2, int onCnt, float err){
  return (n2 > CENTER_OK) || (onCnt >= 2 && fabs(err) < 0.5f);
}

void setup(){
  pinMode(DIR1_A,OUTPUT); pinMode(PWM1_A,OUTPUT);
  pinMode(DIR2_A,OUTPUT); pinMode(PWM2_A,OUTPUT);
  pinMode(DIR1_B,OUTPUT); pinMode(PWM1_B,OUTPUT);
  pinMode(DIR2_B,OUTPUT); pinMode(PWM2_B,OUTPUT);
  pinMode(TRIG,OUTPUT); pinMode(ECHO,INPUT);
  pinMode(LED, OUTPUT);
  stopAll(); digitalWrite(LED, LOW);
  mode = FOLLOW; modeStart = millis();
}

void loop(){
  // 안전(초음파)
  long dmm = readUS();
  if(dmm>0 && dmm <= OBST_MM){ stopAll(); led_obstacle(); delay(20); return; }

  // IR 읽기/정규화
  int raw[5]; for(int i=0;i<5;i++) raw[i]=analogRead(IR[i]);
  float n[5]; for(int i=0;i<5;i++) n[i]=normOne(raw[i]);
  for(int i=0;i<5;i++){ if(n[i] < 0.02f || n[i] > 0.98f) n[i] *= 0.25f; }

  int onCnt=0; for(int i=0;i<5;i++) if(n[i]>0.5f) onCnt++;

  float w[5] = {-2,-1,0,1,2};
  float sum = 1e-6, acc = 0;
  for(int i=0;i<5;i++){ sum+=n[i]; acc+=w[i]*n[i]; }
  float pos = acc / sum;           // -2..+2
  float err = -pos;                // 필요 시 부호 조정
  if(err>0) lastSign=+1; else if(err<0) lastSign=-1;

  switch(mode){
    case FOLLOW: {
      if(isLeftCorner(n[0],n[1],n[2],n[3],n[4]))  { mode=CORNER_LEFT;  modeStart=millis(); break; }
      if(isRightCorner(n[0],n[1],n[2],n[3],n[4])) { mode=CORNER_RIGHT; modeStart=millis(); break; }
      if(onCnt==0){ mode=LOST_SEARCH; modeStart=lostStart=millis(); break; }

      int base = speedWithErr(BASE_PWM, err);
      int corr = (int)(KP * err * 255.0);
      int L = clamp(base - corr, 0, 255);
      int R = clamp(base + corr, 0, 255);
      drive(L,R); led_following();
      break;
    }

    case CORNER_LEFT: {
      unsigned long el = millis() - modeStart;
      int spd = (el < 200) ? PIVOT_SPD_SOFT : PIVOT_SPD;
      // ★ 전진 크리프 넣은 좌측 피벗
      drive(-spd + PIVOT_CREEP, +spd + PIVOT_CREEP);

      if(centerAcquired(n[2], onCnt, err)){ mode=FOLLOW; modeStart=millis(); break; }
      if(n[4] > EDGE_HI){ mode=FOLLOW; modeStart=millis(); break; }
      if(el > PIVOT_TIMEOUT){ mode=LOST_SEARCH; modeStart=lostStart=millis(); break; }
      led_searching_fast(); break;
    }

    case CORNER_RIGHT: {
      unsigned long el = millis() - modeStart;
      int spd = (el < 200) ? PIVOT_SPD_SOFT : PIVOT_SPD;
      // ★ 전진 크리프 넣은 우측 피벗
      drive(+spd + PIVOT_CREEP, -spd + PIVOT_CREEP);

      if(centerAcquired(n[2], onCnt, err)){ mode=FOLLOW; modeStart=millis(); break; }
      if(n[0] > EDGE_HI){ mode=FOLLOW; modeStart=millis(); break; }
      if(el > PIVOT_TIMEOUT){ mode=LOST_SEARCH; modeStart=lostStart=millis(); break; }
      led_searching_fast(); break;
    }

    case LOST_SEARCH: {
      unsigned long el = millis() - lostStart;
      if(onCnt>0){ mode=FOLLOW; modeStart=millis(); break; }

      if(el <= 800){
        // 최근 부호로 아주 느린 전진-호 탐색
        int turn = (lastSign>=0)? 28 : -28;
        int fwd  = 26;
        drive(clamp(fwd - turn, -255, 255),
              clamp(fwd + turn, -255, 255));
        led_searching_slow();
      } else if(el <= 1600){
        // 제자리 선회
        int spin = (lastSign>=0)? 52 : -52;
        drive(-spin, +spin);
        led_searching_fast();
      } else if(el <= LOST_TOTAL_MS){
        // 지그재그
        bool leftPhase = ((el-1600)/250)%2==0;
        int s = 44;
        if(leftPhase) drive(+s, -s); else drive(-s, +s);
        led_searching_fast();
      } else {
        // ★ 완전 정지 대신 초저속 제자리 선회로 계속 탐색
        int spin = (lastSign>=0)? 40 : -40;
        drive(-spin, +spin);
        led_searching_slow();
      }
      break;
    }
  }

  delay(12);
}
