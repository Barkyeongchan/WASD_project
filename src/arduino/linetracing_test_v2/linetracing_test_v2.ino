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
int   BASE_PWM     = 70;   // 초저속 시작
float KP            = 0.25; // P 게인
int   OBST_MM       = 180;  // 장애물 정지(mm)
int   IR_MIN        = 120;
int   IR_MAX        = 880;
bool  INVERT        = true; // 검정에서 값이 커지면 true

// ===== 복구 파라미터 =====
const uint16_t LOST_ARC_MS   = 400;   // 단계1: 전진-호 탐색 시간
const uint16_t LOST_SPIN_MS  = 600;   // 단계2: 제자리 선회 시간
const uint16_t LOST_ZIG_MS   = 800;   // 단계3: 지그재그 시간
const uint16_t LOST_TOTAL_MS = 2500;  // 최대 탐색시간(넘으면 정지)

const int ARC_FWD = 60;   // 단계1 전진 속도(아주 느림)
const int ARC_TURN= 38;   // 단계1 회전 성분(작게)
const int SPIN_SPD= 60;   // 단계2 제자리 회전 속도
const int ZIG_SPD = 45;   // 단계3 지그재그 속도

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

float norm(int raw){
  if(INVERT) raw = 1023 - raw;
  raw = clamp(raw, IR_MIN, IR_MAX - 1);
  return (float)(raw - IR_MIN) / (float)(IR_MAX - IR_MIN);
}

// LED 패턴
void led_following(){ digitalWrite(LED, HIGH); }
void led_searching_slow(){ static unsigned long t; if(millis()-t>300){ t=millis(); digitalWrite(LED, !digitalRead(LED)); } }
void led_searching_fast(){ static unsigned long t; if(millis()-t>120){ t=millis(); digitalWrite(LED, !digitalRead(LED)); } }
void led_obstacle(){ static unsigned long t; if(millis()-t>150){ t=millis(); digitalWrite(LED, !digitalRead(LED)); } }

int lastSign = 0;                 // 마지막 오차 부호(+우, -좌)
unsigned long lostStart = 0;      // 라인 상실 시작 시각(0이면 상실 아님)

// 동적 감속: |err| 클수록 속도 낮추기 (0~1 스케일의 감속계수)
int speedWithErr(int base, float err){
  float mag = fabs(err);              // 대략 0~2 범위
  float k = 1.0f - clamp((int)(mag*35), 0, 50)/100.0f; // err 클수록 0.5~1.0 사이로 감소
  int spd = (int)(base * k);
  return clamp(spd, 22, base);        // 최소 속도 22 보장
}

void setup(){
  pinMode(DIR1_A,OUTPUT); pinMode(PWM1_A,OUTPUT);
  pinMode(DIR2_A,OUTPUT); pinMode(PWM2_A,OUTPUT);
  pinMode(DIR1_B,OUTPUT); pinMode(PWM1_B,OUTPUT);
  pinMode(DIR2_B,OUTPUT); pinMode(PWM2_B,OUTPUT);
  pinMode(TRIG,OUTPUT); pinMode(ECHO,INPUT);
  pinMode(LED, OUTPUT);
  stopAll();
  digitalWrite(LED, LOW);
}

void loop(){
  // 1) 장애물 우선
  long dmm = readUS();
  if(dmm>0 && dmm <= OBST_MM){
    stopAll();
    led_obstacle();
    delay(20);
    return;
  }

  // 2) IR 읽기/정규화
  int raw[5]; for(int i=0;i<5;i++) raw[i]=analogRead(IR[i]);
  float n[5]; for(int i=0;i<5;i++) n[i]=norm(raw[i]);

  // 포화값 영향 축소
  for(int i=0;i<5;i++){
    if(n[i] < 0.02f || n[i] > 0.98f) n[i] *= 0.25f;
  }

  // 라인 감지 개수
  int onCnt=0; for(int i=0;i<5;i++) if(n[i]>0.5f) onCnt++;

  // 3) 라인 상실 복구 로직
  if(onCnt==0){
    if(lostStart==0) lostStart = millis();   // 상실 시작 기록
    unsigned long elapsed = millis() - lostStart;

    if(elapsed <= LOST_ARC_MS){
      // 단계1: 최근 방향으로 아주 느리게 전진하며 호(arc) 탐색
      int turn = (lastSign>=0)? ARC_TURN : -ARC_TURN;
      drive(clamp(ARC_FWD - turn, -255, 255),
            clamp(ARC_FWD + turn, -255, 255));
      led_searching_slow();
    }
    else if(elapsed <= LOST_ARC_MS + LOST_SPIN_MS){
      // 단계2: 제자리 선회
      int spin = (lastSign>=0)? SPIN_SPD : -SPIN_SPD;
      drive(-spin, +spin);
      led_searching_fast();
    }
    else if(elapsed <= LOST_ARC_MS + LOST_SPIN_MS + LOST_ZIG_MS){
      // 단계3: 지그재그 (좌우 짧게 번갈아)
      unsigned long t = (elapsed - (LOST_ARC_MS + LOST_SPIN_MS));
      bool leftPhase = (t/250)%2==0; // 250ms 간격
      int s = ZIG_SPD;
      if(leftPhase) drive(+s, -s); else drive(-s, +s);
      led_searching_fast();
    }
    else{
      // 타임아웃: 안전 정지
      stopAll();
      digitalWrite(LED, LOW);
    }

    delay(20);
    return;
  }
  else{
    // 라인 재획득 → 타이머 리셋
    lostStart = 0;
  }

  // 4) 위치/오차 계산
  float w[5] = {-2,-1,0,1,2};
  float sum = 1e-6, acc = 0;
  for(int i=0;i<5;i++){ sum += n[i]; acc += w[i]*n[i]; }
  float pos = acc / sum;   // -2..+2
  float err = -pos;        // 필요 시 부호 반전은 여기서

  lastSign = (err>0)?1:((err<0)?-1:lastSign);

  // 5) 동적 감속 + P 제어
  int base = speedWithErr(BASE_PWM, err);
  int corr = (int)(KP * err * 255.0);
  int L = clamp(base - corr, 0, 255);
  int R = clamp(base + corr, 0, 255);
  drive(L, R);

  // 6) 상태 LED
  led_following();
  delay(15);
}
