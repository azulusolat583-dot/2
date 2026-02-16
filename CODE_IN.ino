#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

const uint8_t RF_PIN_CE   = 9;
const uint8_t RF_PIN_CSN  = 3;
const uint8_t PIN_TILT    = 7;
const uint8_t PIN_PAN     = 6;
const uint8_t PIN_BEAM    = 5;
const uint8_t PIN_STOP    = 4;

RF24 link(RF_PIN_CE, RF_PIN_CSN);
Servo tiltSrv;
Servo panSrv;

const byte HUB_PIPE[6]   = "BASE1";
const byte NODE_PIPE[6]  = "CUBE1";
const uint8_t UNIT_ID    = 1;

const uint16_t BEAM_TIME   = 2500;
const uint16_t MOVE_PAUSE  = 400;

enum RcCmd : uint8_t {
  RC_NOP = 0,
  RC_ALL,
  RC_LASER,
  RC_SERVO,
  RC_RADIO,
  RC_MCU,
  RC_SCAN_H,
  RC_SCAN_V,
  RC_SCAN_D1,
  RC_SCAN_D2,
  RC_ZERO,
  RC_SCAN_FULL
};

enum RcMode : uint8_t {
  RM_IDLE = 0,
  RM_WAIT,
  RM_SH,
  RM_SV,
  RM_SD1,
  RM_SD2,
  RM_DIAG
};

enum RcTest : uint8_t {
  RT_NOP = 0,
  RT_LASER,
  RT_SERVOS,
  RT_RADIO,
  RT_MCU,
  RT_ALL
};

struct RcCmdFrame {
  uint8_t dev;
  RcCmd   cmd;
};

struct RcTelFrame {
  uint8_t dev;
  int8_t  tilt;
  int8_t  pan;
  RcMode  mode;
};

struct RcDiagFrame {
  uint8_t dev;
  RcTest  test;
  bool    start;
  bool    end;
  bool    ok;
};

int8_t tiltDegNow = 0;
int8_t panDegNow  = 0;
RcMode modeNow    = RM_IDLE;

int toServoVal(int8_t d) {
  if (d < -40) d = -40;
  if (d >  40) d = 40;
  return map(d, -40, 40, 50, 130);
}

bool stopActive() {
  return digitalRead(PIN_STOP) == LOW;
}

void movePlatform(int8_t tTarget, int8_t pTarget, uint16_t stepMs = 20) {
  int tDst = constrain(tTarget, -40, 40);
  int pDst = constrain(pTarget, -40, 40);

  int tStart = tiltDegNow;
  int pStart = panDegNow;

  int diffT = abs(tDst - tStart);
  int diffP = abs(pDst - pStart);
  int steps = max(diffT, diffP);

  if (steps <= 0) {
    tiltSrv.write(toServoVal(tDst));
    panSrv.write(toServoVal(pDst));
    tiltDegNow = tDst;
    panDegNow  = pDst;
    return;
  }

  for (int i = 1; i <= steps; ++i) {
    if (stopActive()) break;
    int tCur = tStart + (tDst - tStart) * i / steps;
    int pCur = pStart + (pDst - pStart) * i / steps;
    tiltSrv.write(toServoVal(tCur));
    panSrv.write(toServoVal(pCur));
    delay(stepMs);
  }

  tiltDegNow = tDst;
  panDegNow  = pDst;
}

void sendState() {
  RcTelFrame f;
  f.dev  = UNIT_ID;
  f.tilt = -tiltDegNow;
  f.pan  =  panDegNow;
  f.mode = modeNow;

  link.stopListening();
  link.openWritingPipe(HUB_PIPE);
  link.write(&f, sizeof(f));
  link.startListening();
}

void sendDiag(RcTest t, bool st, bool en, bool ok) {
  RcDiagFrame f;
  f.dev   = UNIT_ID;
  f.test  = t;
  f.start = st;
  f.end   = en;
  f.ok    = ok;

  link.stopListening();
  link.openWritingPipe(HUB_PIPE);
  link.write(&f, sizeof(f));
  link.startListening();
}

void diagLaser() {
  modeNow = RM_DIAG;
  sendDiag(RT_LASER, true, false, true);

  for (int i = 0; i < 3; ++i) {
    if (stopActive()) {
      sendDiag(RT_LASER, false, true, false);
      return;
    }
    digitalWrite(PIN_BEAM, HIGH);
    delay(300);
    digitalWrite(PIN_BEAM, LOW);
    delay(300);
  }

  sendDiag(RT_LASER, false, true, true);
}

void diagServos() {
  modeNow = RM_DIAG;
  sendDiag(RT_SERVOS, true, false, true);

  movePlatform(0, 0, 10);
  delay(300);
  if (stopActive()) { sendDiag(RT_SERVOS, false, true, false); return; }

  movePlatform(-40, -40, 10);
  delay(500);
  if (stopActive()) { sendDiag(RT_SERVOS, false, true, false); return; }

  movePlatform(40, 40, 10);
  delay(500);
  if (stopActive()) { sendDiag(RT_SERVOS, false, true, false); return; }

  movePlatform(0, 0, 10);
  delay(300);
  sendDiag(RT_SERVOS, false, true, true);
}

void diagRadio() {
  modeNow = RM_DIAG;
  sendDiag(RT_RADIO, true, false, true);

  for (int i = 0; i < 3; ++i) {
    if (stopActive()) {
      sendDiag(RT_RADIO, false, true, false);
      return;
    }
    sendState();
    delay(500);
  }

  sendDiag(RT_RADIO, false, true, true);
}

void diagMCU() {
  modeNow = RM_DIAG;
  sendDiag(RT_MCU, true, false, true);

  for (int i = 0; i < 2; ++i) {
    if (stopActive()) {
      sendDiag(RT_MCU, false, true, false);
      return;
    }
    digitalWrite(PIN_BEAM, HIGH);
    delay(100);
    digitalWrite(PIN_BEAM, LOW);
    delay(100);
  }

  sendDiag(RT_MCU, false, true, true);
}

void diagAll() {
  modeNow = RM_DIAG;
  sendDiag(RT_ALL, true, false, true);

  diagLaser();
  diagServos();
  diagRadio();
  diagMCU();

  sendDiag(RT_ALL, false, true, true);
}

void patternH() {
  modeNow = RM_SH;
  for (int t = 40; t >= -40; t -= 10) {
    if (stopActive()) break;
    movePlatform(t, 0);
    delay(100);
    digitalWrite(PIN_BEAM, HIGH);
    sendState();
    delay(BEAM_TIME);
    digitalWrite(PIN_BEAM, LOW);
    delay(MOVE_PAUSE);
  }
}

void patternV() {
  modeNow = RM_SV;
  for (int p = -40; p <= 40; p += 10) {
    if (stopActive()) break;
    movePlatform(0, p);
    delay(100);
    digitalWrite(PIN_BEAM, HIGH);
    sendState();
    delay(BEAM_TIME);
    digitalWrite(PIN_BEAM, LOW);
    delay(MOVE_PAUSE);
  }
}

void patternD1() {
  modeNow = RM_SD1;
  for (int a = -40; a <= 40; a += 10) {
    if (stopActive()) break;
    movePlatform(-a, a);
    delay(100);
    digitalWrite(PIN_BEAM, HIGH);
    sendState();
    delay(BEAM_TIME);
    digitalWrite(PIN_BEAM, LOW);
    delay(MOVE_PAUSE);
  }
}

void patternD2() {
  modeNow = RM_SD2;
  for (int a = -40; a <= 40; a += 10) {
    if (stopActive()) break;
    movePlatform(-a, -a);
    delay(100);
    digitalWrite(PIN_BEAM, HIGH);
    sendState();
    delay(BEAM_TIME);
    digitalWrite(PIN_BEAM, LOW);
    delay(MOVE_PAUSE);
  }
}

void patternFull() {
  patternH();
  if (stopActive()) return;
  patternV();
  if (stopActive()) return;
  patternD1();
  if (stopActive()) return;
  patternD2();
}

void toCenter() {
  movePlatform(0, 0, 10);
  modeNow = RM_IDLE;
  sendState();
}

void dispatchCmd(const RcCmdFrame &c) {
  if (c.dev != UNIT_ID && c.dev != 255) return;

  switch (c.cmd) {
    case RC_ALL:
      diagAll();
      toCenter();
      break;
    case RC_LASER:
      diagLaser();
      break;
    case RC_SERVO:
      diagServos();
      break;
    case RC_RADIO:
      diagRadio();
      break;
    case RC_MCU:
      diagMCU();
      break;
    case RC_SCAN_H:
      patternH();
      toCenter();
      break;
    case RC_SCAN_V:
      patternV();
      toCenter();
      break;
    case RC_SCAN_D1:
      patternD1();
      toCenter();
      break;
    case RC_SCAN_D2:
      patternD2();
      toCenter();
      break;
    case RC_SCAN_FULL:
      patternFull();
      toCenter();
      break;
    case RC_ZERO:
      toCenter();
      break;
    default:
      break;
  }

  modeNow = RM_WAIT;
}

void setup() {
  pinMode(PIN_BEAM, OUTPUT);
  digitalWrite(PIN_BEAM, LOW);
  pinMode(PIN_STOP, INPUT_PULLUP);

  tiltSrv.attach(PIN_TILT);
  panSrv.attach(PIN_PAN);
  movePlatform(0, 0, 10);

  link.begin();
  link.setChannel(90);
  link.setDataRate(RF24_250KBPS);
  link.setPALevel(RF24_PA_LOW);
  link.openReadingPipe(1, NODE_PIPE);
  link.startListening();

  modeNow = RM_WAIT;
  delay(500);
  sendState();
}

void loop() {
  if (stopActive()) {
    digitalWrite(PIN_BEAM, LOW);
    delay(100);
    return;
  }

  if (link.available()) {
    RcCmdFrame c;
    link.read(&c, sizeof(c));
    dispatchCmd(c);
  }

  static unsigned long lastTx = 0;
  unsigned long now = millis();
  if (now - lastTx > 5000) {
    lastTx = now;
    if (modeNow == RM_WAIT || modeNow == RM_IDLE) {
      sendState();
    }
  }
}
