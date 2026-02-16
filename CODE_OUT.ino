#include <SPI.h>
#include <RF24.h>

enum TxCmd : uint8_t {
  TXC_NONE = 0,
  TXC_ALL,
  TXC_LASER,
  TXC_SERVOS,
  TXC_RADIO,
  TXC_MCU,
  TXC_SCAN_H,
  TXC_SCAN_V,
  TXC_SCAN_D1,
  TXC_SCAN_D2,
  TXC_ZERO,
  TXC_SCAN_FULL
};

enum TxMode : uint8_t {
  TM_IDLE = 0,
  TM_WAIT,
  TM_H,
  TM_V,
  TM_D1,
  TM_D2,
  TM_TEST
};

enum TxTest : uint8_t {
  TT_NONE = 0,
  TT_LASER,
  TT_SERVOS,
  TT_RADIO,
  TT_MCU,
  TT_ALL
};

struct TxCmdPacket {
  uint8_t dev;
  TxCmd   cmd;
};

struct TxTelPacket {
  uint8_t dev;
  int8_t  tilt;
  int8_t  pan;
  TxMode  mode;
};

struct TxTestPacket {
  uint8_t dev;
  TxTest  test;
  bool    start;
  bool    done;
  bool    ok;
};

const uint8_t TX_RF_CE  = 7;
const uint8_t TX_RF_CSN = 8;

RF24 txRadio(TX_RF_CE, TX_RF_CSN);

const byte PIPE_BASE[6] = "BASE1";
const byte PIPE_NODE[6] = "CUBE1";

const uint8_t RX_NODE_ID = 1;

void sendCmdFrame(TxCmd c) {
  TxCmdPacket p;
  p.dev = RX_NODE_ID;
  p.cmd = c;

  txRadio.stopListening();
  txRadio.openWritingPipe(PIPE_NODE);
  bool sent = txRadio.write(&p, sizeof(p));
  txRadio.startListening();

  Serial.print("cmd ");
  Serial.print((int)c);
  Serial.print(" -> ");
  Serial.println(sent ? "ok" : "err");
}

const char* modeStr(TxMode m) {
  switch (m) {
    case TM_IDLE: return "IDLE";
    case TM_WAIT: return "WAIT";
    case TM_H:    return "H";
    case TM_V:    return "V";
    case TM_D1:   return "D1";
    case TM_D2:   return "D2";
    case TM_TEST: return "TEST";
    default:      return "?";
  }
}

const char* testStr(TxTest t) {
  switch (t) {
    case TT_LASER:  return "LASER";
    case TT_SERVOS: return "SERVOS";
    case TT_RADIO:  return "RADIO";
    case TT_MCU:    return "MCU";
    case TT_ALL:    return "ALL";
    default:        return "NONE";
  }
}

void showMenu() {
  Serial.println("Console ready");
  Serial.println("Keys:");
  Serial.println(" 1 -> all tests");
  Serial.println(" 2 -> laser");
  Serial.println(" 3 -> servos");
  Serial.println(" 4 -> radio");
  Serial.println(" 5 -> mcu");
  Serial.println(" h -> scan H");
  Serial.println(" v -> scan V");
  Serial.println(" d -> diag 1");
  Serial.println(" f -> diag 2");
  Serial.println(" s -> scan full");
  Serial.println(" 0 -> home");
}

void handleKey(char c) {
  if      (c == '1') sendCmdFrame(TXC_ALL);
  else if (c == '2') sendCmdFrame(TXC_LASER);
  else if (c == '3') sendCmdFrame(TXC_SERVOS);
  else if (c == '4') sendCmdFrame(TXC_RADIO);
  else if (c == '5') sendCmdFrame(TXC_MCU);
  else if (c == 'h') sendCmdFrame(TXC_SCAN_H);
  else if (c == 'v') sendCmdFrame(TXC_SCAN_V);
  else if (c == 'd') sendCmdFrame(TXC_SCAN_D1);
  else if (c == 'f') sendCmdFrame(TXC_SCAN_D2);
  else if (c == 's') sendCmdFrame(TXC_SCAN_FULL);
  else if (c == '0') sendCmdFrame(TXC_ZERO);
}

void printTest(const TxTestPacket* t) {
  Serial.print("[T] id=");
  Serial.print(t->dev);
  Serial.print(" type=");
  Serial.print(testStr(t->test));
  Serial.print(" st=");
  Serial.print(t->start ? "1" : "0");
  Serial.print(" end=");
  Serial.print(t->done ? "1" : "0");
  Serial.print(" ok=");
  Serial.println(t->ok ? "1" : "0");
}

void printTel(const TxTelPacket* t) {
  Serial.print("[S] id=");
  Serial.print(t->dev);
  Serial.print(" m=");
  Serial.print(modeStr(t->mode));
  Serial.print(" tilt=");
  Serial.print(t->tilt);
  Serial.print(" pan=");
  Serial.println(t->pan);
}

void handleRx() {
  uint8_t buf[32];
  txRadio.read(&buf, sizeof(buf));

  TxTestPacket* tst = (TxTestPacket*)buf;
  bool isDiag =
    tst->dev == RX_NODE_ID &&
    tst->test >= TT_LASER &&
    tst->test <= TT_ALL;

  if (isDiag) {
    printTest(tst);
  } else {
    TxTelPacket* tp = (TxTelPacket*)buf;
    printTel(tp);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  showMenu();

  txRadio.begin();
  txRadio.setChannel(90);
  txRadio.setDataRate(RF24_250KBPS);
  txRadio.setPALevel(RF24_PA_LOW);
  txRadio.openReadingPipe(1, PIPE_BASE);
  txRadio.startListening();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    handleKey(c);
  }

  if (txRadio.available()) {
    handleRx();
  }
}
