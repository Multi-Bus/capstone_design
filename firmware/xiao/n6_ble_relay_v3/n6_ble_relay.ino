// N6 드론 - XIAO nRF52840 BLE <-> UART 중계 펌웨어  (v3: NUCLEO -> BLE 를 큰 버퍼 + 따로 도는 보내기 작업으로)
// 노트북/휴대폰(BLE, Nordic UART Service) <-> XIAO <-> NUCLEO(USART2)
// D6 = TX -> NUCLEO, D7 = RX <- NUCLEO
// D0 = 연결 표시 (High = BLE 연결 + 노트북 데이터가 1초 안에 들어오는 중)
//
// 보드 패키지: Seeed nRF52 Boards (mbed 아닌 쪽) / 보드: Seeed XIAO nRF52840
//
// ---------------------------------------------------------------------------
// v1(문서 "XIAO BLE 중계 펌웨어 올리기") 과 달라진 점 — NUCLEO -> BLE 방향만
//
//  v1 의 문제 (2026-11-19, 11-25 실측):
//   - loop() 가 돌 때마다 UART 에 들어와 있던 1~4 바이트를 곧바로 BLE 알림 하나로 보냈다.
//     21 바이트 HEARTBEAT 하나가 알림 6~7개(1, 1, 1, 2, 16 바이트 ...)로 쪼개져 나갔다.
//   - 알림 큐(3개)가 차면 bleuart.write() 가 빈 자리를 기다리며 최대 100 ms 멈춘다
//     (Bluefruit 의 getHvnPacket). 그동안 loop() 가 Serial1 을 읽지 못하는데 Serial1 의 수신 버퍼는
//     64 바이트(115200 bps 에서 5.5 ms 분량)뿐이라 넘친 바이트가 사라진다.
//   -> 약 1 kB/s 부터 메시지 가운데가 2~14 바이트씩 빠져 CRC 가 깨졌다. 받는 양도 1 kB/s 에서 막혔다.
//
//  고친 것:
//   1. Serial1 을 읽는 일(loop)과 BLE 로 보내는 일(sendLoop)을 서로 다른 작업으로 나눴다.
//      보내는 쪽이 기다려도 loop() 는 계속 돌면서 Serial1 을 4096 바이트 버퍼로 옮긴다.
//   2. 보내는 쪽은 버퍼에서 알림 한 개 크기(MTU-3, 보통 244 B)까지 모아서 보낸다. 보내는 때:
//        - 알림 한 개 크기만큼 찼을 때
//        - 2 ms 동안 새 바이트가 없을 때 (= FC 가 보낸 한 덩어리가 끝남)
//        - 가장 오래된 바이트가 10 ms 를 기다렸을 때 (지연 상한)
//   3. 알림 하나는 MTU-3 이하라서 "전부 성공 / 전부 실패" 이다. 실패하면 버리지 않고 버퍼에 둔 채
//      다시 보낸다(중복 없음). 버퍼가 가득 찼을 때만 새 바이트를 버리고 lost 로 센다.
//   4. 연결 간격을 짧게(7.5~15 ms) 요청해서 초당 보낼 수 있는 알림 수를 늘린다.
//  그대로인 것: 이름(N6-DRONE-x), 115200 8N1, D6/D7, D0 표시 규칙, LED 뜻, BLE -> NUCLEO 방향.
//
//  USB 시리얼(115200)로 나오는 것 (PC 에 꽂았을 때 확인용):
//    N6 BLE relay v3 N6-DRONE-4            켜질 때
//    BLE connected / BLE disconnected      연결이 바뀔 때
//    stat rx=.. sent=.. notif=.. retry=.. lost=.. mtu=.. ci=..    연결 중 5초마다
//      rx = Serial1 에서 받은 바이트, sent = BLE 로 보낸 바이트, notif = 알림 수,
//      retry = 큐가 차서 다시 보낸 횟수, lost = 버퍼가 차서 버린 바이트(0 이어야 함), ci = 연결 간격(1.25 ms 단위)
// ---------------------------------------------------------------------------

#include <bluefruit.h>

// ===== 기체마다 바꿀 값 =====
#ifndef DRONE_ID                              // (명령줄 빌드에서 -DDRONE_ID=n 으로 줄 수도 있음)
#define DRONE_ID 1                            // 1~6, 기체 번호
#endif
// ===========================

#define RELAY_VERSION "v3"

const uint32_t UART_BAUD       = 115200;  // NUCLEO USART2 속도와 같게
const uint32_t LINK_TIMEOUT_MS = 1000;    // 1초 동안 노트북 데이터가 없으면 D0 Low
const uint8_t  LINK_PIN        = D0;      // -> 캐리어 J10 5번 -> NUCLEO PD11

const size_t   PIECE_MAX       = 244;     // 알림 한 개 최대 (MTU 247 - 3)
const uint32_t TX_IDLE_US      = 2000;    // 2 ms 동안 새 바이트가 없으면 보냄 (115200 에서 약 23 바이트 시간)
const uint32_t TX_MAX_WAIT_US  = 10000;   // 가장 오래된 바이트가 10 ms 를 넘겨 기다리지 않게

// XIAO 의 LED 는 LOW 일 때 켜져요
#define LED_ON  LOW
#define LED_OFF HIGH

BLEUart bleuart;
volatile bool     bleConnected = false;
volatile uint16_t connHandle   = BLE_CONN_HANDLE_INVALID;
volatile uint32_t lastRxMs     = 0;

// NUCLEO -> BLE 버퍼. 넣는 쪽은 loop() 하나, 꺼내는 쪽은 sendLoop() 하나라서 잠금 없이 쓴다
// (qHead 는 loop 만, qTail 은 sendLoop 만 바꾼다).
const uint32_t Q_SIZE = 4096;             // 2의 거듭제곱. 115200 bps 로 꽉 채워 들어와도 0.35 s 분량
uint8_t           q[Q_SIZE];
volatile uint32_t qHead = 0;              // 다음에 넣을 자리
volatile uint32_t qTail = 0;              // 다음에 꺼낼 자리
volatile uint32_t qFirstUs = 0;           // 버퍼가 비어 있다가 첫 바이트가 들어온 때
volatile uint32_t qLastUs  = 0;           // 마지막 바이트가 들어온 때
volatile bool     qDiscard = false;       // 연결이 바뀌면 쌓인 것을 버린다 (sendLoop 가 처리)

volatile uint32_t statRx = 0, statSent = 0, statNotif = 0, statRetry = 0, statLost = 0;

void onConnect(uint16_t conn_handle) {
  connHandle = conn_handle;
  qDiscard = true;
  bleConnected = true;
  Serial.println("BLE connected");
}

void onDisconnect(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  bleConnected = false;
  connHandle = BLE_CONN_HANDLE_INVALID;
  qDiscard = true;
  Serial.print("BLE disconnected, reason 0x");
  Serial.println(reason, HEX);
}

void startAdv() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);  // 끊기면 바로 다시 광고
  Bluefruit.Advertising.setInterval(32, 244);       // 20 ms ~ 152.5 ms
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);                   // 0 = 계속 광고
}

// 지금 연결에서 알림 하나에 실을 수 있는 최대 바이트 (MTU - 3)
static size_t notifyPayloadMax() {
  BLEConnection* conn = Bluefruit.Connection(connHandle);
  size_t n = conn ? (size_t)conn->getMtu() - 3 : 20;
  if (n < 20) n = 20;
  if (n > PIECE_MAX) n = PIECE_MAX;
  return n;
}

// 보내기 작업: 버퍼에 쌓인 것을 알림으로 보낸다. bleuart.write() 는 알림 큐가 찼으면 빈 자리를 기다리며
// 최대 100 ms 멈추는데, 이 작업만 멈추고 loop() 는 계속 Serial1 을 읽는다.
void sendLoop() {
  static uint8_t piece[PIECE_MAX];

  if (qDiscard) {                  // 연결이 바뀜: 그 전에 쌓인 것은 보내지 않는다
    qDiscard = false;
    qTail = qHead;
  }
  const uint16_t handle = connHandle;
  if (!bleConnected || handle == BLE_CONN_HANDLE_INVALID || !bleuart.notifyEnabled(handle)) {
    qTail = qHead;                 // 받을 상대가 없으면 버린다 (나중에 묵은 값이 한꺼번에 나가지 않게)
    delay(5);
    return;
  }

  const uint32_t tail = qTail;
  const uint32_t count = (qHead - tail) & (Q_SIZE - 1);
  if (count == 0) {
    delay(1);
    return;
  }
  const size_t maxPiece = notifyPayloadMax();
  const uint32_t now = micros();
  if (count < maxPiece && (now - qLastUs) < TX_IDLE_US && (now - qFirstUs) < TX_MAX_WAIT_US) {
    delay(1);                      // 한 덩어리가 다 들어올 때까지 조금 더 모은다
    return;
  }

  const size_t n = count < maxPiece ? count : maxPiece;
  for (size_t i = 0; i < n; ++i) piece[i] = q[(tail + i) & (Q_SIZE - 1)];
  if (bleuart.write(handle, piece, n) == n) {
    if (!qDiscard) qTail = (tail + n) & (Q_SIZE - 1);
    qFirstUs = micros();           // 남은 바이트가 있으면 지금부터 다시 기다림을 센다
    statSent += n;
    ++statNotif;
  } else {
    ++statRetry;                   // 큐가 찼거나 연결이 막 끊김: 버퍼에 둔 채 다시 시도
    delay(2);
  }
}

void setup() {
  pinMode(LINK_PIN, OUTPUT);
  digitalWrite(LINK_PIN, LOW);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_GREEN, LED_OFF);
  digitalWrite(LED_BLUE, LED_OFF);

  Serial.begin(115200);        // USB 디버그 출력 (PC 에 꽂았을 때만)
  Serial1.begin(UART_BAUD);    // D6 = TX, D7 = RX

  char name[20];
  snprintf(name, sizeof(name), "N6-DRONE-%d", DRONE_ID);

  Bluefruit.autoConnLed(false);  // LED 는 아래 loop() 에서 직접 제어
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName(name);
  Bluefruit.Periph.setConnInterval(6, 12);   // 7.5 ~ 15 ms 요청 (노트북이 받아들이면 알림 처리량 증가)
  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);

  bleuart.begin();
  startAdv();

  Scheduler.startLoop(sendLoop);   // 보내기 작업 (loop 와 같은 우선순위)

  Serial.print("N6 BLE relay " RELAY_VERSION " ");
  Serial.println(name);
}

void loop() {
  // 노트북 -> NUCLEO (v1 그대로)
  while (bleuart.available()) {
    Serial1.write((uint8_t)bleuart.read());
    lastRxMs = millis();
  }

  // NUCLEO -> 버퍼. 여기서는 기다리는 일이 없어야 한다 (Serial1 수신 버퍼가 64 바이트뿐)
  while (Serial1.available()) {
    const uint8_t b = (uint8_t)Serial1.read();
    ++statRx;
    const uint32_t head = qHead;
    const uint32_t next = (head + 1) & (Q_SIZE - 1);
    if (next == qTail) {           // 가득 참: 새 바이트를 버린다
      if (bleConnected) ++statLost;
      continue;
    }
    const uint32_t now = micros();
    if (head == qTail) qFirstUs = now;
    q[head] = b;
    qLastUs = now;
    qHead = next;
  }

  // 연결 표시: 연결돼 있고 1초 안에 노트북 데이터가 왔으면 High (v1 그대로)
  bool alive = bleConnected && (millis() - lastRxMs < LINK_TIMEOUT_MS);
  digitalWrite(LINK_PIN, alive ? HIGH : LOW);
  digitalWrite(LED_GREEN, alive ? LED_ON : LED_OFF);
  digitalWrite(LED_BLUE, bleConnected ? LED_ON : LED_OFF);

  // 연결 중 5초마다 통계 (USB 시리얼 모니터를 열었을 때만)
  static uint32_t lastReportMs = 0;
  if (millis() - lastReportMs >= 5000) {
    lastReportMs = millis();
    if (bleConnected && Serial) {
      BLEConnection* conn = Bluefruit.Connection(connHandle);
      char line[120];
      snprintf(line, sizeof(line), "stat rx=%lu sent=%lu notif=%lu retry=%lu lost=%lu mtu=%u ci=%u",
               (unsigned long)statRx, (unsigned long)statSent, (unsigned long)statNotif,
               (unsigned long)statRetry, (unsigned long)statLost,
               conn ? conn->getMtu() : 0, conn ? conn->getConnectionInterval() : 0);
      Serial.println(line);
    }
  }
}
