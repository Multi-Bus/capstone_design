// N6 드론 - XIAO nRF52840 BLE <-> UART 중계 펌웨어  (v2: NUCLEO -> BLE 묶어 보내기)
// 노트북/휴대폰(BLE, Nordic UART Service) <-> XIAO <-> NUCLEO(USART2)
// D6 = TX -> NUCLEO, D7 = RX <- NUCLEO
// D0 = 연결 표시 (High = BLE 연결 + 노트북 데이터가 1초 안에 들어오는 중)
//
// 보드 패키지: Seeed nRF52 Boards (mbed 아닌 쪽) / 보드: Seeed XIAO nRF52840
//
// ---------------------------------------------------------------------------
// v1(문서 "XIAO BLE 중계 펌웨어 올리기") 과 달라진 점 — NUCLEO -> BLE 방향만
//
//  문제 (2026-11-19 실측):
//   v1 은 loop() 가 돌 때마다 UART 에 들어와 있던 1~4 바이트를 곧바로 BLE 알림 하나로 보냈다.
//   115200 bps 에서는 바이트가 87 us 마다 하나씩 들어오고 loop() 는 그보다 빨라서, 알림이
//   아주 잘게 쪼개져 초당 수백 개가 된다. 알림 큐(BANDWIDTH_MAX 에서 3개)가 차면
//   bleuart.write() 가 실패하는데 v1 은 반환값을 보지 않아 그 바이트를 그냥 버렸다.
//   -> FC 가 1초마다 몰아서 보내는 HEARTBEAT/SYS_STATUS 의 약 70% 가 노트북에 도착하지 않음.
//      (깨진 지점이 항상 "1 B 알림 3개 뒤 몇 바이트 빠짐")
//
//  고친 것:
//   1. 바이트를 txBuf 에 모았다가 한 번에 보낸다. 보내는 때:
//        - 버퍼가 알림 한 개 최대 크기(MTU-3, 보통 244 B)만큼 찼을 때
//        - 2 ms 동안 새 바이트가 없을 때 (= FC 가 보낸 한 덩어리가 끝남)
//        - 첫 바이트가 들어온 지 10 ms 가 지났을 때 (지연 상한)
//   2. 알림 하나는 MTU-3 이하로만 보내므로 "전부 성공 / 전부 실패" 이다. 실패하면 최대 20 ms 동안
//      다시 시도하고, 그래도 안 되면 버리고 txDropped 를 센다 (중복 전송은 생기지 않음).
//   3. 연결 간격을 짧게(7.5~15 ms) 요청해서 초당 보낼 수 있는 알림 수를 늘린다.
//  그대로인 것: 이름(N6-DRONE-x), 115200 8N1, D6/D7, D0 표시 규칙, LED 뜻, BLE -> NUCLEO 방향.
// ---------------------------------------------------------------------------

#include <bluefruit.h>

// ===== 기체마다 바꿀 값 =====
#define DRONE_ID 1                            // 1~6, 기체 번호
// ===========================

const uint32_t UART_BAUD       = 115200;  // NUCLEO USART2 속도와 같게
const uint32_t LINK_TIMEOUT_MS = 1000;    // 1초 동안 노트북 데이터가 없으면 D0 Low
const uint8_t  LINK_PIN        = D0;      // -> 캐리어 J10 5번 -> NUCLEO PD11

const size_t   TX_BUF_MAX      = 244;     // 알림 한 개 최대 (MTU 247 - 3)
const uint32_t TX_IDLE_US      = 2000;    // 2 ms 동안 새 바이트가 없으면 보냄 (115200 에서 약 23 바이트 시간)
const uint32_t TX_MAX_WAIT_US  = 10000;   // 첫 바이트 뒤 최대 10 ms 안에는 보냄
const uint32_t TX_RETRY_MS     = 20;      // 알림 큐가 찼을 때 다시 시도하는 최대 시간

// XIAO 의 LED 는 LOW 일 때 켜져요
#define LED_ON  LOW
#define LED_OFF HIGH

BLEUart bleuart;
volatile bool     bleConnected = false;
volatile uint16_t connHandle   = BLE_CONN_HANDLE_INVALID;
volatile uint32_t lastRxMs     = 0;

uint8_t  txBuf[TX_BUF_MAX];
size_t   txLen     = 0;
uint32_t txFirstUs = 0;
uint32_t txLastUs  = 0;
uint32_t txDropped = 0;   // 끝내 못 보낸 바이트 수 (USB 시리얼로 1초마다 출력)

void onConnect(uint16_t conn_handle) {
  connHandle = conn_handle;
  bleConnected = true;
  txLen = 0;
  Serial.println("BLE connected");
}

void onDisconnect(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  bleConnected = false;
  connHandle = BLE_CONN_HANDLE_INVALID;
  txLen = 0;
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
  if (n > TX_BUF_MAX) n = TX_BUF_MAX;
  return n;
}

// txBuf 를 알림으로 보낸다. 알림 하나는 MTU-3 이하라서 전부 성공 / 전부 실패이므로
// 실패 시 같은 조각을 다시 보내도 중복이 생기지 않는다.
static void flushTx() {
  if (txLen == 0) return;
  if (bleConnected) {
    const size_t maxPiece = notifyPayloadMax();
    size_t sent = 0;
    uint32_t t0 = millis();
    while (sent < txLen) {
      size_t piece = txLen - sent;
      if (piece > maxPiece) piece = maxPiece;
      size_t k = bleuart.write(txBuf + sent, piece);
      if (k == piece) { sent += piece; continue; }
      if (!bleConnected || millis() - t0 > TX_RETRY_MS) break;
      delay(1);   // 알림 큐가 빌 때까지 잠깐 양보
    }
    if (sent < txLen) txDropped += (txLen - sent);
  }
  txLen = 0;
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
}

void loop() {
  // 노트북 -> NUCLEO (v1 그대로)
  while (bleuart.available()) {
    Serial1.write((uint8_t)bleuart.read());
    lastRxMs = millis();
  }

  // NUCLEO -> 노트북: 모아서 보냄
  while (Serial1.available() && txLen < TX_BUF_MAX) {
    uint32_t now = micros();
    if (txLen == 0) txFirstUs = now;
    txBuf[txLen++] = (uint8_t)Serial1.read();
    txLastUs = now;
  }
  if (txLen > 0) {
    uint32_t now = micros();
    if (txLen >= notifyPayloadMax() ||
        (now - txLastUs) >= TX_IDLE_US ||
        (now - txFirstUs) >= TX_MAX_WAIT_US) {
      flushTx();
    }
  }

  // 연결 표시: 연결돼 있고 1초 안에 노트북 데이터가 왔으면 High (v1 그대로)
  bool alive = bleConnected && (millis() - lastRxMs < LINK_TIMEOUT_MS);
  digitalWrite(LINK_PIN, alive ? HIGH : LOW);
  digitalWrite(LED_GREEN, alive ? LED_ON : LED_OFF);
  digitalWrite(LED_BLUE, bleConnected ? LED_ON : LED_OFF);

  // 1초마다 버린 바이트 수 (USB 시리얼 모니터에서 확인용)
  static uint32_t lastReportMs = 0;
  if (millis() - lastReportMs >= 1000) {
    lastReportMs = millis();
    if (txDropped) {
      Serial.print("txDropped=");
      Serial.println(txDropped);
    }
  }
}
