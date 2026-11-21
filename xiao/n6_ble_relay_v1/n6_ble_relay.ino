// N6 드론 - XIAO nRF52840 BLE <-> UART 중계 펌웨어
// 휴대폰(BLE, Nordic UART Service) <-> XIAO <-> NUCLEO(UART2)
// D6 = TX -> NUCLEO, D7 = RX <- NUCLEO
// D0 = 연결 표시 (High = 휴대폰 데이터가 1초 안에 들어오는 중)
// 보드 패키지: Seeed nRF52 Boards / 보드: Seeed XIAO nRF52840

#include <bluefruit.h>

// ===== 기체마다 바꿀 값 =====
#define DRONE_ID 1                  // 1~6, 기체 번호
// ============================

const uint32_t UART_BAUD       = 115200;   // NUCLEO UART2 속도와 같게
const uint32_t LINK_TIMEOUT_MS = 1000;     // 1초 끊기면 D0 Low (+ FC FS_GCS_TIMEOUT 2초 = 3초 뒤 착륙)
const uint8_t  LINK_PIN        = D0;       // -> 캐리어 J10 5번 -> NUCLEO PD11

// XIAO의 LED는 LOW일 때 켜져요
#define LED_ON  LOW
#define LED_OFF HIGH

BLEUart bleuart;
volatile bool bleConnected = false;
volatile uint32_t lastRxMs = 0;

void onConnect(uint16_t conn_handle) {
  (void)conn_handle;
  bleConnected = true;
  Serial.println("BLE connected");
}

void onDisconnect(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  bleConnected = false;
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

void setup() {
  pinMode(LINK_PIN, OUTPUT);
  digitalWrite(LINK_PIN, LOW);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_GREEN, LED_OFF);
  digitalWrite(LED_BLUE, LED_OFF);

  Serial.begin(115200);        // USB 디버그 출력 (PC에 꼽았을 때만)
  Serial1.begin(UART_BAUD);    // D6 = TX, D7 = RX

  char name[20];
  snprintf(name, sizeof(name), "N6-DRONE-%d", DRONE_ID);

  Bluefruit.autoConnLed(false);  // LED는 아래 loop()에서 직접 제어
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName(name);
  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);

  bleuart.begin();
  startAdv();
}

void loop() {
  // 휴대폰 -> NUCLEO
  while (bleuart.available()) {
    Serial1.write((uint8_t)bleuart.read());
    lastRxMs = millis();
  }

  // NUCLEO -> 휴대폰
  uint8_t buf[64];
  size_t n = 0;
  while (Serial1.available() && n < sizeof(buf)) {
    buf[n++] = (uint8_t)Serial1.read();
  }
  if (n > 0 && bleConnected) {
    bleuart.write(buf, n);
  }

  // 연결 표시: 연결돼 있고 1초 안에 데이터가 왔으면 High
  bool alive = bleConnected && (millis() - lastRxMs < LINK_TIMEOUT_MS);
  digitalWrite(LINK_PIN, alive ? HIGH : LOW);
  digitalWrite(LED_GREEN, alive ? LED_ON : LED_OFF);
  digitalWrite(LED_BLUE, bleConnected ? LED_ON : LED_OFF);
}
