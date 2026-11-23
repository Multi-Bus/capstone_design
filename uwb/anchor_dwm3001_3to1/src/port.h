/* DWM3001CDK (nRF52833) 보드 포트. 플래시에서 혼자 도는 앵커용. */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

void Sleep(unsigned int milliseconds);
void reset_DWIC(void);

void port_init(void);                 /* 클록, 타이머, LED, UART, SPI, 난수, 감시 타이머 */
uint32_t cdk_time_us(void);           /* 1 MHz 자유 카운터 (32비트, 약 71분마다 한 바퀴) */
uint32_t port_random32(void);         /* nRF 하드웨어 난수 */
void port_wdt_feed(void);             /* 감시 타이머 (2 s 안에 안 부르면 재시작) */
uint32_t port_dw_device_id(void);     /* 리셋 직후 읽은 DW3110 DEV_ID (0xDECA0302 이어야 함) */
bool port_log(const char *line);      /* J-Link VCOM (P0.19, 115200 8N1) 으로 비동기 송신. 바쁘면 버리고 false */
uint32_t port_log_dropped(void);
void port_log_flush(uint32_t max_us); /* 앞 줄 송신이 끝날 때까지 최대 max_us 기다림 */

/* DWM3001CDK LED (보드 인쇄 D9~D12, 낮은 전압에서 켜짐) */
enum port_led { LED_D9_ERROR = 0, LED_D10_POLL, LED_D11_TAG, LED_D12_SYNC };
void port_led(enum port_led led, bool on);
void port_led_toggle(enum port_led led);
