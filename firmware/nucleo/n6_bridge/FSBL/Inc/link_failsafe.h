/**
  * @file    link_failsafe.h
  * @brief   BLE link-loss motor cut for the n6_bridge (header only, no HAL, host-testable).
  *
  * Problem (3호기, 2026-11-25): when the BLE link drops, the FC GCS failsafe switches to LAND after
  * FS_GCS_TIMEOUT (3 s). If a baro/rangefinder error stops the land detector, the motors keep running
  * and the laptop cannot send anything.
  *
  * This module watches both directions of the bridge:
  *   - laptop -> FC : valid MAVLink2 HEARTBEAT from the GCS (sysid LINKFS_GCS_SYSID)
  *   - FC -> laptop : valid MAVLink2 HEARTBEAT from the FC (sysid 1, compid 1): armed flag + custom_mode
  * If no GCS heartbeat for LINKFS_TIMEOUT_MS while the FC reports armed + LAND, it builds a
  * COMMAND_LONG MAV_CMD_COMPONENT_ARM_DISARM (param1 0, param2 21196 = force) for the bridge to inject
  * towards the FC, once per LINKFS_RESEND_MS until the FC reports disarmed or the link returns.
  * Only the LAND mode triggers it, so GUIDED flight is never cut.
  */
#ifndef LINK_FAILSAFE_H
#define LINK_FAILSAFE_H

#include <stdint.h>
#include <string.h>

#define LINKFS_TIMEOUT_MS   12000U  /* no GCS heartbeat this long -> cut (3 s failsafe + 9 s LAND; 1.3 m at 0.5 m/s = 2.6 s) */
#define LINKFS_RESEND_MS     1000U
#define LINKFS_FC_STALE_MS   3000U  /* FC heartbeat older than this -> FC state unknown, do nothing */
#define LINKFS_GCS_SYSID      250U  /* = FC MAV_GCS_SYSID */
#define LINKFS_OWN_COMPID     191U  /* MAV_COMP_ID_ONBOARD_COMPUTER, sender of the injected command */
#define LINKFS_MODE_LAND        9U
#define LINKFS_FRAME_LEN       45U  /* 10 header + 33 payload + 2 crc */

#define LINKFS_MAX_PAYLOAD    255U

typedef struct
{
  uint8_t  state;
  uint8_t  idx;
  uint8_t  len;
  uint8_t  sig;          /* signature bytes still to skip */
  uint8_t  hdr[9];       /* len, incompat, compat, seq, sys, comp, msgid0..2 */
  uint8_t  payload[LINKFS_MAX_PAYLOAD];
  uint8_t  crc[2];
  uint32_t bad_crc;      /* debug */
} LinkFsParser_t;

typedef struct
{
  LinkFsParser_t from_gcs;
  LinkFsParser_t from_fc;
  uint32_t gcs_hb_ms;    /* last valid GCS heartbeat */
  uint8_t  gcs_seen;     /* got at least one since boot */
  uint32_t fc_hb_ms;
  uint8_t  fc_seen;
  uint8_t  fc_armed;
  uint32_t fc_mode;
  uint32_t last_cmd_ms;
  uint8_t  cmd_sent_once;
  uint8_t  seq;
  uint32_t cuts;         /* debug: number of disarm frames built */
} LinkFs_t;

static inline uint16_t LinkFs_CrcAcc(uint16_t crc, uint8_t b)
{
  uint8_t t = (uint8_t)(b ^ (uint8_t)(crc & 0xFFU));
  t = (uint8_t)(t ^ (uint8_t)(t << 4));
  return (uint16_t)((crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^ ((uint16_t)t >> 4));
}

/* Feed one byte. Returns 1 when a valid HEARTBEAT (msgid 0, CRC ok) just completed. */
static inline int LinkFs_Parse(LinkFsParser_t *p, uint8_t b)
{
  switch (p->state)
  {
    case 0:                                   /* wait magic */
      if (b == 0xFDU) { p->state = 1; p->idx = 0; }
      return 0;
    case 1:                                   /* header after magic */
      p->hdr[p->idx++] = b;
      if (p->idx == 9U)
      {
        p->len = p->hdr[0];
        p->sig = ((p->hdr[1] & 0x01U) != 0U) ? 13U : 0U;
        p->idx = 0;
        p->state = (p->len == 0U) ? 3U : 2U;
      }
      return 0;
    case 2:                                   /* payload */
      p->payload[p->idx++] = b;
      if (p->idx == p->len) { p->idx = 0; p->state = 3; }
      return 0;
    case 3:                                   /* crc */
      p->crc[p->idx++] = b;
      if (p->idx < 2U) { return 0; }
      p->idx = 0;
      p->state = (p->sig != 0U) ? 4U : 0U;
      {
        uint32_t msgid = (uint32_t)p->hdr[6] | ((uint32_t)p->hdr[7] << 8) | ((uint32_t)p->hdr[8] << 16);
        uint16_t crc = 0xFFFFU;
        uint32_t i;
        if (msgid != 0U) { return 0; }
        for (i = 0; i < 9U; i++) { crc = LinkFs_CrcAcc(crc, p->hdr[i]); }
        for (i = 0; i < p->len; i++) { crc = LinkFs_CrcAcc(crc, p->payload[i]); }
        crc = LinkFs_CrcAcc(crc, 50U);        /* HEARTBEAT crc_extra */
        if (crc != (uint16_t)(p->crc[0] | ((uint16_t)p->crc[1] << 8))) { p->bad_crc++; return 0; }
        return 1;
      }
    default:                                  /* skip signature */
      if (--p->sig == 0U) { p->state = 0; }
      return 0;
  }
}

static inline uint8_t LinkFs_Pl(const LinkFsParser_t *p, uint8_t i)
{
  return (i < p->len) ? p->payload[i] : 0U;   /* MAVLink2 trims trailing zero bytes */
}

static inline void LinkFs_OnGcsByte(LinkFs_t *fs, uint8_t b, uint32_t now)
{
  LinkFsParser_t *p = &fs->from_gcs;
  if (LinkFs_Parse(p, b) && p->hdr[4] == LINKFS_GCS_SYSID)
  {
    fs->gcs_hb_ms = now;
    fs->gcs_seen = 1;
  }
}

static inline void LinkFs_OnFcByte(LinkFs_t *fs, uint8_t b, uint32_t now)
{
  LinkFsParser_t *p = &fs->from_fc;
  if (LinkFs_Parse(p, b) && p->hdr[4] == 1U && p->hdr[5] == 1U)
  {
    fs->fc_mode = (uint32_t)LinkFs_Pl(p, 0) | ((uint32_t)LinkFs_Pl(p, 1) << 8)
                | ((uint32_t)LinkFs_Pl(p, 2) << 16) | ((uint32_t)LinkFs_Pl(p, 3) << 24);
    fs->fc_armed = ((LinkFs_Pl(p, 6) & 0x80U) != 0U) ? 1U : 0U;   /* base_mode SAFETY_ARMED */
    fs->fc_hb_ms = now;
    fs->fc_seen = 1;
  }
}

/* Call often. Returns LINKFS_FRAME_LEN and fills out[] when a force-disarm frame must be sent now. */
static inline uint32_t LinkFs_Poll(LinkFs_t *fs, uint32_t now, uint8_t out[LINKFS_FRAME_LEN])
{
  static const float p2 = 21196.0f;
  uint16_t crc = 0xFFFFU;
  uint32_t i;

  if (!fs->gcs_seen || !fs->fc_seen) { return 0; }
  if ((now - fs->gcs_hb_ms) < LINKFS_TIMEOUT_MS) { return 0; }
  if ((now - fs->fc_hb_ms) > LINKFS_FC_STALE_MS) { return 0; }
  if (!fs->fc_armed || fs->fc_mode != LINKFS_MODE_LAND) { return 0; }
  if (fs->cmd_sent_once && (now - fs->last_cmd_ms) < LINKFS_RESEND_MS) { return 0; }

  memset(out, 0, LINKFS_FRAME_LEN);
  out[0] = 0xFDU;
  out[1] = 33U;                       /* COMMAND_LONG payload */
  out[4] = fs->seq++;
  out[5] = LINKFS_GCS_SYSID;
  out[6] = LINKFS_OWN_COMPID;
  out[7] = 76U;                       /* COMMAND_LONG */
  memcpy(&out[10 + 4], &p2, 4);       /* param2 (param1 = 0 -> disarm) */
  out[10 + 28] = (uint8_t)(400U & 0xFFU);   /* MAV_CMD_COMPONENT_ARM_DISARM */
  out[10 + 29] = (uint8_t)(400U >> 8);
  out[10 + 30] = 1U;                  /* target_system */
  out[10 + 31] = 1U;                  /* target_component */
  for (i = 1; i < 43U; i++) { crc = LinkFs_CrcAcc(crc, out[i]); }
  crc = LinkFs_CrcAcc(crc, 152U);     /* COMMAND_LONG crc_extra */
  out[43] = (uint8_t)(crc & 0xFFU);
  out[44] = (uint8_t)(crc >> 8);

  fs->last_cmd_ms = now;
  fs->cmd_sent_once = 1;
  fs->cuts++;
  return LINKFS_FRAME_LEN;
}

#endif /* LINK_FAILSAFE_H */
