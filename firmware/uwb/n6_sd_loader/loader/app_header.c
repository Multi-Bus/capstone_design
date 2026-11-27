#include "app_header.h"

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

enum app_check app_check_header(const uint8_t raw[APP_HEADER_SIZE], uint8_t my_id, uint32_t file_bytes,
                                struct app_header *out)
{
    if (raw[0] != 'N' || raw[1] != '6' || raw[2] != 'A' || raw[3] != 'P') {
        return APP_BAD_MAGIC;
    }
    out->version = (uint16_t)(raw[4] | (raw[5] << 8));
    out->tag_id = raw[6];
    out->load = le32(raw + 8);
    out->length = le32(raw + 12);
    out->crc = le32(raw + 16);
    for (unsigned i = 0; i < 12U; ++i) {
        const char c = (char)raw[20 + i];
        out->name[i] = (c >= 0x20 && c < 0x7F) ? c : '\0';
    }
    out->name[12] = '\0';
    if (out->version != APP_HEADER_VERSION) {
        return APP_BAD_VERSION;
    }
    if (out->tag_id != my_id) {
        return APP_WRONG_ID;
    }
    if (out->load != APP_LOAD_ADDR || out->length == 0U || out->length > APP_MAX_LEN ||
        file_bytes < APP_HEADER_SIZE || file_bytes - APP_HEADER_SIZE < out->length) {
        return APP_BAD_LENGTH;
    }
    return APP_OK;
}

enum app_check app_check_vectors(const uint8_t *image, uint32_t length, uint32_t base)
{
    if (length < 8U) {
        return APP_BAD_VECTORS;
    }
    const uint32_t sp = le32(image);
    const uint32_t entry = le32(image + 4) & ~1UL;
    if (sp <= APP_RAM_LO || sp > APP_RAM_HI || (sp & 7U) != 0U || entry < base || entry >= base + length) {
        return APP_BAD_VECTORS;
    }
    return APP_OK;
}

uint32_t app_crc32(uint32_t crc, const uint8_t *data, uint32_t length)
{
    crc = ~crc;
    while (length--) {
        crc ^= *data++;
        for (unsigned k = 0; k < 8U; ++k) {
            crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
        }
    }
    return ~crc;
}

const char *app_check_name(enum app_check result)
{
    switch (result) {
    case APP_OK: return "ok";
    case APP_BAD_MAGIC: return "bad_magic";
    case APP_BAD_VERSION: return "bad_version";
    case APP_WRONG_ID: return "wrong_id";
    case APP_BAD_LENGTH: return "bad_length";
    case APP_BAD_CRC: return "bad_crc";
    case APP_BAD_VECTORS: return "bad_vectors";
    }
    return "?";
}
