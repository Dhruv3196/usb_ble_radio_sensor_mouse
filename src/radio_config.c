#include <hal/nrf_radio.h>
#include <tinycrypt/aes.h>
#include <tinycrypt/constants.h>

// Example AES-128 key (16 bytes)
static uint8_t aes_key[TC_AES_KEY_SIZE] = { 
    0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 
    0xab, 0xf7, 0x7f, 0x67, 0x1f, 0xc7, 0x7e, 0x55 
};

// Example IV (16 bytes)
static uint8_t aes_iv[TC_AES_BLOCK_SIZE] = { 
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f 
};

void radio_configure()
{
    NRF_RADIO->POWER = 1;
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;

    NRF_RADIO->MODE = RADIO_MODE_MODE_Ble_1Mbit;
    NRF_RADIO->FREQUENCY = 40;

    NRF_RADIO->PCNF0 = (8 << RADIO_PCNF0_LFLEN_Pos) | (0 << RADIO_PCNF0_S0LEN_Pos) | (0 << RADIO_PCNF0_S1LEN_Pos);
    NRF_RADIO->PCNF1 = (32 << RADIO_PCNF1_MAXLEN_Pos) | (0 << RADIO_PCNF1_STATLEN_Pos) | (3 << RADIO_PCNF1_BALEN_Pos) | (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos) | (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos);

    NRF_RADIO->BASE0 = 0x89ABCDEF;
    NRF_RADIO->PREFIX0 = 0xC3C2C1C0;
    NRF_RADIO->TXADDRESS = 0;
    NRF_RADIO->RXADDRESSES = 1;

    NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_0dBm;

    NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_Three;
    NRF_RADIO->CRCINIT = 0xFFFFFF;
    NRF_RADIO->CRCPOLY = 0x11021;

    NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
}

void radio_send_packet(uint32_t *packet)
{
    struct tc_aes_key_sched_struct sched;
    uint8_t encrypted_packet[32];

    tc_aes128_set_encrypt_key(&sched, aes_key);
    tc_aes_encrypt(encrypted_packet, (const uint8_t *)packet, &sched);

    NRF_RADIO->PACKETPTR = (uint32_t)encrypted_packet;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_TXEN = 1;
    while (NRF_RADIO->EVENTS_END == 0);
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);
}

void radio_receive_packet(uint32_t *packet)
{
    struct tc_aes_key_sched_struct sched;
    uint8_t encrypted_packet[32];

    NRF_RADIO->PACKETPTR = (uint32_t)encrypted_packet;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_RXEN = 1;
    while (NRF_RADIO->EVENTS_END == 0);
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);

    tc_aes128_set_decrypt_key(&sched, aes_key);
    tc_aes_decrypt((uint8_t *)packet, encrypted_packet, &sched);
}
