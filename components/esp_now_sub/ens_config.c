#include "ens_config.h"

static const uint8_t OBC_MAC_ADDRESS[ESP_NOW_ETH_ALEN] = {0x04, 0x20, 0x04, 0x20, 0x04, 0x20};
static const uint8_t BROADCAST_MAC_ADDRESS[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t *get_obc_mac_address(void){
    return OBC_MAC_ADDRESS;
}

uint8_t *get_broadcast_mac_address(void){
    return BROADCAST_MAC_ADDRESS;
}