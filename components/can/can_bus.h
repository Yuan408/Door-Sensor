#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "esp_err.h"
#include "driver/spi_master.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Remote node sensor data, updated by poll task */
extern float g_can_temp;
extern float g_can_humi;

/** Raw sensor readings from remote SHT30_B (set by main loop) */
extern float g_remote_raw_temp;
extern float g_remote_raw_humi;

/**
 * @brief Initialize SPI bus, MCP2515, and start the poll task.
 *
 * Configures SPI (CS=5, SCK=18, MISO=19, MOSI=23) at 10 MHz.
 * Sets MCP2515 to normal mode at the baud rate selected in Kconfig.
 */
esp_err_t can_init(void);

/**
 * @brief Send a CAN frame via MCP2515 (uses TXB0).
 *
 * @param id   11-bit standard CAN ID
 * @param data Pointer to payload bytes
 * @param len  Payload length (0–8)
 */
esp_err_t can_send(uint32_t id, uint8_t *data, uint8_t len);

/**
 * @brief Read a CAN frame from MCP2515 RXB0 with timeout.
 *
 * Polls CANINTF.RX0IF every millisecond until a frame arrives or timeout.
 *
 * @param[out] id         Received CAN ID
 * @param[out] data       Buffer for payload (caller provides 8 bytes)
 * @param[in,out] len     In: buffer capacity, Out: actual DLC
 * @param timeout_ms      Max wait time in ms (0 = return immediately)
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if no frame within timeout
 */
esp_err_t can_read(uint32_t *id, uint8_t *data, uint8_t *len, int timeout_ms);

/**
 * @brief Return the SPI device handle for shared use (e.g. can_node).
 */
spi_device_handle_t can_get_spi_handle(void);

/**
 * @brief Start the background poll task that periodically requests
 *        remote sensor data (ID 0x101) and parses replies (ID 0x201).
 */
void can_start_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_BUS_H */
