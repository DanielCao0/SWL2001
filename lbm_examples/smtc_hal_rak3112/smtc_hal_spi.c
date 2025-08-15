/*!
 * \file      smtc_hal_spi.c
 *
 * \brief     SPI Hardware Abstraction Layer implementation
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_hal_spi.h"
#include "smtc_hal_gpio.h"

#include "modem_pinout.h"
#include "smtc_hal_mcu.h"
#include "driver/spi_master.h"  // ESP-IDF SPI master driver
#include "driver/gpio.h"        // ESP-IDF GPIO driver
#include "esp_err.h"            // ESP-IDF error types

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

// SPI Configuration
#define SPI_FREQUENCY_HZ    8000000     // 8 MHz SPI frequency
#define SPI_MODE            0           // SPI mode 0 (CPOL=0, CPHA=0)
#define SPI_QUEUE_SIZE      1           // SPI transaction queue size

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static spi_device_handle_t spi_handle = NULL;
static bool spi_initialized = false;


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_spi_init( const uint32_t id, const hal_gpio_pin_names_t mosi, const hal_gpio_pin_names_t miso,
                   const hal_gpio_pin_names_t sclk )
{
    if (spi_initialized) {
        return; // Already initialized
    }

    esp_err_t ret;

    // Configure SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = miso,
        .mosi_io_num = mosi,
        .sclk_io_num = sclk,
        .quadwp_io_num = -1,        // Not used
        .quadhd_io_num = -1,        // Not used
        .max_transfer_sz = 4096,    // Maximum transfer size
    };

    // Configure SPI device
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_FREQUENCY_HZ,
        .mode = SPI_MODE,
        .spics_io_num = -1,         // CS pin handled manually
        .queue_size = SPI_QUEUE_SIZE,
        .flags = 0,                 // Full duplex mode (remove SPI_DEVICE_HALFDUPLEX)
    };

    // Initialize SPI bus
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        return; // Failed to initialize bus
    }

    // Add device to SPI bus
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        spi_bus_free(SPI2_HOST);
        return; // Failed to add device
    }

    spi_initialized = true;
}

void hal_spi_de_init( const uint32_t id )
{
    if (!spi_initialized) {
        return; // Not initialized
    }

    // Remove device from SPI bus
    if (spi_handle != NULL) {
        spi_bus_remove_device(spi_handle);
        spi_handle = NULL;
    }

    // Free SPI bus
    spi_bus_free(SPI2_HOST);
    
    spi_initialized = false;
}

uint16_t hal_spi_in_out( const uint32_t id, const uint16_t out_data )
{
    if (!spi_initialized || spi_handle == NULL) {
        return 0; // SPI not initialized
    }

    uint8_t tx_data = (uint8_t)(out_data & 0xFF);  // Extract 8-bit data
    uint8_t rx_data = 0;

    spi_transaction_t trans = {
        .length = 8,        // 8 bits
        .tx_buffer = &tx_data,
        .rx_buffer = &rx_data,
    };

    esp_err_t ret = spi_device_transmit(spi_handle, &trans);
    if (ret != ESP_OK) {
        return 0; // Transaction failed
    } 

    return (uint16_t)rx_data;
}

/* --- EOF ------------------------------------------------------------------ */
