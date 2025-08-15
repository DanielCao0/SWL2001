/*!
 * \file      smtc_hal_flash.c
 *
 * \brief     FLASH Hardware Abstraction Layer implementation
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

#include "smtc_hal_flash.h"
#include "smtc_hal_dbg_trace.h"

#include <string.h>

#include "esp_flash.h"
#include "esp_partition.h"
#include "esp_log.h"
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * Generic definition
 */
#ifndef SUCCESS
#define SUCCESS 1
#endif

#ifndef FAIL
#define FAIL 0
#endif

// ESP32S3 Flash sector/page size
#define FLASH_PAGE_SIZE         4096    // ESP32S3 uses 4KB sectors
#define FLASH_USER_START_ADDR   0x300000  // User data area start (3MB offset)

#define TAG "HAL_FLASH"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
#if defined( USE_FLASH_READ_MODIFY_WRITE ) || defined( MULTISTACK )
static uint8_t copy_page[FLASH_PAGE_SIZE] = { 0x00 };
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

// ESP32S3 doesn't need page/bank calculation functions like STM32
// Flash operations work directly with addresses

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
uint16_t hal_flash_get_page_size( )
{
    return FLASH_PAGE_SIZE;
}

uint8_t hal_flash_erase_page( uint32_t addr, uint8_t nb_page )
{
    esp_err_t err;
    uint32_t erase_size = nb_page * FLASH_PAGE_SIZE;
    
    ESP_LOGI(TAG, "Erasing %d pages at address 0x%lx", nb_page, addr);
    
    // Ensure address is aligned to sector boundary
    if (addr % FLASH_PAGE_SIZE != 0) {
        ESP_LOGE(TAG, "Address 0x%lx not aligned to sector boundary", addr);
        return FAIL;
    }
    
    err = esp_flash_erase_region(NULL, addr, erase_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase flash: %s", esp_err_to_name(err));
        return FAIL;
    }
    
    ESP_LOGI(TAG, "Successfully erased %d pages", nb_page);
    return SUCCESS;
}

uint32_t hal_flash_write_buffer( uint32_t addr, const uint8_t* buffer, uint32_t size )
{
    esp_err_t err;
    
    if (buffer == NULL || size == 0) {
        ESP_LOGE(TAG, "Invalid parameters for flash write");
        return 0;
    }
    
    ESP_LOGI(TAG, "Writing %lu bytes to address 0x%lx", size, addr);
    
    err = esp_flash_write(NULL, buffer, addr, size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write flash: %s", esp_err_to_name(err));
        return 0;
    }
    
    ESP_LOGI(TAG, "Successfully wrote %lu bytes", size);
    return size;
}

void hal_flash_read_buffer( uint32_t addr, uint8_t* buffer, uint32_t size )
{
    esp_err_t err;
    
    if (buffer == NULL || size == 0) {
        ESP_LOGE(TAG, "Invalid parameters for flash read");
        return;
    }
    
    ESP_LOGI(TAG, "Reading %lu bytes from address 0x%lx", size, addr);
    
    err = esp_flash_read(NULL, buffer, addr, size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read flash: %s", esp_err_to_name(err));
        memset(buffer, 0xFF, size);  // Fill with 0xFF on error (erased flash value)
        return;
    }
    
    ESP_LOGI(TAG, "Successfully read %lu bytes", size);
}

#if defined( USE_FLASH_READ_MODIFY_WRITE ) || defined( MULTISTACK )
void hal_flash_read_modify_write( uint32_t addr, const uint8_t* buffer, uint32_t size )
{
    if (buffer == NULL || size == 0) {
        ESP_LOGE(TAG, "Invalid parameters for flash read-modify-write");
        return;
    }
    
    uint32_t page_addr = addr & ~(FLASH_PAGE_SIZE - 1);  // Align to page boundary
    uint32_t page_offset = addr - page_addr;
    
    ESP_LOGI(TAG, "Read-modify-write: addr=0x%lx, size=%lu, page_addr=0x%lx, offset=%lu", 
             addr, size, page_addr, page_offset);
    
    // Check if we need to span multiple pages
    if (page_offset + size > FLASH_PAGE_SIZE) {
        ESP_LOGW(TAG, "Data spans multiple pages, only modifying first page");
        size = FLASH_PAGE_SIZE - page_offset;
    }
    
    // Read the entire page
    hal_flash_read_buffer(page_addr, copy_page, FLASH_PAGE_SIZE);
    
    // Modify the data in the page buffer
    memcpy(&copy_page[page_offset], buffer, size);
    
    // Erase the page
    if (hal_flash_erase_page(page_addr, 1) != SUCCESS) {
        ESP_LOGE(TAG, "Failed to erase page for read-modify-write");
        return;
    }
    
    // Write the modified page back
    if (hal_flash_write_buffer(page_addr, copy_page, FLASH_PAGE_SIZE) != FLASH_PAGE_SIZE) {
        ESP_LOGE(TAG, "Failed to write page for read-modify-write");
        return;
    }
    
    ESP_LOGI(TAG, "Read-modify-write completed successfully");
}
#endif  // USE_FLASH_READ_MODIFY_WRITE || MULTISTACK

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

// No private functions needed for ESP32S3 Flash HAL implementation

/* --- EOF ------------------------------------------------------------------ */
