/*!
 * \file      smtc_hal_lp_timer.c
 *
 * \brief     Implements Low Power Timer utilities functions.
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

#include "smtc_hal_lp_timer.h"
#include "smtc_hal_mcu.h"

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_timer.h"
#include "esp_log.h"
#include <stdio.h>  // For printf

// Use printf for reliable output
// #define HAL_TIMER_PRINTF(fmt, ...) printf("[HAL_LP_TIMER] " fmt "\n", ##__VA_ARGS__)
// #define HAL_TIMER_PRINT(msg) printf("[HAL_LP_TIMER] " msg "\n")

#define HAL_TIMER_PRINTF(fmt, ...)
#define HAL_TIMER_PRINT(msg)

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define HAL_LP_TIMER_NB 2  //!< Number of supported low power timers

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct {
    esp_timer_handle_t timer_handle;
    hal_lp_timer_irq_t irq_context;
    bool initialized;
    bool enabled;
} lp_timer_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lp_timer_context_t lp_timers[HAL_LP_TIMER_NB] = {0};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void lp_timer_callback(void* arg);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_lp_timer_init( hal_lp_timer_id_t id )
{
    if (id >= HAL_LP_TIMER_NB) {
        HAL_TIMER_PRINTF("Invalid timer ID: %d", id);
        return;
    }
    
    if (!lp_timers[id].initialized) {
        HAL_TIMER_PRINTF("Initializing LP Timer %d", id);
        lp_timers[id].timer_handle = NULL;
        lp_timers[id].initialized = true;
        lp_timers[id].enabled = false;

        hal_lp_timer_irq_enable(id);
    }
}

void hal_lp_timer_start( hal_lp_timer_id_t id, const uint32_t milliseconds, const hal_lp_timer_irq_t* tmr_irq )
{
    if (id >= HAL_LP_TIMER_NB) {
        HAL_TIMER_PRINTF("Invalid timer ID: %d", id);
        return;
    }
    
    if (!lp_timers[id].initialized) {
        hal_lp_timer_init(id);
    }
    
    // Stop any existing timer
    hal_lp_timer_stop(id);
    
    // Store IRQ context
    if (tmr_irq) {
        lp_timers[id].irq_context = *tmr_irq;
    }
    
    // Create timer configuration
    esp_timer_create_args_t timer_args = {
        .callback = lp_timer_callback,
        .arg = (void*)((uintptr_t)id),
        .name = (id == HAL_LP_TIMER_ID_1) ? "lp_timer_1" : "lp_timer_2"
    };
    
    esp_err_t err = esp_timer_create(&timer_args, &lp_timers[id].timer_handle);
    if (err != ESP_OK) {
        HAL_TIMER_PRINTF("Failed to create timer %d: %s", id, esp_err_to_name(err));
        return;
    }
    
    // Start the timer
    uint64_t timeout_us = (uint64_t)milliseconds * 1000;
    err = esp_timer_start_once(lp_timers[id].timer_handle, timeout_us);
    if (err != ESP_OK) {
        HAL_TIMER_PRINTF("Failed to start timer %d: %s", id, esp_err_to_name(err));
        esp_timer_delete(lp_timers[id].timer_handle);
        lp_timers[id].timer_handle = NULL;
    } else {
        HAL_TIMER_PRINTF("Started LP Timer %d for %lu ms", id, milliseconds);
    }
}

void hal_lp_timer_stop( hal_lp_timer_id_t id )
{
    if (id >= HAL_LP_TIMER_NB) {
        HAL_TIMER_PRINTF("Invalid timer ID: %d", id);
        return;
    }
    
    if (lp_timers[id].timer_handle) {
        esp_timer_stop(lp_timers[id].timer_handle);
        esp_timer_delete(lp_timers[id].timer_handle);
        lp_timers[id].timer_handle = NULL;
        HAL_TIMER_PRINTF("Stopped LP Timer %d", id);
    }
}

void hal_lp_timer_irq_enable( hal_lp_timer_id_t id )
{
    if (id >= HAL_LP_TIMER_NB) {
        HAL_TIMER_PRINTF("Invalid timer ID: %d", id);
        return;
    }
    
    lp_timers[id].enabled = true;
    HAL_TIMER_PRINTF("Enabled IRQ for LP Timer %d", id);
}

void hal_lp_timer_irq_disable( hal_lp_timer_id_t id )
{
    if (id >= HAL_LP_TIMER_NB) {
        HAL_TIMER_PRINTF("Invalid timer ID: %d", id);
        return;
    }
    
    lp_timers[id].enabled = false;
    HAL_TIMER_PRINTF("Disabled IRQ for LP Timer %d", id);
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lp_timer_callback(void* arg)
{
    hal_lp_timer_id_t id = (hal_lp_timer_id_t)((uintptr_t)arg);
    
    if (id >= HAL_LP_TIMER_NB) {
        HAL_TIMER_PRINTF("Invalid timer ID in callback: %d", id);
        return;
    }
    
    // Check if interrupts are enabled for this timer
    if (!lp_timers[id].enabled) {
        HAL_TIMER_PRINTF("Timer %d callback called but IRQ disabled", id);
        return;
    }
    
    // Call the user callback if it exists
    if (lp_timers[id].irq_context.callback) {
        HAL_TIMER_PRINTF("Calling user callback for LP Timer %d", id);
        lp_timers[id].irq_context.callback(lp_timers[id].irq_context.context);
    }
    
    // Clean up the timer handle after one-shot timer expires
    if (lp_timers[id].timer_handle) {
        esp_timer_delete(lp_timers[id].timer_handle);
        lp_timers[id].timer_handle = NULL;
    }
}

/* --- EOF ------------------------------------------------------------------ */
