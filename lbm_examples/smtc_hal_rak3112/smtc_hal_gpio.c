/*!
 * \file      smtc_hal_gpio.c
 *
 * \brief     GPIO Hardware Abstraction Layer implementation
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

#include "smtc_hal_gpio.h"
#include "driver/gpio.h"  // ESP-IDF GPIO driver
#include "esp_err.h"      // ESP-IDF error types


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * Flag to track if GPIO ISR service is installed
 */
static bool gpio_isr_service_installed = false;

/*!
 * Flag to track global GPIO interrupt enable/disable state
 */
static bool gpio_global_irq_enabled = true;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */



/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

//
// MCU input pin Handling
//

void hal_gpio_init_in( const hal_gpio_pin_names_t pin, const hal_gpio_pull_mode_t pull_mode,
                       const hal_gpio_irq_mode_t irq_mode, hal_gpio_irq_t* irq )
{
    gpio_config_t io_conf = {};
    
    // Configure as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << pin);
    
    // Set pull-up/pull-down mode
    switch(pull_mode) {
        case BSP_GPIO_PULL_MODE_UP:
            io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            break;
        case BSP_GPIO_PULL_MODE_DOWN:
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
            io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
            break;
        case BSP_GPIO_PULL_MODE_NONE:
        default:
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            break;
    }
    
    // Set interrupt type
    switch(irq_mode) {
        case BSP_GPIO_IRQ_MODE_RISING:
            io_conf.intr_type = GPIO_INTR_POSEDGE;
            break;
        case BSP_GPIO_IRQ_MODE_FALLING:
            io_conf.intr_type = GPIO_INTR_NEGEDGE;
            break;
        case BSP_GPIO_IRQ_MODE_RISING_FALLING:
            io_conf.intr_type = GPIO_INTR_ANYEDGE;
            break;
        case BSP_GPIO_IRQ_MODE_OFF:
        default:
            io_conf.intr_type = GPIO_INTR_DISABLE;
            break;
    }
    
    // Apply configuration
    gpio_config(&io_conf);

    gpio_set_intr_type( pin , io_conf.intr_type );
    // Data consistency: ensure pin field in IRQ struct matches the actual configured pin
    // Future use: hal_gpio_irq_attach() function needs to know the specific pin number
    // Error prevention: prevent user from forgetting to set or setting wrong pin number
    if(irq != NULL && irq_mode != BSP_GPIO_IRQ_MODE_OFF) {
        irq->pin = pin;
    }

    hal_gpio_irq_attach( irq );
}

void hal_gpio_init_out( const hal_gpio_pin_names_t pin, const uint32_t value )
{
    gpio_config_t io_conf = {};
    
    // Configure as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    
    // Apply configuration
    gpio_config(&io_conf);
    
    // Set initial level
    gpio_set_level(pin, value);
}


void test_callback( void* arg )
{
    // Example callback function
    // This can be used to test GPIO interrupt functionality
    // printf("GPIO interrupt triggered!\n");
}   


void hal_gpio_irq_attach( const hal_gpio_irq_t* irq )
{
    if( ( irq != NULL ) && ( irq->callback != NULL ) )
    {
        printf("Attaching IRQ handler for pin %d\n", irq->pin);
        // Install GPIO ISR service (only once)
        if (!gpio_isr_service_installed) {
            //esp_err_t ret = gpio_install_isr_service(0);
            esp_err_t ret = gpio_install_isr_service(0);
            if (ret == ESP_OK) {
                gpio_isr_service_installed = true;
            }
        }
        

        // gpio_set_intr_type( irq->pin, GPIO_INTR_POSEDGE );
        // Add interrupt handler for specified pin
        gpio_isr_handler_add( irq->pin, irq->callback, irq->context );

        //gpio_isr_handler_add( irq->pin, test_callback, irq->context );

        // Enable pin interrupt if global interrupt is enabled
        if (gpio_global_irq_enabled) {
            gpio_intr_enable( irq->pin );
        }
    } 
}

void hal_gpio_irq_deatach( const hal_gpio_irq_t* irq )
{
    if( irq != NULL )
    {
        // Remove interrupt handler for specified pin
        gpio_isr_handler_remove( irq->pin );
        
        // Disable interrupt for this pin
        gpio_intr_disable( irq->pin );
    }
}

void hal_gpio_irq_enable( void )
{
    // ESP-IDF does not have global GPIO interrupt switch, this function is for specific use cases
    // Control through flag, actual interrupt enable happens in attach function
    // gpio_global_irq_enabled = true;
}

void hal_gpio_irq_disable( void )
{
    // ESP-IDF does not have global GPIO interrupt switch, this function is for specific use cases  
    // Control through flag, actual interrupt disable needs to be done per pin
    // gpio_global_irq_enabled = false;
}

//
// MCU pin state control
//

void hal_gpio_set_value( const hal_gpio_pin_names_t pin, const uint32_t value )
{
    // Set GPIO pin level
    gpio_set_level(pin, value);
}

uint32_t hal_gpio_get_value( const hal_gpio_pin_names_t pin )
{
    // Read GPIO pin level
    return gpio_get_level(pin);
}

void hal_gpio_clear_pending_irq( const hal_gpio_pin_names_t pin )
{
    // GPIO interrupt status is automatically cleared by hardware in ESP-IDF
    // When ISR executes, interrupt status is automatically cleared
    // If manual clear is needed, can use following method:
    // gpio_intr_disable(pin);
    // gpio_intr_enable(pin);
    (void)pin; // Avoid unused parameter warning
}

void hal_gpio_enable_clock( const hal_gpio_pin_names_t pin )
{
    // GPIO clock is already enabled at system startup in ESP32
    // ESP-IDF automatically manages GPIO module clock
    // No need to manually enable GPIO clock
    (void)pin; // Avoid unused parameter warning
}


/* --- EOF ------------------------------------------------------------------ */
