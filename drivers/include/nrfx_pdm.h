/*
 * Copyright (c) 2015 - 2022, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NRFX_PDM_H__
#define NRFX_PDM_H__

#include <nrfx.h>
#include <haly/nrfy_pdm.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrfx_pdm PDM driver
 * @{
 * @ingroup nrf_pdm
 * @brief   Pulse Density Modulation (PDM) peripheral driver.
 */

/** @brief Maximum supported PDM buffer size. */
#define NRFX_PDM_MAX_BUFFER_SIZE 32767

/** @brief PDM interface driver configuration structure. */
typedef struct
{
    nrfy_pdm_config_t nrfy_config;        ///< Basic hardware configuration.
    uint8_t           interrupt_priority; ///< Interrupt priority.
    bool              skip_gpio_cfg;      ///< Skip GPIO configuration of pins.
                                          /**< When set to true, the driver does not modify
                                           *   any GPIO parameters of the used pins. Those
                                           *   parameters are supposed to be configured
                                           *   externally before the driver is initialized. */
} nrfx_pdm_config_t;

/** @brief PDM error type. */
typedef enum
{
    NRFX_PDM_NO_ERROR = 0,      ///< No error.
    NRFX_PDM_ERROR_OVERFLOW = 1 ///< Overflow error.
} nrfx_pdm_error_t;

/** @brief PDM event structure. */
typedef struct
{
    bool             buffer_requested; ///< Buffer request flag.
    int16_t *        buffer_released;  ///< Pointer to the released buffer. Can be NULL.
    nrfx_pdm_error_t error;            ///< Error type.
} nrfx_pdm_evt_t;

#if NRF_PDM_HAS_RATIO_CONFIG || defined(__NRFX_DOXYGEN__)
    /** @brief PDM additional ratio configuration. */
    #define NRFX_PDM_DEFAULT_EXTENDED_RATIO_CONFIG \
        .ratio = NRF_PDM_RATIO_64X,
#else
    #define NRFX_PDM_DEFAULT_EXTENDED_RATIO_CONFIG
#endif

#if NRF_PDM_HAS_MCLKCONFIG || defined(__NRFX_DOXYGEN__)
    /** @brief PDM additional master clock source configuration. */
    #define NRFX_PDM_DEFAULT_EXTENDED_MCLKSRC_CONFIG \
        .mclksrc = NRF_PDM_MCLKSRC_PCLK32M,
#else
    #define NRFX_PDM_DEFAULT_EXTENDED_MCLKSRC_CONFIG
#endif

/**
 * @brief PDM driver default configuration.
 *
 * This configuration sets up PDM with the following options:
 * - mono mode
 * - data sampled on the clock falling edge
 * - frequency: 1.032 MHz
 * - standard gain
 *
 * @param[in] _pin_clk CLK output pin.
 * @param[in] _pin_din DIN input pin.
 */
#define NRFX_PDM_DEFAULT_CONFIG(_pin_clk, _pin_din)             \
{                                                               \
    .nrfy_config =                                              \
    {                                                           \
        .mode               = NRF_PDM_MODE_MONO,                \
        .edge               = NRF_PDM_EDGE_LEFTFALLING,         \
        .pins               =                                   \
        {                                                       \
            .clk_pin        = _pin_clk,                         \
            .din_pin        = _pin_din,                         \
        },                                                      \
        .clock_freq         = NRF_PDM_FREQ_1032K,               \
        .gain_l             = NRF_PDM_GAIN_DEFAULT,             \
        .gain_r             = NRF_PDM_GAIN_DEFAULT,             \
        NRFX_PDM_DEFAULT_EXTENDED_RATIO_CONFIG                  \
        NRFX_PDM_DEFAULT_EXTENDED_MCLKSRC_CONFIG                \
    },                                                          \
    .interrupt_priority = NRFX_PDM_DEFAULT_CONFIG_IRQ_PRIORITY, \
}

/**
 * @brief Handler for the PDM interface ready events.
 *
 * This event handler is called on a buffer request, an error or when a buffer
 * is full and ready to be processed.
 *
 * @param[in] p_evt Pointer to the PDM event structure.
 */
typedef void (*nrfx_pdm_event_handler_t)(nrfx_pdm_evt_t const * p_evt);


/**
 * @brief Function for initializing the PDM interface.
 *
 * @param[in] p_config      Pointer to the structure with the initial configuration.
 * @param[in] event_handler Event handler provided by the user. Cannot be NULL.
 *
 * @retval NRFX_SUCCESS             Initialization was successful.
 * @retval NRFX_ERROR_INVALID_STATE The driver is already initialized.
 * @retval NRFX_ERROR_INVALID_PARAM Invalid configuration was specified.
 */
nrfx_err_t nrfx_pdm_init(nrfx_pdm_config_t const * p_config,
                         nrfx_pdm_event_handler_t  event_handler);


/**
 * @brief Function for reconfiguring the PDM interface.
 *
 * @param[in] p_config Pointer to the structure with the configuration.
 *
 * @retval NRFX_SUCCESS             Reconfiguration was successful.
 * @retval NRFX_ERROR_BUSY          There is ongoing sampling and driver cannot be reconfigured.
 * @retval NRFX_ERROR_INVALID_STATE The driver is not initialized.
 * @retval NRFX_ERROR_INVALID_PARAM Invalid configuration was specified.
 */
nrfx_err_t nrfx_pdm_reconfigure(nrfx_pdm_config_t const * p_config);

/**
 * @brief Function for uninitializing the PDM interface.
 *
 * This function stops PDM sampling, if it is in progress.
 */
void nrfx_pdm_uninit(void);

/**
 * @brief Function for getting the address of a PDM interface task.
 *
 * @param[in] task Task.
 *
 * @return Task address.
 */
NRFX_STATIC_INLINE uint32_t nrfx_pdm_task_address_get(nrf_pdm_task_t task);

/**
 * @brief Function for getting the state of the PDM interface.
 *
 * @retval true  The PDM interface is enabled.
 * @retval false The PDM interface is disabled.
 */
NRFX_STATIC_INLINE bool nrfx_pdm_enable_check(void);

/**
 * @brief Function for starting the PDM sampling.
 *
 * @retval NRFX_SUCCESS    Sampling was started successfully or was already in progress.
 * @retval NRFX_ERROR_BUSY Previous start/stop operation is in progress.
 */
nrfx_err_t nrfx_pdm_start(void);

/**
 * @brief Function for stopping the PDM sampling.
 *
 * When this function is called, the PDM interface is stopped after finishing
 * the current frame.
 * The event handler function might be called once more after calling this function.
 *
 * @retval NRFX_SUCCESS    Sampling was stopped successfully or was already stopped before.
 * @retval NRFX_ERROR_BUSY Previous start/stop operation is in progress.
 */
nrfx_err_t nrfx_pdm_stop(void);

/**
 * @brief Function for supplying the sample buffer.
 *
 * Call this function after every buffer request event.
 *
 * @param[in] buffer        Pointer to the receive buffer. Cannot be NULL.
 * @param[in] buffer_length Length of the receive buffer in 16-bit words.
 *
 * @retval NRFX_SUCCESS             The buffer was applied successfully.
 * @retval NRFX_ERROR_BUSY          The buffer was already supplied or the peripheral is currently being stopped.
 * @retval NRFX_ERROR_INVALID_STATE The driver was not initialized.
 * @retval NRFX_ERROR_INVALID_PARAM Invalid parameters were provided.
 */
nrfx_err_t nrfx_pdm_buffer_set(int16_t * buffer, uint16_t buffer_length);

#ifndef NRFX_DECLARE_ONLY
NRFX_STATIC_INLINE uint32_t nrfx_pdm_task_address_get(nrf_pdm_task_t task)
{
    return nrf_pdm_task_address_get(NRF_PDM0, task);
}

NRFX_STATIC_INLINE bool nrfx_pdm_enable_check(void)
{
    return nrf_pdm_enable_check(NRF_PDM0);
}
#endif // NRFX_DECLARE_ONLY

/** @} */


void nrfx_pdm_irq_handler(void);


#ifdef __cplusplus
}
#endif

#endif // NRFX_PDM_H__
