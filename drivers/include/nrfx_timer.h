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

#ifndef NRFX_TIMER_H__
#define NRFX_TIMER_H__

#include <nrfx.h>
#include <haly/nrfy_timer.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrfx_timer Timer driver
 * @{
 * @ingroup nrf_timer
 * @brief   TIMER peripheral driver.
 */

/**
 * @brief Timer driver instance data structure.
 */
typedef struct
{
    NRF_TIMER_Type * p_reg;            ///< Pointer to the structure with TIMER peripheral instance registers.
    uint8_t          instance_id;      ///< Index of the driver instance. For internal use only.
    uint8_t          cc_channel_count; ///< Number of capture/compare channels.
} nrfx_timer_t;

/** @brief Macro for creating a timer driver instance. */
#define NRFX_TIMER_INSTANCE(id)                                   \
{                                                                 \
    .p_reg            = NRFX_CONCAT_2(NRF_TIMER, id),             \
    .instance_id      = NRFX_CONCAT_3(NRFX_TIMER, id, _INST_IDX), \
    .cc_channel_count = NRF_TIMER_CC_CHANNEL_COUNT(id),           \
}

#ifndef __NRFX_DOXYGEN__
enum {
    NRFX_INSTANCE_ENUM_LIST(TIMER)
    NRFX_TIMER_ENABLED_COUNT
};
#endif

/** @brief The configuration structure of the timer driver instance. */
typedef struct
{
    nrfy_timer_config_t nrfy_config;        ///< TIMER Configuration.
    uint8_t             interrupt_priority; ///< Interrupt priority.
    void *              p_context;          ///< Context passed to interrupt handler.
} nrfx_timer_config_t;

/**
 * @brief TIMER driver default configuration.
 *
 * This configuration sets up TIMER with the following options:
 * - prescaler: set to achieve maximum available frequency
 * - works as timer
 * - width: 16 bit
 */
#define NRFX_TIMER_DEFAULT_CONFIG                                 \
{                                                                 \
    .nrfy_config =                                                \
    {                                                             \
        .prescaler = 0,                                           \
        .mode      = NRF_TIMER_MODE_TIMER,                        \
        .bit_width = NRF_TIMER_BIT_WIDTH_16,                      \
    },                                                            \
    .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY, \
    .p_context          = NULL                                    \
}

/**
 * @brief Timer driver event handler type.
 *
 * @param[in] event_type Timer event.
 * @param[in] p_context  General purpose parameter set during initialization of
 *                       the timer. This parameter can be used to pass
 *                       additional information to the handler function, for
 *                       example, the timer ID.
 */
typedef void (* nrfx_timer_event_handler_t)(nrf_timer_event_t event_type,
                                            void            * p_context);

/**
 * @brief Function for initializing the timer.
 *
 * @param[in] p_instance          Pointer to the driver instance structure.
 * @param[in] p_config            Pointer to the structure with the initial configuration.
 * @param[in] timer_event_handler Event handler provided by the user.
 *                                Must not be NULL.
 *
 * @retval NRFX_SUCCESS             Initialization was successful.
 * @retval NRFX_ERROR_INVALID_STATE The instance is already initialized.
 */
nrfx_err_t nrfx_timer_init(nrfx_timer_t const *        p_instance,
                           nrfx_timer_config_t const * p_config,
                           nrfx_timer_event_handler_t  timer_event_handler);

/**
 * @brief Function for reconfiguring the timer.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] p_config   Pointer to the structure with the configuration.
 *
 * @retval NRFX_SUCCESS             Reconfiguration was successful.
 * @retval NRFX_ERROR_BUSY          The driver is enabled and cannot be reconfigured.
 * @retval NRFX_ERROR_INVALID_STATE The driver is uninitialized.
 */
nrfx_err_t nrfx_timer_reconfigure(nrfx_timer_t const *        p_instance,
                                  nrfx_timer_config_t const * p_config);

/**
 * @brief Function for uninitializing the timer.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrfx_timer_uninit(nrfx_timer_t const * p_instance);

/**
 * @brief Function for turning on the timer.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrfx_timer_enable(nrfx_timer_t const * p_instance);

/**
 * @brief Function for turning off the timer.
 *
 * The timer will allow to enter the lowest possible SYSTEM_ON state
 * only after this function is called.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrfx_timer_disable(nrfx_timer_t const * p_instance);

/**
 * @brief Function for checking the timer state.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 *
 * @retval true  Timer is enabled.
 * @retval false Timer is not enabled.
 */
bool nrfx_timer_is_enabled(nrfx_timer_t const * p_instance);

/**
 * @brief Function for pausing the timer.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrfx_timer_pause(nrfx_timer_t const * p_instance);

/**
 * @brief Function for resuming the timer.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrfx_timer_resume(nrfx_timer_t const * p_instance);

/**
 * @brief Function for clearing the timer.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrfx_timer_clear(nrfx_timer_t const * p_instance);

/**
 * @brief Function for incrementing the timer.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrfx_timer_increment(nrfx_timer_t const * p_instance);

/**
 * @brief Function for getting the prescaler setting for given frequency value.
 *
 * @param[in]  p_instance Pointer to the driver instance structure.
 * @param[in]  frequency  Desired frequency value.
 * @param[out] prescaler  Pointer to the variable to be filled with prescaler value
 *                        corresponding to desired frequency.
 *
 * @retval NRFX_SUCCESS             Operation was successful.
 * @retval NRFX_ERROR_INVALID_PARAM No prescaler values available for given frequency and instance pair.
 */
nrfx_err_t nrfx_timer_prescaler_calculate(nrfx_timer_t const * p_instance,
                                          uint32_t             frequency,
                                          uint32_t *           prescaler);

/**
 * @brief Function for returning the address of the specified timer task.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] timer_task Timer task.
 *
 * @return Task address.
 */
NRFX_STATIC_INLINE uint32_t nrfx_timer_task_address_get(nrfx_timer_t const * p_instance,
                                                        nrf_timer_task_t     timer_task);

/**
 * @brief Function for returning the address of the specified timer capture task.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] channel    Capture channel number.
 *
 * @return Task address.
 */
NRFX_STATIC_INLINE uint32_t nrfx_timer_capture_task_address_get(nrfx_timer_t const * p_instance,
                                                                uint32_t             channel);

/**
 * @brief Function for returning the address of the specified timer event.
 *
 * @param[in] p_instance  Pointer to the driver instance structure.
 * @param[in] timer_event Timer event.
 *
 * @return Event address.
 */
NRFX_STATIC_INLINE uint32_t nrfx_timer_event_address_get(nrfx_timer_t const * p_instance,
                                                         nrf_timer_event_t    timer_event);

/**
 * @brief Function for returning the address of the specified timer compare event.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] channel    Compare channel number.
 *
 * @return Event address.
 */
NRFX_STATIC_INLINE uint32_t nrfx_timer_compare_event_address_get(nrfx_timer_t const * p_instance,
                                                                 uint32_t             channel);

/**
 * @brief Function for capturing the timer value.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] cc_channel Capture channel number.
 *
 * @return Captured value.
 */
uint32_t nrfx_timer_capture(nrfx_timer_t const *   p_instance,
                            nrf_timer_cc_channel_t cc_channel);

/**
 * @brief Function for returning the capture value from the specified channel.
 *
 * Use this function to read channel values when PPI is used for capturing.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] cc_channel Capture channel number.
 *
 * @return Captured value.
 */
NRFX_STATIC_INLINE uint32_t nrfx_timer_capture_get(nrfx_timer_t const *   p_instance,
                                                   nrf_timer_cc_channel_t cc_channel);

/**
 * @brief Function for setting the timer channel in compare mode.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] cc_channel Compare channel number.
 * @param[in] cc_value   Compare value.
 * @param[in] enable_int Enable or disable the interrupt for the compare channel.
 */
void nrfx_timer_compare(nrfx_timer_t const *   p_instance,
                        nrf_timer_cc_channel_t cc_channel,
                        uint32_t               cc_value,
                        bool                   enable_int);

/**
 * @brief Function for setting the timer channel in the extended compare mode.
 *
 * @param[in] p_instance       Pointer to the driver instance structure.
 * @param[in] cc_channel       Compare channel number.
 * @param[in] cc_value         Compare value.
 * @param[in] timer_short_mask Shortcut between the compare event on the channel
 *                             and the timer task (STOP or CLEAR).
 * @param[in] enable_int       Enable or disable the interrupt for the compare channel.
 */
void nrfx_timer_extended_compare(nrfx_timer_t const *   p_instance,
                                 nrf_timer_cc_channel_t cc_channel,
                                 uint32_t               cc_value,
                                 nrf_timer_short_mask_t timer_short_mask,
                                 bool                   enable_int);

/**
 * @brief Function for checking whether specified @p frequency can be obtained for given timer
 *        instance.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] frequency  Frequency value in Hz to be checked.
 *
 * @retval true  Given frequency can be obtained.
 * @retval false Given frequency cannot be obtained.
 */
bool nrfx_timer_frequency_vaild_check(nrfx_timer_t const * p_instance,
                                      uint32_t             frequency);

/**
 * @brief Function for converting time in microseconds to timer ticks.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] time_us    Time in microseconds.
 *
 * @return Number of ticks.
 */
uint32_t nrfx_timer_us_to_ticks(nrfx_timer_t const * p_instance,
                                uint32_t             time_us);

/**
 * @brief Function for converting time in milliseconds to timer ticks.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] time_ms    Time in milliseconds.
 *
 * @return Number of ticks.
 */
uint32_t nrfx_timer_ms_to_ticks(nrfx_timer_t const * p_instance,
                                uint32_t             time_ms);

/**
 * @brief Function for enabling timer compare interrupt.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] channel    Compare channel.
 */
void nrfx_timer_compare_int_enable(nrfx_timer_t const * p_instance,
                                   uint32_t             channel);

/**
 * @brief Function for disabling timer compare interrupt.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] channel    Compare channel.
 */
void nrfx_timer_compare_int_disable(nrfx_timer_t const * p_instance,
                                    uint32_t             channel);

#ifndef NRFX_DECLARE_ONLY
NRFX_STATIC_INLINE uint32_t nrfx_timer_task_address_get(nrfx_timer_t const * p_instance,
                                                        nrf_timer_task_t     timer_task)
{
    return nrfy_timer_task_address_get(p_instance->p_reg, timer_task);
}

NRFX_STATIC_INLINE uint32_t nrfx_timer_capture_task_address_get(nrfx_timer_t const * p_instance,
                                                                uint32_t             channel)
{
    NRFX_ASSERT(channel < p_instance->cc_channel_count);
    return nrfy_timer_task_address_get(p_instance->p_reg, nrfy_timer_capture_task_get(channel));
}

NRFX_STATIC_INLINE uint32_t nrfx_timer_event_address_get(nrfx_timer_t const * p_instance,
                                                         nrf_timer_event_t    timer_event)
{
    return nrfy_timer_event_address_get(p_instance->p_reg, timer_event);
}

NRFX_STATIC_INLINE uint32_t nrfx_timer_compare_event_address_get(nrfx_timer_t const * p_instance,
                                                                 uint32_t             channel)
{
    NRFX_ASSERT(channel < p_instance->cc_channel_count);
    return nrfy_timer_event_address_get(p_instance->p_reg, nrfy_timer_compare_event_get(channel));
}

NRFX_STATIC_INLINE uint32_t nrfx_timer_capture_get(nrfx_timer_t const *   p_instance,
                                                   nrf_timer_cc_channel_t cc_channel)
{
    return nrfy_timer_cc_get(p_instance->p_reg, cc_channel);
}

#endif // NRFX_DECLARE_ONLY

/** @} */

/* Declare interrupt handlers for enabled instances. */
NRFX_INSTANCE_IRQ_HANDLERS_DECLARE(TIMER, timer)

#ifdef __cplusplus
}
#endif

#endif // NRFX_TIMER_H__

