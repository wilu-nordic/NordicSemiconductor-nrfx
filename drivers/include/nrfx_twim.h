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

#ifndef NRFX_TWIM_H__
#define NRFX_TWIM_H__

#include <nrfx.h>
#include <nrfx_twi_twim.h>
#include <haly/nrfy_twim.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrfx_twim TWIM driver
 * @{
 * @ingroup nrf_twim
 * @brief   Two Wire Interface Master with EasyDMA (TWIM) peripheral driver.
 */

/** @brief Structure for the TWI master driver instance. */
typedef struct
{
    NRF_TWIM_Type * p_twim;       ///< Pointer to a structure with TWIM registers.
    uint8_t         drv_inst_idx; ///< Index of the driver instance. For internal use only.
} nrfx_twim_t;

/** @brief Macro for creating a TWI master driver instance. */
#define NRFX_TWIM_INSTANCE(id)                               \
{                                                            \
    .p_twim       = NRFX_CONCAT_2(NRF_TWIM, id),             \
    .drv_inst_idx = NRFX_CONCAT_3(NRFX_TWIM, id, _INST_IDX), \
}

#ifndef __NRFX_DOXYGEN__
enum {
    NRFX_INSTANCE_ENUM_LIST(TWIM)
    NRFX_TWIM_ENABLED_COUNT
};
#endif

/** @brief Structure for the TWI master driver instance configuration. */
typedef struct
{
    nrfy_twim_config_t nrfy_config;        ///< TWIM configuration.
    uint8_t            interrupt_priority; ///< Interrupt priority.
    bool               hold_bus_uninit;    ///< Hold pull up state on GPIO pins after uninit.
    bool               skip_gpio_cfg;      ///< Skip GPIO configuration of pins.
                                           /**< When set to true, the driver does not modify
                                            *   any GPIO parameters of the used pins. Those
                                            *   parameters are supposed to be configured
                                            *   externally before the driver is initialized. */
} nrfx_twim_config_t;

/**
 * @brief TWIM driver default configuration.
 *
 * This configuration sets up TWIM with the following options:
 * - clock frequency: 100 kHz
 * - disable bus holding after uninit
 *
 * @param[in] _pin_scl SCL pin.
 * @param[in] _pin_sda SDA pin.
 */
#define NRFX_TWIM_DEFAULT_CONFIG(_pin_scl, _pin_sda)             \
{                                                                \
    .nrfy_config =                                               \
    {                                                            \
        .pins =                                                  \
        {                                                        \
            .scl_pin    = _pin_scl,                              \
            .sda_pin    = _pin_sda                               \
        },                                                       \
        .frequency      = NRF_TWIM_FREQ_100K                     \
    },                                                           \
    .interrupt_priority = NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY, \
    .hold_bus_uninit    = false,                                 \
}

/** @brief Flag indicating that TX buffer address will be incremented after the transfer. */
#define NRFX_TWIM_FLAG_TX_POSTINC             (1UL << 0)
/** @brief Flag indicating that RX buffer address will be incremented after the transfer. */
#define NRFX_TWIM_FLAG_RX_POSTINC             (1UL << 1)
/** @brief Flag indicating that the interrupt after each transfer will be suppressed, and the event handler will not be called. */
#define NRFX_TWIM_FLAG_NO_XFER_EVT_HANDLER    (1UL << 2)
/** @brief Flag indicating that the transfer will be set up, but not started. */
#define NRFX_TWIM_FLAG_HOLD_XFER              (1UL << 3)
/** @brief Flag indicating that the transfer will be executed multiple times. */
#define NRFX_TWIM_FLAG_REPEATED_XFER          (1UL << 4)
/** @brief Flag indicating that the TX transfer will not end with a stop condition. */
#define NRFX_TWIM_FLAG_TX_NO_STOP             (1UL << 5)
/** @brief Flag indicating that checks for spurious STOP condition will not be performed. */
#define NRFX_TWIM_FLAG_NO_SPURIOUS_STOP_CHECK (1UL << 6)

/** @brief TWI master driver event types. */
typedef enum
{
    NRFX_TWIM_EVT_DONE,         ///< Transfer completed event.
    NRFX_TWIM_EVT_ADDRESS_NACK, ///< Error event: NACK received after sending the address.
    NRFX_TWIM_EVT_DATA_NACK,    ///< Error event: NACK received after sending a data byte.
    NRFX_TWIM_EVT_OVERRUN,      ///< Error event: The unread data is replaced by new data.
    NRFX_TWIM_EVT_BUS_ERROR     ///< Error event: An unexpected transition occurred on the bus.
} nrfx_twim_evt_type_t;

/** @brief TWI master driver transfer types. */
typedef enum
{
    NRFX_TWIM_XFER_TX,   ///< TX transfer.
    NRFX_TWIM_XFER_RX,   ///< RX transfer.
    NRFX_TWIM_XFER_TXRX, ///< TX transfer followed by RX transfer with repeated start.
    NRFX_TWIM_XFER_TXTX  ///< TX transfer followed by TX transfer with repeated start.
} nrfx_twim_xfer_type_t;

/** @brief Structure for a TWI transfer descriptor. */
typedef struct
{
    nrfx_twim_xfer_type_t type;             ///< Type of transfer.
    uint8_t               address;          ///< Slave address.
    nrfy_twim_xfer_desc_t primary_buffer;   ///< Primary structure describing a TWIM transfer.
    nrfy_twim_xfer_desc_t secondary_buffer; ///< Secondary structure describing a TWIM transfer.
} nrfx_twim_xfer_desc_t;

/** @brief Macro for setting the transfer descriptor. */
#define NRFX_TWIM_XFER_DESC(transfer, addr, p_buf1, buf_len1, p_buf2, buf_len2) \
{                                                                               \
    .type             = (transfer),                                             \
    .address          = (addr),                                                 \
    .primary_buffer   =                                                         \
    {                                                                           \
            .p_buffer = (p_buf1),                                               \
            .length   = (buf_len1)                                              \
    },                                                                          \
    .secondary_buffer =                                                         \
    {                                                                           \
            .p_buffer = (p_buf2),                                               \
            .length   = (buf_len2)                                              \
    }                                                                           \
}

/** @brief Macro for setting the TX transfer descriptor. */
#define NRFX_TWIM_XFER_DESC_TX(addr, p_data, length) \
        NRFX_TWIM_XFER_DESC(NRFX_TWIM_XFER_TX, addr, p_data, length, NULL, 0)

/** @brief Macro for setting the RX transfer descriptor. */
#define NRFX_TWIM_XFER_DESC_RX(addr, p_data, length) \
        NRFX_TWIM_XFER_DESC(NRFX_TWIM_XFER_RX, addr, p_data, length, NULL, 0)

/** @brief Macro for setting the TX-RX transfer descriptor. */
#define NRFX_TWIM_XFER_DESC_TXRX(addr, p_tx, tx_len, p_rx, rx_len) \
        NRFX_TWIM_XFER_DESC(NRFX_TWIM_XFER_TXRX, addr, p_tx, tx_len, p_rx, rx_len)

/** @brief Macro for setting the TX-TX transfer descriptor. */
#define NRFX_TWIM_XFER_DESC_TXTX(addr, p_tx, tx_len, p_tx2, tx_len2) \
        NRFX_TWIM_XFER_DESC(NRFX_TWIM_XFER_TXTX, addr, p_tx, tx_len, p_tx2, tx_len2)

/** @brief Structure for a TWI event. */
typedef struct
{
    nrfx_twim_evt_type_t  type;      ///< Event type.
    nrfx_twim_xfer_desc_t xfer_desc; ///< Transfer details.
} nrfx_twim_evt_t;

/** @brief TWI event handler prototype. */
typedef void (* nrfx_twim_evt_handler_t)(nrfx_twim_evt_t const * p_event,
                                         void *                  p_context);

/**
 * @brief Function for initializing the TWI driver instance.
 *
 * @param[in] p_instance    Pointer to the driver instance structure.
 * @param[in] p_config      Pointer to the structure with the initial configuration.
 * @param[in] event_handler Event handler provided by the user. If NULL, blocking mode is enabled.
 * @param[in] p_context     Context passed to event handler.
 *
 * @warning On nRF5340, 1 MHz setting is supported only on the dedicated pins. See the chapter
 *          <a href=@nRF5340pinAssignmentsURL>Pin assignments</a> in the Product Specification.
 *
 * @retval NRFX_SUCCESS             Initialization was successful.
 * @retval NRFX_ERROR_INVALID_STATE The driver is in invalid state.
 * @retval NRFX_ERROR_INVALID_PARAM Requested frequency is not available on the specified pins.
 * @retval NRFX_ERROR_BUSY          Some other peripheral with the same
 *                                  instance ID is already in use. This is
 *                                  possible only if @ref nrfx_prs module
 *                                  is enabled.
 */
nrfx_err_t nrfx_twim_init(nrfx_twim_t const *        p_instance,
                          nrfx_twim_config_t const * p_config,
                          nrfx_twim_evt_handler_t    event_handler,
                          void *                     p_context);

/**
 * @brief Function for reconfiguring the TWI instance.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] p_config   Pointer to the structure with the configuration.
 *
 * @retval NRFX_SUCCESS             Reconfiguration was successful.
 * @retval NRFX_ERROR_BUSY          The driver is during transaction.
 * @retval NRFX_ERROR_INVALID_STATE The driver is uninitialized.
 */
nrfx_err_t nrfx_twim_reconfigure(nrfx_twim_t const *        p_instance,
                                 nrfx_twim_config_t const * p_config);

/**
 * @brief Function for uninitializing the TWI instance.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrfx_twim_uninit(nrfx_twim_t const * p_instance);

/**
 * @brief Function for enabling the TWI instance.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrfx_twim_enable(nrfx_twim_t const * p_instance);

/**
 * @brief Function for disabling the TWI instance.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrfx_twim_disable(nrfx_twim_t const * p_instance);

/**
 * @brief Function for performing a TWI transfer.
 *
 * The following transfer types can be configured (@ref nrfx_twim_xfer_desc_t.type):
 * - @ref NRFX_TWIM_XFER_TXRX - Write operation followed by a read operation (without STOP condition in between).
 * - @ref NRFX_TWIM_XFER_TXTX - Write operation followed by a write operation (without STOP condition in between).
 * - @ref NRFX_TWIM_XFER_TX - Write operation (with or without STOP condition).
 * - @ref NRFX_TWIM_XFER_RX - Read operation  (with STOP condition).
 *
 * @note TX-RX and TX-TX transfers are supported only in non-blocking mode.
 *
 * Additional options are provided using the flags parameter:
 * - @ref NRFX_TWIM_FLAG_TX_POSTINC and @ref NRFX_TWIM_FLAG_RX_POSTINC - Post-incrementation of buffer addresses.
 * - @ref NRFX_TWIM_FLAG_NO_XFER_EVT_HANDLER - No user event handler after the transfer completion. In most cases, this also means no interrupt at the end of the transfer.
 * - @ref NRFX_TWIM_FLAG_HOLD_XFER - Driver is not starting the transfer. Use this flag if the transfer is triggered externally by PPI.
 *   Use @ref nrfx_twim_start_task_get to get the address of the start task.
 * - @ref NRFX_TWIM_FLAG_REPEATED_XFER - Prepare for repeated transfers. You can set up a number of transfers that will be triggered externally (for example by PPI).
 *   An example is a TXRX transfer with the options @ref NRFX_TWIM_FLAG_RX_POSTINC, @ref NRFX_TWIM_FLAG_NO_XFER_EVT_HANDLER, and @ref NRFX_TWIM_FLAG_REPEATED_XFER.
 *   After the transfer is set up, a set of transfers can be triggered by PPI that will read, for example, the same register of an
 *   external component and put it into a RAM buffer without any interrupts. @ref nrfx_twim_stopped_event_get can be used to get the
 *   address of the STOPPED event, which can be used to count the number of transfers. If @ref NRFX_TWIM_FLAG_REPEATED_XFER is used,
 *   the driver does not set the driver instance into busy state, so you must ensure that the next transfers are set up
 *   when TWIM is not active.
 * - @ref NRFX_TWIM_FLAG_TX_NO_STOP - No stop condition after the TX transfer.
 * - @ref NRFX_TWIM_FLAG_NO_SPURIOUS_STOP_CHECK - Checks for spurious STOP conditions are disabled.
 *        Used together with @ref NRFX_TWIM_FLAG_NO_XFER_EVT_HANDLER can result in lower power consumption
 *        when transfers are triggered externally and CPU is sleeping.
 *        Use only with I2C standard-compliant slave devices.
 *
 * @note
 * Some flag combinations are invalid:
 * - @ref NRFX_TWIM_FLAG_TX_NO_STOP with @ref nrfx_twim_xfer_desc_t.type different than @ref NRFX_TWIM_XFER_TX
 * - @ref NRFX_TWIM_FLAG_REPEATED_XFER with @ref nrfx_twim_xfer_desc_t.type set to @ref NRFX_TWIM_XFER_TXTX
 *
 * If @ref nrfx_twim_xfer_desc_t.type is set to @ref NRFX_TWIM_XFER_TX and the @ref NRFX_TWIM_FLAG_TX_NO_STOP and @ref NRFX_TWIM_FLAG_REPEATED_XFER
 * flags are set, two tasks must be used to trigger a transfer: TASKS_RESUME followed by TASKS_STARTTX. If no stop condition is generated,
 * TWIM is in SUSPENDED state. Therefore, it must be resumed before the transfer can be started.
 *
 * @note Peripherals using EasyDMA (including TWIM) require the transfer buffers
 *       to be placed in the Data RAM region. If this condition is not met,
 *       this function will fail with the error code NRFX_ERROR_INVALID_ADDR.
 *
 * @param[in] p_instance  Pointer to the driver instance structure.
 * @param[in] p_xfer_desc Pointer to the transfer descriptor.
 * @param[in] flags       Transfer options (0 for default settings).
 *
 * @retval NRFX_SUCCESS                   The procedure is successful.
 * @retval NRFX_ERROR_BUSY                The driver is not ready for a new transfer.
 * @retval NRFX_ERROR_NOT_SUPPORTED       The provided parameters are not supported.
 * @retval NRFX_ERROR_INTERNAL            An unexpected transition occurred on the bus.
 * @retval NRFX_ERROR_INVALID_ADDR        The provided buffers are not placed in the Data RAM region.
 * @retval NRFX_ERROR_DRV_TWI_ERR_OVERRUN The unread data is replaced by new data.
 * @retval NRFX_ERROR_DRV_TWI_ERR_ANACK   NACK is received after sending the address.
 * @retval NRFX_ERROR_DRV_TWI_ERR_DNACK   NACK is received after sending a data byte.
 */
nrfx_err_t nrfx_twim_xfer(nrfx_twim_t           const * p_instance,
                          nrfx_twim_xfer_desc_t const * p_xfer_desc,
                          uint32_t                      flags);

/**
 * @brief Function for checking the TWI driver state.
 *
 * @param[in] p_instance TWI instance.
 *
 * @retval true  The TWI driver is currently busy performing a transfer.
 * @retval false The TWI driver is ready for a new transfer.
 */
bool nrfx_twim_is_busy(nrfx_twim_t const * p_instance);


/**
 * @brief Function for returning the address of a TWIM start task.
 *
 * This function is to be used if @ref nrfx_twim_xfer was called with the flag @ref NRFX_TWIM_FLAG_HOLD_XFER.
 * In that case, the transfer is not started by the driver, but it must be started externally by PPI.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] xfer_type  Transfer type used in the last call of the @ref nrfx_twim_xfer function.
 *
 * @return Start task address (TX or RX) depending on the value of xfer_type.
 */
uint32_t nrfx_twim_start_task_get(nrfx_twim_t const * p_instance, nrfx_twim_xfer_type_t xfer_type);

/**
 * @brief Function for returning the address of a STOPPED TWIM event.
 *
 * A STOPPED event can be used to detect the end of a transfer if the @ref NRFX_TWIM_FLAG_NO_XFER_EVT_HANDLER
 * option is used.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 *
 * @return STOPPED event address.
 */
uint32_t nrfx_twim_stopped_event_get(nrfx_twim_t const * p_instance);

/**
 * @brief Function for recovering the bus.
 *
 * This function checks if the bus is not stuck because of a slave holding the SDA line in the low state,
 * and if needed it performs required number of pulses on the SCL line to make the slave release the SDA line.
 * Finally, the function generates a STOP condition on the bus to put it into a known state.
 *
 * @note This function can be used only if the TWIM driver is uninitialized.
 *
 * @param[in] scl_pin SCL pin number.
 * @param[in] sda_pin SDA pin number.
 *
 * @retval NRFX_SUCCESS        Bus recovery was successful.
 * @retval NRFX_ERROR_INTERNAL Bus recovery failed.
 */
NRFX_STATIC_INLINE nrfx_err_t nrfx_twim_bus_recover(uint32_t scl_pin, uint32_t sda_pin);

#ifndef NRFX_DECLARE_ONLY
NRFX_STATIC_INLINE nrfx_err_t nrfx_twim_bus_recover(uint32_t scl_pin, uint32_t sda_pin)
{
    return nrfx_twi_twim_bus_recover(scl_pin, sda_pin);
}
#endif

/** @} */

/* Declare interrupt handlers for enabled instances. */
NRFX_INSTANCE_IRQ_HANDLERS_DECLARE(TWIM, twim)


#ifdef __cplusplus
}
#endif

#endif // NRFX_TWIM_H__
