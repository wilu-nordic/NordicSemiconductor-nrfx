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

#include <nrfx.h>

#if NRFX_CHECK(NRFX_SPIM_ENABLED)
#if !NRFX_FEATURE_PRESENT(NRFX_SPIM, _ENABLED)
#error "No enabled SPIM instances. Check <nrfx_config.h>."
#endif

#include <nrfx_spim.h>
#include "prs/nrfx_prs.h"

#define NRFX_LOG_MODULE SPIM
#include <nrfx_log.h>

#if NRFX_CHECK(NRFX_SPIM_EXTENDED_ENABLED) && !NRFY_SPIM_HAS_EXTENDED
#error "Extended options are not available in the SoC currently in use."
#endif

#define SPIMX_LENGTH_VALIDATE(periph_name, prefix, i, drv_inst_idx, rx_len, tx_len) \
    (((drv_inst_idx) == NRFX_CONCAT(NRFX_, periph_name, prefix, i, _INST_IDX)) && \
     NRFX_EASYDMA_LENGTH_VALIDATE(NRFX_CONCAT(periph_name, prefix, i), rx_len, tx_len))

#define SPIM_LENGTH_VALIDATE(drv_inst_idx, rx_len, tx_len)    \
        (NRFX_FOREACH_ENABLED(SPIM, SPIMX_LENGTH_VALIDATE, (||), (0), drv_inst_idx, rx_len, tx_len))

#define SPIMX_HW_CSN_PRESENT_VALIDATE(periph_name, prefix, i, drv_inst_idx)         \
    (((drv_inst_idx) == NRFX_CONCAT(NRFX_, periph_name, prefix, i, _INST_IDX)) && \
     NRFX_CONCAT(periph_name, prefix, i, _FEATURE_HARDWARE_CSN_PRESENT))

#define SPIM_HW_CSN_PRESENT_VALIDATE(drv_inst_idx)    \
        (NRFX_FOREACH_ENABLED(SPIM, SPIMX_HW_CSN_PRESENT_VALIDATE, (||), (0), drv_inst_idx))

#define SPIMX_DCX_PRESENT_VALIDATE(periph_name, prefix, i, drv_inst_idx)            \
    (((drv_inst_idx) == NRFX_CONCAT(NRFX_, periph_name, prefix, i, _INST_IDX)) && \
     NRFX_CONCAT(periph_name, prefix, i, _FEATURE_DCX_PRESENT))

#define SPIM_DCX_PRESENT_VALIDATE(drv_inst_idx)    \
        (NRFX_FOREACH_ENABLED(SPIM, SPIMX_DCX_PRESENT_VALIDATE, (||), (0), drv_inst_idx))

#define SPIMX_SUPPORTED_FREQ_VALIDATE(periph_name, prefix, i, drv_inst_idx, freq)                \
    (                                                                                            \
    ((drv_inst_idx) == NRFX_CONCAT(NRFX_, periph_name, prefix, i, _INST_IDX)) &&                 \
    (                                                                                            \
        (((freq) != NRF_SPIM_FREQ_16M) && ((freq) != NRF_SPIM_FREQ_32M)) ||                      \
        (((freq) == NRF_SPIM_FREQ_16M) && ((NRFX_CONCAT(periph_name, prefix, i, _MAX_DATARATE) >= 16))) || \
        (((freq) == NRF_SPIM_FREQ_32M) && ((NRFX_CONCAT(periph_name, prefix, i, _MAX_DATARATE) >= 32)))    \
    )                                                                                            \
    )

#define SPIM_SUPPORTED_FREQ_VALIDATE(drv_inst_idx, freq)    \
        (NRFX_FOREACH_ENABLED(SPIM, SPIMX_SUPPORTED_FREQ_VALIDATE, (||), (0), drv_inst_idx, freq))

// Requested pin can either match dedicated pin or be not connected at all.
#define SPIM_DEDICATED_PIN_VALIDATE(requested_pin, supported_pin) \
    (((requested_pin) == NRF_SPIM_PIN_NOT_CONNECTED) || ((requested_pin) == (supported_pin)))

#if !defined(USE_WORKAROUND_FOR_ANOMALY_195) && \
    defined(NRF52840_XXAA) && NRFX_CHECK(NRFX_SPIM3_ENABLED)
// Enable workaround for nRF52840 anomaly 195 (SPIM3 continues to draw current after disable).
#define USE_WORKAROUND_FOR_ANOMALY_195 1
#endif

// Control block - driver instance local data.
typedef struct
{
    nrfx_spim_evt_handler_t handler;
    void *                  p_context;
    nrfx_spim_evt_t         evt;  // Keep the struct that is ready for event handler. Less memcpy.
    nrfx_drv_state_t        state;
    volatile bool           transfer_in_progress;
    bool                    skip_gpio_cfg  : 1;
    bool                    ss_active_high;
    uint32_t                ss_pin;
} spim_control_block_t;
static spim_control_block_t m_cb[NRFX_SPIM_ENABLED_COUNT];

#if NRFX_CHECK(NRFX_SPIM3_NRF52840_ANOMALY_198_WORKAROUND_ENABLED)

// Workaround for nRF52840 anomaly 198: SPIM3 transmit data might be corrupted.

static uint32_t m_anomaly_198_preserved_value;

static void anomaly_198_enable(uint8_t const * p_buffer, size_t buf_len)
{
    m_anomaly_198_preserved_value = *((volatile uint32_t *)0x40000E00);

    if (buf_len == 0)
    {
        return;
    }
    uint32_t buffer_end_addr = ((uint32_t)p_buffer) + buf_len;
    uint32_t block_addr      = ((uint32_t)p_buffer) & ~0x1FFF;
    uint32_t block_flag      = (1UL << ((block_addr >> 13) & 0xFFFF));
    uint32_t occupied_blocks = 0;

    if (block_addr >= 0x20010000)
    {
        occupied_blocks = (1UL << 8);
    }
    else
    {
        do {
            occupied_blocks |= block_flag;
            block_flag <<= 1;
            block_addr  += 0x2000;
        } while ((block_addr < buffer_end_addr) && (block_addr < 0x20012000));
    }

    *((volatile uint32_t *)0x40000E00) = occupied_blocks;
}

static void anomaly_198_disable(void)
{
    *((volatile uint32_t *)0x40000E00) = m_anomaly_198_preserved_value;
}
#endif // NRFX_CHECK(NRFX_SPIM3_NRF52840_ANOMALY_198_WORKAROUND_ENABLED)

static void spim_abort(NRF_SPIM_Type * p_spim, spim_control_block_t * p_cb)
{
    nrfy_spim_abort(p_spim, NULL);
    bool stopped;
    uint32_t stopped_mask = NRFY_EVENT_TO_INT_BITMASK(NRF_SPIM_EVENT_STOPPED);
    NRFX_WAIT_FOR(nrfy_spim_events_process(p_spim, stopped_mask, NULL), 100, 1, stopped);
    if (!stopped)
    {
        NRFX_LOG_ERROR("Failed to stop instance with base address: %p.", (void *)p_spim);
    }
    p_cb->transfer_in_progress = false;
}

static void configure_pins(nrfx_spim_t const *        p_instance,
                           nrfx_spim_config_t const * p_config)
{
    nrfy_spim_config_t const * p_nrfy_config = &p_config->nrfy_config;
    spim_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    uint32_t ss_pin_to_configure;
    bool     ss_active_high;
#if NRFX_CHECK(NRFX_SPIM_EXTENDED_ENABLED)
    nrfy_spim_ext_config_t const * p_ext_config = &p_nrfy_config->ext_config;
    if (p_nrfy_config->ext_enable && (p_ext_config->pins.csn_pin != NRF_SPIM_PIN_NOT_CONNECTED))
    {
        ss_pin_to_configure = p_ext_config->pins.csn_pin;
        ss_active_high = (p_ext_config->csn_pol == NRF_SPIM_CSN_POL_HIGH) ? true : false;
    }
    else
#endif
    {
        ss_pin_to_configure  = p_config->sw_ss_pin;
        ss_active_high       = p_config->sw_ss_active_high;
        p_cb->ss_active_high = ss_active_high;
    }

    if (!p_config->skip_gpio_cfg)
    {
        // Configure pins used by the peripheral:
        // - SCK - output with initial value corresponding with the SPI mode used:
        //   0 - for modes 0 and 1 (CPOL = 0), 1 - for modes 2 and 3 (CPOL = 1);
        //   according to the reference manual guidelines this pin and its input
        //   buffer must always be connected for the SPI to work.
        if (p_nrfy_config->mode <= NRF_SPIM_MODE_1)
        {
            nrfy_gpio_pin_clear(p_nrfy_config->pins.sck_pin);
        }
        else
        {
            nrfy_gpio_pin_set(p_nrfy_config->pins.sck_pin);
        }

        nrf_gpio_pin_drive_t pin_drive;
        // Configure pin drive - high drive for 32 MHz clock frequency.
#if NRF_SPIM_HAS_32_MHZ_FREQ
        pin_drive = (p_nrfy_config->frequency == NRF_SPIM_FREQ_32M) ? NRF_GPIO_PIN_H0H1 : NRF_GPIO_PIN_S0S1;
#else
        pin_drive = NRF_GPIO_PIN_S0S1;
#endif

        nrfy_gpio_cfg(p_nrfy_config->pins.sck_pin,
                      NRF_GPIO_PIN_DIR_OUTPUT,
                      NRF_GPIO_PIN_INPUT_CONNECT,
                      NRF_GPIO_PIN_NOPULL,
                      pin_drive,
                      NRF_GPIO_PIN_NOSENSE);
#if NRF_GPIO_HAS_CLOCKPIN
        nrfy_gpio_pin_clock_set(p_nrfy_config->pins.sck_pin, true);
#endif
        // - MOSI (optional) - output with initial value 0
        if (p_nrfy_config->pins.mosi_pin != NRF_SPIM_PIN_NOT_CONNECTED)
        {
            nrfy_gpio_pin_clear(p_nrfy_config->pins.mosi_pin);
            nrfy_gpio_cfg(p_nrfy_config->pins.mosi_pin,
                          NRF_GPIO_PIN_DIR_OUTPUT,
                          NRF_GPIO_PIN_INPUT_DISCONNECT,
                          NRF_GPIO_PIN_NOPULL,
                          pin_drive,
                          NRF_GPIO_PIN_NOSENSE);
        }
        // - MISO (optional) - input
        if (p_nrfy_config->pins.miso_pin != NRF_SPIM_PIN_NOT_CONNECTED)
        {
            nrfy_gpio_cfg(p_nrfy_config->pins.miso_pin,
                          NRF_GPIO_PIN_DIR_INPUT,
                          NRF_GPIO_PIN_INPUT_CONNECT,
                          p_config->miso_pull,
                          pin_drive,
                          NRF_GPIO_PIN_NOSENSE);
        }

        // - Slave Select (optional) - output with initial value 1 (inactive).

        if (ss_pin_to_configure != NRF_SPIM_PIN_NOT_CONNECTED)
        {
            if (ss_active_high)
            {
                nrfy_gpio_pin_clear(ss_pin_to_configure);
            }
            else
            {
                nrfy_gpio_pin_set(ss_pin_to_configure);
            }
            nrfy_gpio_cfg(ss_pin_to_configure,
                          NRF_GPIO_PIN_DIR_OUTPUT,
                          NRF_GPIO_PIN_INPUT_DISCONNECT,
                          NRF_GPIO_PIN_NOPULL,
                          pin_drive,
                          NRF_GPIO_PIN_NOSENSE);
        }

#if NRFX_CHECK(NRFX_SPIM_EXTENDED_ENABLED)
        // - DCX (optional) - output.
        if (p_nrfy_config->ext_config.pins.dcx_pin != NRF_SPIM_PIN_NOT_CONNECTED)
        {
            nrfy_gpio_pin_set(p_nrfy_config->ext_config.pins.dcx_pin);
            nrfy_gpio_cfg(p_nrfy_config->ext_config.pins.dcx_pin,
                          NRF_GPIO_PIN_DIR_OUTPUT,
                          NRF_GPIO_PIN_INPUT_DISCONNECT,
                          NRF_GPIO_PIN_NOPULL,
                          pin_drive,
                          NRF_GPIO_PIN_NOSENSE);
        }
#endif
    }
}


static nrfx_err_t spim_configuration_verify(nrfx_spim_t const *        p_instance,
                                            nrfx_spim_config_t const * p_config)
{
#if NRFX_CHECK(NRFX_SPIM_EXTENDED_ENABLED)
    nrfy_spim_config_t const * p_nrfy_config = &p_config->nrfy_config;
    bool ext_enable = p_nrfy_config->ext_enable;
    nrfy_spim_ext_config_t const * p_ext_config = &p_nrfy_config->ext_config;
    nrfx_err_t err_code;

    // Check if SPIM instance supports the extended features.
    if ((ext_enable) &&
        ((!SPIM_SUPPORTED_FREQ_VALIDATE(p_instance->drv_inst_idx,
                                        p_nrfy_config->frequency)) ||
         ((p_ext_config->pins.csn_pin != NRF_SPIM_PIN_NOT_CONNECTED) &&
          !SPIM_HW_CSN_PRESENT_VALIDATE(p_instance->drv_inst_idx)) ||
         ((p_ext_config->pins.dcx_pin != NRF_SPIM_PIN_NOT_CONNECTED) &&
          !SPIM_DCX_PRESENT_VALIDATE(p_instance->drv_inst_idx))))
    {
        err_code = NRFX_ERROR_NOT_SUPPORTED;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    // Check for an attempt to enable software-controlled
    //  and hardware-controlled Slave Select simultenously.
    if ((ext_enable) &&
        (p_ext_config->pins.csn_pin != NRF_SPIM_PIN_NOT_CONNECTED) &&
        (p_config->sw_ss_pin != NRF_SPIM_PIN_NOT_CONNECTED))
    {
        err_code = NRFX_ERROR_FORBIDDEN;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

#if NRF_SPIM_HAS_32_MHZ_FREQ && defined(NRF5340_XXAA_APPLICATION)
    // Check if dedicated SPIM pins are used, unless both GPIO configuration
    // and pin selection are to be skipped (pin numbers may be not specified
    // in such case).
    if (!(p_config->skip_gpio_cfg && p_nrfy_config->skip_psel_cfg) &&
        (p_instance->p_reg == NRF_SPIM4) && (p_nrfy_config->frequency == NRF_SPIM_FREQ_32M))
    {
        enum {
            SPIM_SCK_DEDICATED  = NRF_GPIO_PIN_MAP(0, 8),
            SPIM_MOSI_DEDICATED = NRF_GPIO_PIN_MAP(0, 9),
            SPIM_MISO_DEDICATED = NRF_GPIO_PIN_MAP(0, 10),
            SPIM_CSN_DEDICATED  = NRF_GPIO_PIN_MAP(0, 11),
            SPIM_DCX_DEDICATED  = NRF_GPIO_PIN_MAP(0, 12),
        };

        if (!SPIM_DEDICATED_PIN_VALIDATE(p_nrfy_config->pins.sck_pin, SPIM_SCK_DEDICATED) ||
            !SPIM_DEDICATED_PIN_VALIDATE(p_nrfy_config->pins.mosi_pin, SPIM_MOSI_DEDICATED) ||
            !SPIM_DEDICATED_PIN_VALIDATE(p_nrfy_config->pins.miso_pin, SPIM_MISO_DEDICATED) ||
            (ext_enable &&
             (!SPIM_DEDICATED_PIN_VALIDATE(p_ext_config->pins.csn_pin, SPIM_CSN_DEDICATED) ||
              !SPIM_DEDICATED_PIN_VALIDATE(p_ext_config->pins.dcx_pin, SPIM_DCX_DEDICATED))))
        {
            err_code = NRFX_ERROR_INVALID_PARAM;
            NRFX_LOG_WARNING("Function: %s, error code: %s.",
                             __func__,
                             NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
        }
    }
#endif // NRF_SPIM_HAS_32_MHZ_FREQ && defined(NRF5340_XXAA_APPLICATION)

#else
    (void)p_instance;
    (void)p_config;
#endif // NRFX_CHECK(NRFX_SPIM_EXTENDED_ENABLED)
    return NRFX_SUCCESS;
}

static void spim_configure(nrfx_spim_t const *        p_instance,
                           nrfx_spim_config_t const * p_config)
{
    spim_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    nrfy_spim_config_t const * p_nrfy_config = &p_config->nrfy_config;

    p_cb->skip_gpio_cfg = p_config->skip_gpio_cfg;
    p_cb->ss_pin = p_config->sw_ss_pin;

    configure_pins(p_instance, p_config);

    nrfy_spim_periph_configure(p_instance->p_reg, p_nrfy_config);
    if (m_cb[p_instance->drv_inst_idx].handler)
    {
        nrfy_spim_int_init(p_instance->p_reg, 0, p_config->irq_priority, false);
    }
}

nrfx_err_t nrfx_spim_init(nrfx_spim_t const *        p_instance,
                          nrfx_spim_config_t const * p_config,
                          nrfx_spim_evt_handler_t    handler,
                          void *                     p_context)
{
    NRFX_ASSERT(p_config);
    spim_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    nrfx_err_t err_code;

    if (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        err_code = NRFX_ERROR_INVALID_STATE;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

#if NRFX_CHECK(NRFX_PRS_ENABLED)
    static nrfx_irq_handler_t const irq_handlers[NRFX_SPIM_ENABLED_COUNT] = {
        NRFX_INSTANCE_IRQ_HANDLERS_LIST(SPIM, spim)
    };
    if (nrfx_prs_acquire(p_instance->p_reg, irq_handlers[p_instance->drv_inst_idx]) != NRFX_SUCCESS)
    {
        err_code = NRFX_ERROR_BUSY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
#endif // NRFX_CHECK(NRFX_PRS_ENABLED)

    p_cb->handler = handler;
    p_cb->p_context = p_context;

    if (p_config)
    {
        p_cb->skip_gpio_cfg = p_config->skip_gpio_cfg;
        p_cb->ss_active_high = p_config->sw_ss_active_high;
        p_cb->ss_pin = p_config->sw_ss_pin;

        err_code = spim_configuration_verify(p_instance, p_config);
        if (err_code != NRFX_SUCCESS)
        {
            return err_code;
        }
        spim_configure(p_instance, p_config);
        nrfy_spim_enable(p_instance->p_reg);
    }

    p_cb->transfer_in_progress = false;
    p_cb->state = NRFX_DRV_STATE_INITIALIZED;

    err_code = NRFX_SUCCESS;
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

nrfx_err_t nrfx_spim_reconfigure(nrfx_spim_t const *        p_instance,
                                 nrfx_spim_config_t const * p_config)
{
    NRFX_ASSERT(p_config);
    spim_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    if (p_cb->state == NRFX_DRV_STATE_UNINITIALIZED)
    {
        return NRFX_ERROR_INVALID_STATE;
    }
    if (p_cb->transfer_in_progress)
    {
        return NRFX_ERROR_BUSY;
    }
    nrfx_err_t err_code = spim_configuration_verify(p_instance, p_config);
    if (err_code != NRFX_SUCCESS)
    {
        return err_code;
    }

    nrfy_spim_disable(p_instance->p_reg);
    spim_configure(p_instance, p_config);
    nrfy_spim_enable(p_instance->p_reg);

    return NRFX_SUCCESS;
}

static void spim_pin_uninit(uint32_t pin)
{
    if (pin == NRF_SPIM_PIN_NOT_CONNECTED)
    {
        return;
    }

    nrfy_gpio_cfg_default(pin);
}

void nrfx_spim_uninit(nrfx_spim_t const * p_instance)
{
    spim_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    NRFX_ASSERT(p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_spim_int_uninit(p_instance->p_reg);
    if (p_cb->handler)
    {
        nrfy_spim_int_disable(p_instance->p_reg, NRF_SPIM_ALL_INTS_MASK);
        if (p_cb->transfer_in_progress)
        {
            // Ensure that SPI is not performing any transfer.
            spim_abort(p_instance->p_reg, p_cb);
        }
    }

    nrfy_spim_disable(p_instance->p_reg);
    nrfy_spim_pins_t pins;
    nrfy_spim_pins_get(p_instance->p_reg, &pins);

    if (!p_cb->skip_gpio_cfg)
    {
        spim_pin_uninit(pins.sck_pin);
        spim_pin_uninit(pins.miso_pin);
        spim_pin_uninit(pins.mosi_pin);
        spim_pin_uninit(p_cb->ss_pin);
#if NRFX_CHECK(NRFX_SPIM_EXTENDED_ENABLED)
        if (SPIM_DCX_PRESENT_VALIDATE(p_instance->drv_inst_idx) &&
            SPIM_HW_CSN_PRESENT_VALIDATE(p_instance->drv_inst_idx))
        {
            nrfy_spim_ext_pins_t ext_pins;
            nrfy_spim_ext_pins_get(p_instance->p_reg, &ext_pins);
            spim_pin_uninit(ext_pins.dcx_pin);
            spim_pin_uninit(ext_pins.csn_pin);
        }
#endif
    }

#if NRFX_CHECK(USE_WORKAROUND_FOR_ANOMALY_195)
    if (p_instance->p_reg == NRF_SPIM3)
    {
        *(volatile uint32_t *)0x4002F004 = 1;
    }
#endif

#if NRFX_CHECK(NRFX_PRS_ENABLED)
    nrfx_prs_release(p_instance->p_reg);
#endif

    p_cb->state = NRFX_DRV_STATE_UNINITIALIZED;
}

#if NRFX_CHECK(NRFX_SPIM_EXTENDED_ENABLED)
nrfx_err_t nrfx_spim_xfer_dcx(nrfx_spim_t const *           p_instance,
                              nrfx_spim_xfer_desc_t const * p_xfer_desc,
                              uint32_t                      flags,
                              uint8_t                       cmd_length)
{
    (void)flags;

    NRFX_ASSERT(cmd_length <= NRF_SPIM_DCX_CNT_ALL_CMD);
    nrfy_spim_dcx_cnt_set((NRF_SPIM_Type *)p_instance->p_reg, cmd_length);
    return nrfx_spim_xfer(p_instance, p_xfer_desc, 0);
}
#endif

static void set_ss_pin_state(spim_control_block_t * p_cb, bool active)
{
    if (p_cb->ss_pin != NRF_SPIM_PIN_NOT_CONNECTED)
    {
        if (p_cb->ss_active_high)
        {
            if (active)
            {
                nrfy_gpio_pin_set(p_cb->ss_pin);
            }
            else
            {
                nrfy_gpio_pin_clear(p_cb->ss_pin);
            }
        }
        else
        {
            if (active)
            {
                nrfy_gpio_pin_clear(p_cb->ss_pin);
            }
            else
            {
                nrfy_gpio_pin_set(p_cb->ss_pin);
            }
        }
    }
}

static void finish_transfer(spim_control_block_t * p_cb)
{
    // If Slave Select signal is used, this is the time to deactivate it.
    set_ss_pin_state(p_cb, false);

    // By clearing this flag before calling the handler we allow subsequent
    // transfers to be started directly from the handler function.
    p_cb->transfer_in_progress = false;

    p_cb->evt.type = NRFX_SPIM_EVENT_DONE;
    p_cb->handler(&p_cb->evt, p_cb->p_context);
}

static nrfx_err_t spim_xfer(NRF_SPIM_Type               * p_spim,
                            spim_control_block_t        * p_cb,
                            nrfx_spim_xfer_desc_t const * p_xfer_desc,
                            uint32_t                      flags)
{
    nrfx_err_t err_code;
    // EasyDMA requires that transfer buffers are placed in Data RAM region;
    // signal error if they are not.
    if ((p_xfer_desc->p_tx_buffer != NULL && !nrfx_is_in_ram(p_xfer_desc->p_tx_buffer)) ||
        (p_xfer_desc->p_rx_buffer != NULL && !nrfx_is_in_ram(p_xfer_desc->p_rx_buffer)))
    {
        p_cb->transfer_in_progress = false;
        err_code = NRFX_ERROR_INVALID_ADDR;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

#if NRFX_CHECK(NRFX_SPIM3_NRF52840_ANOMALY_198_WORKAROUND_ENABLED)
    if (p_spim == NRF_SPIM3)
    {
        anomaly_198_enable(p_xfer_desc->p_tx_buffer, p_xfer_desc->tx_length);
    }
#endif

#if NRFY_SPIM_HAS_ARRAY_LIST
    nrfy_spim_tx_list_set(p_spim, NRFX_SPIM_FLAG_TX_POSTINC & flags);
    nrfy_spim_rx_list_set(p_spim, NRFX_SPIM_FLAG_RX_POSTINC & flags);
#endif

    nrfy_spim_xfer_desc_t xfer_desc = *p_xfer_desc;
#if NRFX_CHECK(NRFX_SPIM_NRF52_ANOMALY_109_WORKAROUND_ENABLED)
    if (flags & NRFX_SPIM_FLAG_HOLD_XFER)
    {
        xfer_desc.tx_length = 0;
        xfer_desc.rx_length = 0;
        nrfy_spim_int_enable(p_spim, NRF_SPIM_INT_STARTED_MASK);
    }
#endif
    nrfy_spim_buffers_set(p_spim, &xfer_desc);

    nrfy_spim_event_clear(p_spim, NRF_SPIM_EVENT_END);

    if (!(flags & NRFX_SPIM_FLAG_HOLD_XFER))
    {
        nrfy_spim_xfer_start(p_spim, p_cb->handler ? NULL : &xfer_desc);
    }

    if (!p_cb->handler)
    {
#if NRFX_CHECK(NRFX_SPIM3_NRF52840_ANOMALY_198_WORKAROUND_ENABLED)
        if (p_spim == NRF_SPIM3)
        {
            anomaly_198_disable();
        }
#endif
        set_ss_pin_state(p_cb, false);
    }
    else
    {
        if (flags & NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER)
        {
            nrfy_spim_int_disable(p_spim, NRF_SPIM_INT_END_MASK);
        }
        else
        {
            nrfy_spim_int_enable(p_spim, NRF_SPIM_INT_END_MASK);
        }
    }
    err_code = NRFX_SUCCESS;
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

nrfx_err_t nrfx_spim_xfer(nrfx_spim_t const *           p_instance,
                          nrfx_spim_xfer_desc_t const * p_xfer_desc,
                          uint32_t                      flags)
{
    spim_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    NRFX_ASSERT(p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);
    NRFX_ASSERT(p_xfer_desc->p_tx_buffer != NULL || p_xfer_desc->tx_length == 0);
    NRFX_ASSERT(p_xfer_desc->p_rx_buffer != NULL || p_xfer_desc->rx_length == 0);
    NRFX_ASSERT(SPIM_LENGTH_VALIDATE(p_instance->drv_inst_idx,
                                     p_xfer_desc->rx_length,
                                     p_xfer_desc->tx_length));
    NRFX_ASSERT(!(flags & NRFX_SPIM_FLAG_HOLD_XFER) ||
                (p_cb->ss_pin == NRF_SPIM_PIN_NOT_CONNECTED));

    nrfx_err_t err_code = NRFX_SUCCESS;

#if !NRFY_SPIM_HAS_ARRAY_LIST
    if ((NRFX_SPIM_FLAG_TX_POSTINC | NRFX_SPIM_FLAG_RX_POSTINC) & flags)
    {
        err_code = NRFX_ERROR_NOT_SUPPORTED;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
#endif

    if (p_cb->transfer_in_progress)
    {
        err_code = NRFX_ERROR_BUSY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
    else
    {
        if (p_cb->handler && !(flags & (NRFX_SPIM_FLAG_REPEATED_XFER |
                                        NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER)))
        {
            p_cb->transfer_in_progress = true;
        }
    }

    p_cb->evt.xfer_desc = *p_xfer_desc;

    set_ss_pin_state(p_cb, true);

    return spim_xfer(p_instance->p_reg, p_cb,  p_xfer_desc, flags);
}

void nrfx_spim_abort(nrfx_spim_t const * p_instance)
{
    spim_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    NRFX_ASSERT(p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);

    spim_abort(p_instance->p_reg, p_cb);
}

static void irq_handler(NRF_SPIM_Type * p_spim, spim_control_block_t * p_cb)
{
    uint32_t evt_mask = nrfy_spim_events_process(p_spim,
#if NRFX_CHECK(NRFX_SPIM_NRF52_ANOMALY_109_WORKAROUND_ENABLED)
                                                 NRFY_EVENT_TO_INT_BITMASK(NRF_SPIM_EVENT_STARTED) |
#endif
                                                 NRFY_EVENT_TO_INT_BITMASK(NRF_SPIM_EVENT_END),
                                                 &p_cb->evt.xfer_desc);

#if NRFX_CHECK(NRFX_SPIM_NRF52_ANOMALY_109_WORKAROUND_ENABLED)
    if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_SPIM_EVENT_STARTED))
    {
        /* Handle first, zero-length, auxiliary transmission. */
        NRFX_ASSERT(nrfy_spim_tx_maxcnt_get(p_spim) == 0);
        NRFX_ASSERT(nrfy_spim_rx_maxcnt_get(p_spim) == 0);

        /* Disable STARTED interrupt, used only in auxiliary transmission. */
        nrfy_spim_int_disable(p_spim, NRF_SPIM_INT_STARTED_MASK);

        /* Start the actual, glitch-free transmission. */
        nrfy_spim_buffers_set(p_spim, &p_cb->evt.xfer_desc);
        nrfy_spim_xfer_start(p_spim, NULL);
        return;
    }
#endif

    if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_SPIM_EVENT_END))
    {
#if NRFX_CHECK(NRFX_SPIM3_NRF52840_ANOMALY_198_WORKAROUND_ENABLED)
        if (p_spim == NRF_SPIM3)
        {
            anomaly_198_disable();
        }
#endif
        NRFX_ASSERT(p_cb->handler);
        NRFX_LOG_DEBUG("Event: NRF_SPIM_EVENT_END.");
        finish_transfer(p_cb);
    }
}

NRFX_INSTANCE_IRQ_HANDLERS(SPIM, spim)

#endif // NRFX_CHECK(NRFX_SPIM_ENABLED)
