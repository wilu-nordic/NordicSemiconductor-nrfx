/*
 * Copyright (c) 2019 - 2022, Nordic Semiconductor ASA
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

#ifndef NRF_CACHE_H__
#define NRF_CACHE_H__

#include <nrfx.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrf_cache_hal CACHE HAL
 * @{
 * @ingroup nrf_cache
 * @brief   The hardware access layer for managing the CACHE peripheral.
 */

#if defined(CACHEDATA_SET_WAY_DATA0_Data_Msk) || defined(__NRFX_DOXYGEN__)
/** @brief Presence of the CACHEDATA feature. */
#define NRF_CACHE_HAS_CACHEDATA 1
#else
#define NRF_CACHE_HAS_CACHEDATA 0
#endif

#if defined(CACHEINFO_SET_WAY_TAG_Msk) || defined(__NRFX_DOXYGEN__)
/** @brief Presence of the CACHEINFO feature. */
#define NRF_CACHE_HAS_CACHEINFO 1
#else
#define NRF_CACHE_HAS_CACHEINFO 0
#endif

#if defined(CACHE_TASKS_INVALIDATECACHE_TASKS_INVALIDATECACHE_Msk) || defined(__NRFX_DOXYGEN__)
/** @brief Symbol indicating whether cache/line tasks are supported */
#define NRF_CACHE_HAS_TASKS 1
#else
#define NRF_CACHE_HAS_TASKS 0
#endif

#if defined(CACHE_TASKS_CLEANCACHE_TASKS_CLEANCACHE_Msk) || defined(__NRFX_DOXYGEN__)
/** @brief Symbol indicating whether the CLEAN cache/line tasks are supported. */
#define NRF_CACHE_HAS_TASK_CLEAN 1
#else
#define NRF_CACHE_HAS_TASK_CLEAN 0
#endif

#if defined(CACHE_TASKS_FLUSHCACHE_TASKS_FLUSHCACHE_Msk) || defined(__NRFX_DOXYGEN__)
/** @brief Symbol indicating whether the FLUSH cache/line tasks are supported. */
#define NRF_CACHE_HAS_TASK_FLUSH 1
#else
#define NRF_CACHE_HAS_TASK_FLUSH 0
#endif

#if defined(CACHE_STATUS_READY_Msk) || defined(__NRFX_DOXYGEN__)
/** @brief Symbol indicating whether status/busy check is supported */
#define NRF_CACHE_HAS_STATUS 1
#else
#define NRF_CACHE_HAS_STATUS 0
#endif

#if NRF_CACHE_HAS_TASKS
/** @brief CACHE tasks. */
typedef enum
{
#if NRF_CACHE_HAS_TASK_CLEAN
    NRF_CACHE_TASK_CLEANCACHE      = offsetof(NRF_CACHE_Type, TASKS_CLEANCACHE),      /**< Clean the whole cache. */
    NRF_CACHE_TASK_CLEANLINE       = offsetof(NRF_CACHE_Type, TASKS_CLEANLINE),       /**< Clean the cache line. */
#endif
#if NRF_CACHE_HAS_TASK_FLUSH
    NRF_CACHE_TASK_FLUSHCACHE      = offsetof(NRF_CACHE_Type, TASKS_FLUSHCACHE),      /**< Flush the whole cache. */
    NRF_CACHE_TASK_FLUSHLINE       = offsetof(NRF_CACHE_Type, TASKS_FLUSHLINE),       /**< Flush the cache line. */
#endif
    NRF_CACHE_TAST_SAVE            = offsetof(NRF_CACHE_Type, TASKS_SAVE),            /**< Save the state to a retained memory space. */
    NRF_CACHE_TAST_RESTORE         = offsetof(NRF_CACHE_Type, TASKS_RESTORE),         /**< Restore the state from a retained memory space. */
    NRF_CACHE_TASK_INVALIDATECACHE = offsetof(NRF_CACHE_Type, TASKS_INVALIDATECACHE), /**< Invalidate the whole cache. */
    NRF_CACHE_TASK_INVALIDATELINE  = offsetof(NRF_CACHE_Type, TASKS_INVALIDATELINE),  /**< Invalidate the cache line. */
    NRF_CACHE_TASK_ERASE           = offsetof(NRF_CACHE_Type, TASKS_ERASE),           /**< Erase the whole cache. */
} nrf_cache_task_t;
#endif

/** @brief Cache regions. */
typedef enum
{
    NRF_CACHE_REGION_FLASH = 0, ///< Cache region related to Flash access.
    NRF_CACHE_REGION_XIP   = 1, ///< Cache region related to XIP access.
} nrf_cache_region_t;

/**
 * @brief Function for enabling the CACHE peripheral.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 */
NRF_STATIC_INLINE void nrf_cache_enable(NRF_CACHE_Type * p_reg);

/**
 * @brief Function for disabling the CACHE peripheral.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 */
NRF_STATIC_INLINE void nrf_cache_disable(NRF_CACHE_Type * p_reg);

/**
 * @brief Function for invalidating the cache content.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 */
NRF_STATIC_INLINE void nrf_cache_invalidate(NRF_CACHE_Type * p_reg);

/**
 * @brief Function for erasing the cache content.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 */
NRF_STATIC_INLINE void nrf_cache_erase(NRF_CACHE_Type * p_reg);

/**
 * @brief Function for checking the status of @ref nrf_cache_erase().
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 *
 * @retval true  Erase is finished.
 * @retval false Erase is not complete or has not started.
 */
NRF_STATIC_INLINE bool nrf_cache_erase_status_check(NRF_CACHE_Type const * p_reg);

/**
 * @brief Function for clearing the status of the cache erase.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 */
NRF_STATIC_INLINE void nrf_cache_erase_status_clear(NRF_CACHE_Type * p_reg);

/**
 * @brief Function for setting the cache profiling.
 *
 * @param[in] p_reg  Pointer to the structure of registers of the peripheral.
 * @param[in] enable True if cache profiling is to be enabled, false otherwise.
 */
NRF_STATIC_INLINE void nrf_cache_profiling_set(NRF_CACHE_Type * p_reg, bool enable);

/**
 * @brief Function for clearing the cache profiling counters.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 */
NRF_STATIC_INLINE void nrf_cache_profiling_counters_clear(NRF_CACHE_Type * p_reg);

/**
 * @brief Function for getting the number of cache hits for instruction fetch from the specified
 *        cache region.
 *
 * @note Separate counters are used for flash region and XIP region.
 * @note Cache profiling must be enabled first. See @ref nrf_cache_profiling_set.
 *
 * @param[in] p_reg  Pointer to the structure of registers of the peripheral.
 * @param[in] region Cache region.
 *
 * @return Number of instruction fetch cache hits.
 */
NRF_STATIC_INLINE uint32_t nrf_cache_instruction_hit_counter_get(NRF_CACHE_Type const * p_reg,
                                                                 nrf_cache_region_t     region);

/**
 * @brief Function for getting the number of cache misses for instruction fetch from the specified
 *        cache region.
 *
 * @note Separate counters are used for flash region and XIP region.
 * @note Cache profiling must be enabled first. See @ref nrf_cache_profiling_set.
 *
 * @param[in] p_reg  Pointer to the structure of registers of the peripheral.
 * @param[in] region Cache region.
 *
 * @return Number of instruction fetch cache misses.
 */
NRF_STATIC_INLINE uint32_t nrf_cache_instruction_miss_counter_get(NRF_CACHE_Type const * p_reg,
                                                                  nrf_cache_region_t     region);

/**
 * @brief Function for getting the number of cache hits for data fetch from the specified
 *        cache region.
 *
 * @note Separate counters are used for flash region and XIP region.
 * @note Cache profiling must be enabled first. See @ref nrf_cache_profiling_set.
 *
 * @param[in] p_reg  Pointer to the structure of registers of the peripheral.
 * @param[in] region Cache region.
 *
 * @return Number of data fetch cache hits.
 */
NRF_STATIC_INLINE uint32_t nrf_cache_data_hit_counter_get(NRF_CACHE_Type const * p_reg,
                                                          nrf_cache_region_t     region);

/**
 * @brief Function for getting the number of cache misses for data fetch from the specified
 *        cache region.
 *
 * @note Separate counters are used for flash region and XIP region.
 * @note Cache profiling must be enabled first. See @ref nrf_cache_profiling_set.
 *
 * @param[in] p_reg  Pointer to the structure of registers of the peripheral.
 * @param[in] region Cache region.
 *
 * @return Number of data fetch cache misses.
 */
NRF_STATIC_INLINE uint32_t nrf_cache_data_miss_counter_get(NRF_CACHE_Type const * p_reg,
                                                           nrf_cache_region_t     region);

/**
 * @brief Function for setting the cache RAM mode.
 *
 * When configured in the RAM mode, the accesses to internal or external flash will not be cached.
 * In this mode, the cache data contents can be used as the read/write RAM.
 * Only the data content of the cache is available as RAM.
 *
 * @note -Enabling the RAM mode causes the RAM to be cleared.
 * @note -Disabling the RAM mode causes the cache to be invalidated.
 *
 * @param[in] p_reg  Pointer to the structure of registers of the peripheral.
 * @param[in] enable True if the cache RAM mode is to be enabled, false otherwise.
 */
NRF_STATIC_INLINE void nrf_cache_ram_mode_set(NRF_CACHE_Type * p_reg, bool enable);

/**
 * @brief Function for blocking the cache content access.
 *
 * To unlock the cache content access, a reset has to be performed.
 *
 * @note Blocking is ignored in the RAM mode.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 */
NRF_STATIC_INLINE void nrf_cache_read_lock_enable(NRF_CACHE_Type * p_reg);

/**
 * @brief Function for blocking the cache content updates.
 *
 * Blocking of updates prevents updating of cache content on cache misses,
 * but the peripheral will continue to check for instruction/data fetches
 * in the content already present in the cache.
 *
 * @note Blocking is ignored in the RAM mode.
 *
 * @param[in] p_reg  Pointer to the structure of registers of the peripheral.
 * @param[in] enable True if cache content update lock is to be enabled, false otherwise.
 */
NRF_STATIC_INLINE void nrf_cache_update_lock_set(NRF_CACHE_Type * p_reg, bool enable);

#if NRF_CACHE_HAS_CACHEDATA
/**
 * @brief Function for getting the cache data word.
 *
 * @note When operating in the RAM mode, the cache data is accessible as a general purpose RAM.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] set   Set that contains the data to get.
 * @param[in] way   Way that contains the data to get.
 * @param[in] word  Data word index to get.
 *
 * @return 32-bit data word.
 */
NRF_STATIC_INLINE uint32_t nrf_cache_data_get(NRF_CACHEDATA_Type const * p_reg,
                                              uint32_t                   set,
                                              uint8_t                    way,
                                              uint8_t                    word);
#endif

#if NRF_CACHE_HAS_CACHEINFO
/**
 * @brief Function for getting the tag associated with the specified set and way.
 *
 * The tag is used to check if an entry in the cache matches the address that is being fetched.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] set   Set that contains the tag to get.
 * @param[in] way   Way that contains the tag to get.
 *
 * @return Tag value.
 */
NRF_STATIC_INLINE uint32_t nrf_cache_tag_get(NRF_CACHEINFO_Type const * p_reg,
                                             uint32_t                   set,
                                             uint8_t                    way);

/**
 * @brief Function for checking the validity of a cache line associated with the specified set and way.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] set   Set that contains the cache line to check.
 * @param[in] way   Way that contains the cache line to check.
 *
 * @retval true  Cache line is valid.
 * @retval false Cache line is invalid.
 */
NRF_STATIC_INLINE bool nrf_cache_line_validity_check(NRF_CACHEINFO_Type const * p_reg,
                                                     uint32_t                   set,
                                                     uint8_t                    way);

/**
 * @brief Function for getting the most recently used way in the specified set.
 *
 * The most recently used way is updated on each fetch from the cache and is used for the cache replacement policy.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] set   Specified set.
 *
 * @return The most recently used way in the specified set.
 */
NRF_STATIC_INLINE uint8_t nrf_cache_mru_get(NRF_CACHEINFO_Type const * p_reg, uint32_t set);
#endif

#if NRF_CACHE_HAS_TASKS
/**
 * @brief Function to set the memory address covered by the line to be maintained.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] addr  Cache line adress.
 */
NRF_STATIC_INLINE void nrf_cache_lineaddr_set(NRF_CACHE_Type * p_reg, uint32_t addr);

/**
 * @brief Function for triggering the specified CACHE task.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] task  Task.
 */
NRF_STATIC_INLINE void nrf_cache_task_trigger(NRF_CACHE_Type * p_reg, nrf_cache_task_t task);

/**
 * @brief Function for returning the address of the specified task register.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] task  Task.
 *
 * @return Task address.
 */
NRF_STATIC_INLINE uint32_t nrf_cache_task_address_get(NRF_CACHE_Type const * p_reg,
                                                      nrf_cache_task_t       task);
#endif

#if NRF_CACHE_HAS_STATUS
/**
 * @brief Function for checking if the cache is busy or not.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 *
 * @return True if the cache is busy, false otherwise.
 */
NRF_STATIC_INLINE bool nrf_cache_busy_check(NRF_CACHE_Type const * p_reg);
#endif

#ifndef NRF_DECLARE_ONLY

NRF_STATIC_INLINE void nrf_cache_enable(NRF_CACHE_Type * p_reg)
{
    p_reg->ENABLE = CACHE_ENABLE_ENABLE_Enabled;
}

NRF_STATIC_INLINE void nrf_cache_disable(NRF_CACHE_Type * p_reg)
{
    p_reg->ENABLE = CACHE_ENABLE_ENABLE_Disabled;
}

NRF_STATIC_INLINE void nrf_cache_invalidate(NRF_CACHE_Type * p_reg)
{
#if NRF_CACHE_HAS_TASKS
    nrf_cache_task_trigger(p_reg, NRF_CACHE_TASK_INVALIDATECACHE);
#else
    p_reg->INVALIDATE = CACHE_INVALIDATE_INVALIDATE_Invalidate;
#endif
}

NRF_STATIC_INLINE void nrf_cache_erase(NRF_CACHE_Type * p_reg)
{
#if NRF_CACHE_HAS_TASKS
    nrf_cache_task_trigger(p_reg, NRF_CACHE_TASK_ERASE);
#else
    p_reg->ERASE = CACHE_ERASE_ERASE_Erase;
#endif
}

NRF_STATIC_INLINE bool nrf_cache_erase_status_check(NRF_CACHE_Type const * p_reg)
{
#if NRF_CACHE_HAS_STATUS
    return (!nrf_cache_busy_check(p_reg));
#else
    return (bool)(p_reg->ERASESTATUS & CACHE_ERASESTATUS_ERASESTATUS_Msk);
#endif
}

NRF_STATIC_INLINE void nrf_cache_erase_status_clear(NRF_CACHE_Type * p_reg)
{
#if NRF_CACHE_HAS_TASKS
    /* No task for erasing the status */
    (void)p_reg;
#else
    p_reg->ERASESTATUS = 0;
#endif
}

NRF_STATIC_INLINE void nrf_cache_profiling_set(NRF_CACHE_Type * p_reg, bool enable)
{
#if defined(CACHE_PROFILINGENABLE_ENABLE_Msk)
    p_reg->PROFILINGENABLE =
        (enable ? CACHE_PROFILINGENABLE_ENABLE_Enable : CACHE_PROFILINGENABLE_ENABLE_Disable);
#elif defined(CACHE_PROFILING_ENABLE_ENABLE_Msk)
    p_reg->PROFILING.ENABLE =
        (enable ? CACHE_PROFILING_ENABLE_ENABLE_Enable : CACHE_PROFILING_ENABLE_ENABLE_Disable);
#endif
}

NRF_STATIC_INLINE void nrf_cache_profiling_counters_clear(NRF_CACHE_Type * p_reg)
{
#if defined(CACHE_PROFILINGCLEAR_CLEAR_Msk)
    p_reg->PROFILINGCLEAR = (CACHE_PROFILINGCLEAR_CLEAR_Clear << CACHE_PROFILINGCLEAR_CLEAR_Pos);
#elif defined(CACHE_PROFILING_CLEAR_CLEAR_Msk)
    p_reg->PROFILING.CLEAR = (CACHE_PROFILING_CLEAR_CLEAR_Clear << CACHE_PROFILING_CLEAR_CLEAR_Pos);
#endif
}

NRF_STATIC_INLINE uint32_t nrf_cache_instruction_hit_counter_get(NRF_CACHE_Type const * p_reg,
                                                                 nrf_cache_region_t     region)
{
#if defined(CACHE_PROFILING_IHIT_HITS_Msk)
    return p_reg->PROFILING[region].IHIT;
#elif defined(CACHE_PROFILING_HIT_HITS_Msk)
    (void)region;
    return p_reg->PROFILING.HIT;
#endif
}

NRF_STATIC_INLINE uint32_t nrf_cache_instruction_miss_counter_get(NRF_CACHE_Type const * p_reg,
                                                                  nrf_cache_region_t     region)
{
#if defined(CACHE_PROFILING_IMISS_MISSES_Msk)
    return p_reg->PROFILING[region].IMISS;
#elif defined(CACHE_PROFILING_MISS_MISSES_Msk)
    (void)region;
    return p_reg->PROFILING.MISS;
#endif
}

NRF_STATIC_INLINE uint32_t nrf_cache_data_hit_counter_get(NRF_CACHE_Type const * p_reg,
                                                          nrf_cache_region_t     region)
{
#if defined(CACHE_PROFILING_DHIT_HITS_Msk)
    return p_reg->PROFILING[region].DHIT;
#elif defined(CACHE_PROFILING_HIT_HITS_Msk)
    (void)region;
    return p_reg->PROFILING.HIT;
#endif
}

NRF_STATIC_INLINE uint32_t nrf_cache_data_miss_counter_get(NRF_CACHE_Type const * p_reg,
                                                           nrf_cache_region_t     region)
{
#if defined(CACHE_PROFILING_DMISS_MISSES_Msk)
    return p_reg->PROFILING[region].DMISS;
#elif defined(CACHE_PROFILING_MISS_MISSES_Msk)
    (void)region;
    return p_reg->PROFILING.MISS;
#endif
}

NRF_STATIC_INLINE void nrf_cache_ram_mode_set(NRF_CACHE_Type * p_reg, bool enable)
{
    p_reg->MODE = (enable ? CACHE_MODE_MODE_Ram : CACHE_MODE_MODE_Cache);
}

NRF_STATIC_INLINE void nrf_cache_read_lock_enable(NRF_CACHE_Type * p_reg)
{
    p_reg->DEBUGLOCK = CACHE_DEBUGLOCK_DEBUGLOCK_Locked;
}

NRF_STATIC_INLINE void nrf_cache_update_lock_set(NRF_CACHE_Type * p_reg, bool enable)
{
    p_reg->WRITELOCK =
        (enable ? CACHE_WRITELOCK_WRITELOCK_Locked : CACHE_WRITELOCK_WRITELOCK_Unlocked);
}

#if NRF_CACHE_HAS_CACHEDATA
NRF_STATIC_INLINE uint32_t nrf_cache_data_get(NRF_CACHEDATA_Type const * p_reg,
                                              uint32_t                   set,
                                              uint8_t                    way,
                                              uint8_t                    word)
{
    volatile CACHEDATA_SET_WAY_Type const * reg = &p_reg->SET[set].WAY[way];
    switch (word)
    {
        case 0: return reg->DATA0;
        case 1: return reg->DATA1;
        case 2: return reg->DATA2;
        case 3: return reg->DATA3;
        default:
            NRFX_ASSERT(false);
            return 0;
    }
}
#endif

#if NRF_CACHE_HAS_CACHEINFO
NRF_STATIC_INLINE uint32_t nrf_cache_tag_get(NRF_CACHEINFO_Type const * p_reg,
                                             uint32_t                   set,
                                             uint8_t                    way)
{
    return (p_reg->SET[set].WAY[way] & CACHEINFO_SET_WAY_TAG_Msk);
}

NRF_STATIC_INLINE bool nrf_cache_line_validity_check(NRF_CACHEINFO_Type const * p_reg,
                                                     uint32_t                   set,
                                                     uint8_t                    way)
{
    return (bool)(p_reg->SET[set].WAY[way] & CACHEINFO_SET_WAY_V_Msk);
}

NRF_STATIC_INLINE uint8_t nrf_cache_mru_get(NRF_CACHEINFO_Type const * p_reg, uint32_t set)
{
    return ((p_reg->SET[set].WAY[0] & CACHEINFO_SET_WAY_MRU_Msk) >> CACHEINFO_SET_WAY_MRU_Pos);
}
#endif

#if NRF_CACHE_HAS_TASKS
NRF_STATIC_INLINE void nrf_cache_lineaddr_set(NRF_CACHE_Type * p_reg, uint32_t addr)
{
    p_reg->LINEADDR = addr;
}

NRF_STATIC_INLINE void nrf_cache_task_trigger(NRF_CACHE_Type * p_reg, nrf_cache_task_t task)
{
    *((volatile uint32_t *)((uint8_t *)p_reg + (uint32_t)task)) = 0x1UL;
}

NRF_STATIC_INLINE uint32_t nrf_cache_task_address_get(NRF_CACHE_Type const * p_reg,
                                                      nrf_cache_task_t       task)
{
    return (uint32_t)p_reg + (uint32_t)task;
}
#endif

#if NRF_CACHE_HAS_STATUS
NRF_STATIC_INLINE bool nrf_cache_busy_check(NRF_CACHE_Type const * p_reg)
{
    return (bool)(p_reg->STATUS & CACHE_STATUS_READY_Msk);
}
#endif

#endif // NRF_DECLARE_ONLY

/** @} */

#ifdef __cplusplus
}
#endif

#endif // NRF_CACHE_H__
