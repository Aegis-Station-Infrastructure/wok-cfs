/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Storage Subsystem (WOK_SEIS data products)
** File: wok_common/hal/inc/wok_hal_storage.h
*/

#ifndef WOK_HAL_STORAGE_H
#define WOK_HAL_STORAGE_H

#include "wok_hal.h"

int32_t WOK_HAL_Storage_Init(void);
int32_t WOK_HAL_Storage_WriteFile(const char *name, const void *data, uint32_t size);

/**
 * @brief Read a stored data product into buf (up to max_size bytes).
 *        *size receives the number of bytes read. Used by the
 *        file-downlink path.
 */
int32_t WOK_HAL_Storage_ReadFile(const char *name, void *buf,
                                 uint32_t max_size, uint32_t *size);
int32_t WOK_HAL_Storage_GetFreeSpace(uint64_t *bytes);

#endif /* WOK_HAL_STORAGE_H */
