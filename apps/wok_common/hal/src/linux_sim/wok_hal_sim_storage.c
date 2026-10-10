/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Linux Sim Storage
** File: wok_common/hal/src/linux_sim/wok_hal_sim_storage.c
*/

#include "wok_hal_storage.h"
#include <stdio.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <string.h>

#define SIM_DATA_DIR "/tmp/wok_data"

int32_t WOK_HAL_Storage_Init(void)
{
    mkdir(SIM_DATA_DIR, 0755);
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Storage_WriteFile(const char *name, const void *data, uint32_t size)
{
    if (!name || !data) return WOK_HAL_ERROR;
    char path[256];
    snprintf(path, sizeof(path), "%s/%s", SIM_DATA_DIR, name);
    FILE *f = fopen(path, "wb");
    if (!f) return WOK_HAL_ERROR;
    size_t written = fwrite(data, 1, size, f);
    fclose(f);
    return (written == size) ? WOK_HAL_OK : WOK_HAL_ERROR;
}

int32_t WOK_HAL_Storage_ReadFile(const char *name, void *buf,
                                 uint32_t max_size, uint32_t *size)
{
    if (!name || !buf || !size) return WOK_HAL_ERROR;
    /* Reject path escapes — products are flat files in the data dir */
    if (strchr(name, '/') != NULL) return WOK_HAL_ERROR;
    char path[256];
    snprintf(path, sizeof(path), "%s/%s", SIM_DATA_DIR, name);
    FILE *f = fopen(path, "rb");
    if (!f) return WOK_HAL_ERROR;
    *size = (uint32_t)fread(buf, 1, max_size, f);
    fclose(f);
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Storage_GetFreeSpace(uint64_t *bytes)
{
    if (!bytes) return WOK_HAL_ERROR;
    *bytes = (uint64_t)1 * 1024 * 1024 * 1024; /* Report 1 GB */
    return WOK_HAL_OK;
}
