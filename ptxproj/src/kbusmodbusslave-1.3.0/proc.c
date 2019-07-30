//------------------------------------------------------------------------------
/// Copyright (c) WAGO Kontakttechnik GmbH & Co. KG
///
/// PROPRIETARY RIGHTS are involved in the subject matter of this material.
/// All manufacturing, reproduction, use and sales rights pertaining to this
/// subject matter are governed by the license agreement. The recipient of this
/// software implicitly accepts the terms of the license.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
///
///  \file     proc.c
///
///  \brief    Creating proc entry for kbus information
///
///  \author   <BrT> : WAGO Kontakttechnik GmbH & Co. KG
//------------------------------------------------------------------------------

//Unless we are not a kernel module we will not write any information to /proc!!!

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include "utils.h"
#include "proc.h"

#define FILE_PATH   "/tmp/KBUS/"                            /**< @brief Filepath for information filed */
#define FILE_NAME_TERMINAL_COUNT    FILE_PATH"termCount"    /**< @brief Filename for I/O Module count*/
#define FILE_NAME_TERMINAL_ASSEMBLY FILE_PATH"termInfo"     /**< @brief Filename for I/O Module description*/
#define MAX_BUFFER_SIZE 1*1024 /**< @brief Buffer for files 1kB*/

/**
 * @brief Creating two files:
 * 1. I/O Module count
 * 2. I/O Module assembly
 * which represents the I/O Modules connected to the kbus.
 *
 * @param[in] terminalCnt Number of I/O-Modules
 * @param[in] *modules Pointer to I/O-Modules-type table
 *
 * @retval 0 on success
 * @retval <0 on failure
 */
int proc_createEntry(size_t terminalCnt, module_desc_t *modules, tldkc_KbusInfo_TerminalInfo *termDescription)
{
    int error = 0;
    int fd_count = 0;
    int fd_info = 0;

    //Create directory if not exists
    if (mkdir(FILE_PATH, 0755) < 0)
    {
        dprintf(VERBOSE_STD, "MKDIR %s failed: %s\n", FILE_PATH, strerror(errno));
        error = -1;
        goto exit;
    }

    fd_count = open (FILE_NAME_TERMINAL_COUNT, O_RDWR|O_CREAT, S_IRUSR|S_IRGRP|S_IROTH);
    if (fd_count < 0)
    {
        dprintf(VERBOSE_STD, "File Create: %s failed\n", FILE_NAME_TERMINAL_COUNT);
        error = -1;
        goto exit;
    }

    fd_info = open (FILE_NAME_TERMINAL_ASSEMBLY, O_RDWR|O_CREAT, S_IRUSR|S_IRGRP|S_IROTH);
    if (fd_info < 0)
    {
        dprintf(VERBOSE_STD, "File Create: %s failed\n", FILE_NAME_TERMINAL_ASSEMBLY);
        error = -2;
        goto exit;
    }

    char buffer[MAX_BUFFER_SIZE];
    int bytesToWrite = 0;
    int written = 0;

    bytesToWrite = snprintf(buffer, sizeof(buffer), "%zd", terminalCnt);
    while (((written = write(fd_count, buffer, bytesToWrite)) < 0) && errno == EINTR);

    if (written != bytesToWrite)
    {
        dprintf(VERBOSE_STD, "File write %s failed: %s\n", FILE_NAME_TERMINAL_COUNT, strerror(errno));
        error = -3;
        goto exit;
    }

    size_t i = 0;
    int k = 0;
    int bufferSize = sizeof(buffer);
    char *ptr = buffer; //get start address of buffer
    bytesToWrite = 0; //reset value

    for (i = 0; i < terminalCnt; i++)
    {
        bufferSize = sizeof(buffer); //reset orignal bufferSize
        ptr = buffer; //reset ptr to buffer start
        bytesToWrite = 0; // reset bytes to write

        k = snprintf(ptr, bufferSize, "Pos:%d \tType:", i);
        ptr += k;
        bufferSize -= k;
        bytesToWrite += k;

        k = snprintf(ptr, bufferSize, "%s\t", modules[i].desc_str);
        ptr += k;
        bufferSize -= k;
        bytesToWrite += k;

        k = snprintf(ptr, bufferSize, "BitOffsetOut:%d\t", termDescription[i].OffsetOutput_bits);
        ptr += k;
        bufferSize -= k;
        bytesToWrite += k;

        k = snprintf(ptr, bufferSize, "BitSizeOut:%d\t", termDescription[i].SizeOutput_bits);
        ptr += k;
        bufferSize -= k;
        bytesToWrite += k;

        k = snprintf(ptr, bufferSize, "BitOffsetIn:%d\t", termDescription[i].OffsetInput_bits);
        ptr += k;
        bufferSize -= k;
        bytesToWrite += k;

        k = snprintf(ptr, bufferSize, "BitSizeIn:%d\t", termDescription[i].SizeInput_bits);
        ptr += k;
        bufferSize -= k;
        bytesToWrite += k;

        k = snprintf(ptr, bufferSize, "Channels:%d\t", termDescription[i].AdditionalInfo.ChannelCount);
        ptr += k;
        bufferSize -= k;
        bytesToWrite += k;

        k = snprintf(ptr, bufferSize, "PiFormat:%d\n", termDescription[i].AdditionalInfo.PiFormat);
        bufferSize -= k; // reduce bufferSize
        ptr += k;     //move buffer pointer to next position
        bytesToWrite += k; //increment how much bytes has to be written.

        while (((written = write(fd_info, buffer, bytesToWrite)) < 0) && errno == EINTR);
        if (written != bytesToWrite)
        {
            dprintf(VERBOSE_STD, "File write %s failed: %s\n", FILE_NAME_TERMINAL_ASSEMBLY, strerror(errno));
            error = -3;
            goto exit;
        }
    }


exit:
    close(fd_count);
    close(fd_info);
    return error;;
}

/**
 * @brief Removing created files
 *
 * @retval 0 on success
 * @retval <0 on failure
 */
int proc_removeEntry(void)
{
    int error = 0;
    if (remove(FILE_NAME_TERMINAL_COUNT) < 0)
    {
        dprintf(VERBOSE_STD, "File delete %s failed: %s\n", FILE_NAME_TERMINAL_COUNT, strerror(errno));
        error = -1;
    }

    if (remove(FILE_NAME_TERMINAL_ASSEMBLY) < 0)
    {
        dprintf(VERBOSE_STD, "File delete %s failed: %s\n", FILE_NAME_TERMINAL_ASSEMBLY, strerror(errno));
        error = -2;
    }

    if (rmdir(FILE_PATH) < 0)
    {
        dprintf(VERBOSE_STD, "RMDIR: %s failed: %s\n", FILE_PATH, strerror(errno));
        error = -3;
    }

    return error;
}
