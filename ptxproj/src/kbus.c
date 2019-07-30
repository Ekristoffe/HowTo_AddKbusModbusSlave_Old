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
///  \file     kbus.c
///
///  \brief    kbus handling
///
///  \author   <BrT> : WAGO Kontakttechnik GmbH & Co. KG
//------------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include "kbus.h"
#include "modbus.h"
#include "utils.h"
#include "proc.h"
#include "conffile_reader.h"

#include <dal/adi_application_interface.h>
#include <ldkc_kbus_information.h>
#include <ldkc_kbus_register_communication.h>
#include <libpackbus.h>
#include <sched.h>
#include <errno.h>

#define KBUS_APPLICATION_STATE ApplicationState_Running /**< @brief Application state for KBUS mode*/

static tApplicationDeviceInterface *adi;
static tDeviceId kbusDeviceId = -1;
static tApplicationStateChangedEvent event; /**< @brief var for the event interface of the ADI*/
static tldkc_KbusInfo_Status status;

static unsigned int offset_input; /**< @brief Process data offset for digital inputs*/
static unsigned int offset_output; /**< @brief Process data offset for digital outputs*/

static uint16_t bytesToRead; /**< @brief Process data read size */
static uint16_t bytesToWrite; /**< @brief Process data write size */
static tldkc_KbusInfo_TerminalInfo terminalDescription[LDKC_KBUS_TERMINAL_COUNT_MAX]; /**< @brief I/O Module detail description*/
static size_t terminalCount; /**< @brief actual I/O Module count*/
static unsigned char kbus_initialized = FALSE; /**< @brief Flag for kbus initialized ready.*/
static module_desc_t modules[LDKC_KBUS_TERMINAL_COUNT_MAX]; /**< @brief Storage for terminal description */
static pthread_mutex_t kbus_update_mutex=PTHREAD_MUTEX_INITIALIZER;

static void kbus_update(void);
//---------------------------------------------------------------------------------------------------------------------------------
// Timer declaration
//---------------------------------------------------------------------------------------------------------------------------------
#include <signal.h>
#include <time.h>
static timer_t kbus_timerID; //*< @brief Global timer handle*/

/**
 * @brief Setting the interval time for timer
 *
 * @param[in] timerID Timer handle
 * @param[in] intervalMS Interval in ms
 */
static void kbus_timerSetTime(timer_t timerID, int intervalMS)
{
    struct itimerspec       its;
    memset((void*)&its, 0, sizeof(its));

    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = intervalMS * 1000000;
    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = intervalMS * 1000000;
    timer_settime(timerID, 0, &its, NULL);
}

/**
 * @brief Callback function for timer
 */
static void kbus_timerHandler( int sig, siginfo_t *si, void *uc )
{
    sig=sig;
    si = si;
    uc = uc;
    kbus_update();
}

/**
 * @brief Create timer and setting up the interval time. 
 * If timer expired it will signal via SIGRTMIN.
 *
 * @param[in] timerID Timer handle
 * @param[in] intervalMS Interval in ms
 *
 * @retval 0 on success
 * @retval <0 on failure
 */
static int kbus_timerSetup(timer_t *timerID, int intervalMS)
{
    struct sigevent         te;
    struct sigaction        sa;
    int                     sigNo = SIGRTMIN;

    /* Set up signal handler. */
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = kbus_timerHandler;
    sigemptyset(&sa.sa_mask);
    if (sigaction(sigNo, &sa, NULL) == -1)
    {
        dprintf(VERBOSE_STD, "Failed to setup signal handling for kbus timer.\n");
        return(-1);
    }

    /* Set and enable alarm */
    te.sigev_notify = SIGEV_SIGNAL;
    te.sigev_signo = sigNo;
    te.sigev_value.sival_ptr = timerID;
    timer_create(CLOCK_REALTIME, &te, timerID);

    kbus_timerSetTime(*timerID, intervalMS);
    return 0;
}

/**
 * @brief Delete timer
 *
 * @param[in] timerID Timer handle
 */
static void kbus_timerDelete(timer_t *timerID)
{
    timer_delete(timerID);
}

//---------------------------------------------------------------------------------------------------------------------------------
// KBUS Specific
//---------------------------------------------------------------------------------------------------------------------------------

/**
 * @brief Initialize the kbus connection, scan for libpackkbus device to use.
 * @retval 0 on success
 * @retval <0 on failure
 */
static int kbus_open(void)
{
    tDeviceInfo deviceList[10];          // the list of devices given by the ADI
    int kbusId = -1;
    size_t i;
    size_t ndevices;

    adi = adi_GetApplicationInterface();

    if (adi == NULL)
    {
        dprintf(VERBOSE_STD, "Failed to get application Interface\n");
        return -1;
    }

    adi->Init();
    adi->ScanDevices();
    adi->GetDeviceList(sizeof(deviceList), deviceList, &ndevices);

    for ( i = 0; i < ndevices; i++)
    {
        dprintf(VERBOSE_INFO, "ADI Device[%d]: %s\n", i, deviceList[i].DeviceName);

        if (strcmp(deviceList[i].DeviceName, "libpackbus") == 0)
        {
            dprintf(VERBOSE_STD, "Found kbus device on: %d\n", i);
            kbusId = i;
            break; // leave loop
        }
    }

    //If no KBUS-Device is found
    if (kbusId == -1)
    {
        fprintf(stderr, "No KBUS device found\n");
        adi->Exit();
        return -2;
    }

    // open KBUS-Device
    kbusDeviceId = deviceList[kbusId].DeviceId;
    if (adi->OpenDevice(kbusDeviceId) != DAL_SUCCESS)
    {
        fprintf(stderr, "KBUS open device failure.\n");
        adi->Exit();
        return -3;
    }

    dprintf(VERBOSE_STD, "KBUS device open OK\n");
    return 0;
}

/**
 * @brief Set the application mode for kbus
 * @retval 0 on success
 * @retval <0 on failure
 */
static int kbus_setMode(tApplicationState ev)
{
    if (adi == NULL)
    {
        fprintf(stderr, "ADI not vaild\n");
        return -1;
    }

    if (kbusDeviceId < 0)
    {
        fprintf(stderr, "Device ID invalid\n");
        return -2;
    }

    event.State = ev;
    if (adi->ApplicationStateChanged(event) != DAL_SUCCESS)
    {
        //Set application state failed
        fprintf(stderr, "Set application state failed\n");
        adi->CloseDevice(kbusDeviceId);
        adi->Exit();
        return -3;
    }

    dprintf(VERBOSE_STD, "KBUS set to application state: %d\n", ev);
    return 0;
}

/**
 * @brief Setting up kbus and configuration
 * @retval 0 on success
 * @retval <0 on failure
 */
static int kbus_setConfig(void)
{
    if (adi == NULL)
    {
        fprintf(stderr, "ADI not vaild\n");
        return -1;
    }

    if (ldkc_KbusInfo_Create() == KbusInfo_Failed)
    {
        fprintf(stderr, "KbusInfo_Create failed\n");
        adi->CloseDevice(kbusDeviceId);
        adi->Exit();
        return -2;
    }

    return 0;
}

/**
 * @brief Getting kbus information like process data length, I/O Module count,
 * kbus error.
 * @retval 0 on success
 * @retval <0 on failure
 */
static int kbus_getStatus(void)
{
    if (adi == NULL)
    {
        fprintf(stderr, "ADI not vaild\n");
        return -1;
    }

    if (ldkc_KbusInfo_GetStatus(&status) == KbusInfo_Failed)
    {
        dprintf(VERBOSE_DEBUG, "ldkc_KbusInfo_GetStatus() failed\n");
        adi->CloseDevice(kbusDeviceId);
        adi->Exit();
        ldkc_KbusInfo_Destroy();
        return -2;
    }

    dprintf(VERBOSE_INFO, "\n        .KbusBitCount: %i ",status.KbusBitCount);
    dprintf(VERBOSE_INFO, "\n        .TerminalCount: %i ",status.TerminalCount);
    dprintf(VERBOSE_INFO, "\n        .ErrorCode: %i ",status.ErrorCode);
    dprintf(VERBOSE_INFO, "\n        .ErrorArg: %i ",status.ErrorArg);
    dprintf(VERBOSE_INFO, "\n        .ErrorPos: %i ",status.ErrorPos);
    dprintf(VERBOSE_INFO, "\n        .BitCountAnalogInput: %i ",status.BitCountAnalogInput);
    dprintf(VERBOSE_INFO, "\n        .BitCountAnalogOutput: %i ",status.BitCountAnalogOutput);
    dprintf(VERBOSE_INFO, "\n        .BitCountDigitalInput: %i ",status.BitCountDigitalInput);
    dprintf(VERBOSE_INFO, "\n        .BitCountDigitalOutput: %i ",status.BitCountDigitalOutput);

    if (ldkc_KbusInfo_GetDigitalOffset((u16 *)&offset_input, (u16*) &offset_output) == KbusInfo_Failed)
    {
        dprintf(VERBOSE_DEBUG, "ldkc_KbusInfo_GetDigitalOffset() failed\n");
        adi->CloseDevice(kbusDeviceId);
        adi->Exit();
        ldkc_KbusInfo_Destroy();
    }

    dprintf(VERBOSE_STD, "\nOffset: IN: %u - OUT: %u\n", offset_input, offset_output);
    return 0;
}

/**
 * @brief Getting the detailed terminal description via libpackbus.
 * It will be created also a description string which has to be freed
 * if not longer used.
 *
 * @param[in] cnt Number of terminals
 *
 * @retval 0 on success
 * @retval <0 on failure
 */
static int kbus_getTerminalType(int cnt)
{
    unsigned short value;
    unsigned int result;
    int i = 1;

    for( i = 1; i <= cnt; i++)
    {
        adi->CallDeviceSpecificFunction(LIBPACKBUS_DAL_FUNC_READ_TAB_9,
                                        &result, i, &value);
        if ((result == 0) && (value != 0))
        {
            modules[i-1].series = 750;
            modules[i-1].value = value; //store data
            modules[i-1].spec1 = 0;
            modules[i-1].spec2 = 0;

            if ((value & 0x8000) == 0)// if non digital I/O
            {

                adi->CallDeviceSpecificFunction(LIBPACKBUS_DAL_FUNC_READ_CONF_REG,
                                                &result, i, 16, &value);

                if ((result == 0) && (value & 0x100)) //series 753
                {
                    modules[i-1].series = 753;
                }

                adi->CallDeviceSpecificFunction(LIBPACKBUS_DAL_FUNC_READ_CONF_REG,
                                                &result, i, 30, &value);

                //----LET THE MAGIC HAPPEN----?????-----
                if ((result == 0) && (value != 0))
                {
                    value %= 10;
                    if (value != 9)
                    {
                        modules[i-1].spec2 = value;
                    }
                    else
                    {
                        adi->CallDeviceSpecificFunction(LIBPACKBUS_DAL_FUNC_READ_CONF_REG,
                                                        &result, i, 29, &value);

                        if (result == 0)
                        {
                            modules[i-1].spec1 = value;
                        }

                        adi->CallDeviceSpecificFunction(LIBPACKBUS_DAL_FUNC_READ_CONF_REG,
                                                        &result, i, 28, &value);

                        if (result == 0)
                        {
                            modules[i-1].spec2 = value;
                        }
                    }
                }
            }
        }
        else
        {
            return -1;

        }
    }

    //Create description string
    for (i = 1; i <= cnt; i++)
    {
        char buffer[50];
        size_t written=0;
        unsigned char channels = ((modules[i-1].value >> 8) & 0x7F);

        written = snprintf(buffer, OS_ARRAY_SIZE(buffer), "%u-", modules[i-1].series);
        if (modules[i-1].value & 0x8000)
        {
            if ((modules[i-1].value & 0x03) == 0x03) // DO-DIAG
            {
                written += snprintf(&buffer[written], OS_ARRAY_SIZE(buffer)-written, "5XX / %uDO-DIAG", channels);
            }
            else if (modules[i-1].value & 0x01) // DI
            {
                written += snprintf(&buffer[written], OS_ARRAY_SIZE(buffer)-written, "4XX / %uDI", channels);
            }
            else if (modules[i-1].value & 0x02) // DO
            {
                written += snprintf(&buffer[written], OS_ARRAY_SIZE(buffer)-written, "5XX / %uDO", channels);
            }
        }
        else
        {
            written += snprintf(&buffer[written], OS_ARRAY_SIZE(buffer)-written, "%u / %u-%u", modules[i-1].value,
                               modules[i-1].spec1, modules[i-1].spec2);
        }

        buffer[written] = '\0';
        written++; //written + '\0'-termination

        modules[i-1].desc_str = malloc(written);
        if (modules[i-1].desc_str == NULL)
        {
            dprintf(VERBOSE_STD, "Unable to allocate memory\n");
            return -2;
        }
        strncpy(modules[i-1].desc_str, buffer, written);
    }
    return 0;
}

/**
 * @brief Free module description sting which was allocated by
 * kbus_getTerminalType
 *
 * @param[in] cnt Number of terminals
 */
static void kbus_freeModulesDescString(int cnt)
{
    int i = 0;
    for (i = 0; i<cnt; i++)
    {
        if (modules[i].desc_str != NULL)
        {
            free(modules[i].desc_str);
        }
    }
}

/**
 * @brief Get I/O Module information
 * @retval 0 on success
 * @retval <0 on failure
 */
static int kbus_getTerminalInfo(void)
{
    unsigned char ucPosition;
    unsigned char ucIndex;
    unsigned char ucMaxPosition;

    if (adi == NULL)
    {
        fprintf(stderr, "ADI not vaild\n");
        return -1;
    }

    if (ldkc_KbusInfo_GetTerminalInfo(OS_ARRAY_SIZE(terminalDescription), terminalDescription, &terminalCount) == KbusInfo_Failed)
    {
        fprintf(stderr, "ldkc_KbusInfo_GetTerminalInfo() failed\n");
        adi->CloseDevice(kbusDeviceId);
        adi->Exit();
        ldkc_KbusInfo_Destroy();
        return -2;
    }

    if (kbus_getTerminalType(terminalCount) < 0)
    {
        return -3;
    }

    ucPosition    = 1;
    ucMaxPosition = terminalCount;

    for (ucIndex = 0; ucPosition <= ucMaxPosition; ucPosition++, ucIndex++)
    {
        const u32 idx = ucPosition - 1;

        dprintf(VERBOSE_INFO, "\n Pos:%i:", ucPosition);
        dprintf(VERBOSE_INFO, "\t Type: %s", modules[idx].desc_str);
        dprintf(VERBOSE_INFO, "\t BitOffsetOut:%i;", terminalDescription[idx].OffsetOutput_bits);
        dprintf(VERBOSE_INFO, "\t BitSizeOut:%i;", terminalDescription[idx].SizeOutput_bits);
        dprintf(VERBOSE_INFO, "\t BitOffsetIn:%i;", terminalDescription[idx].OffsetInput_bits);
        dprintf(VERBOSE_INFO, "\t BitSizeIn:%i;", terminalDescription[idx].SizeInput_bits);
        dprintf(VERBOSE_INFO, "\t Channels:%i;", terminalDescription[idx].AdditionalInfo.ChannelCount);
        dprintf(VERBOSE_INFO, "\t PiFormat:%i;", terminalDescription[idx].AdditionalInfo.PiFormat);
    }

    dprintf(VERBOSE_INFO, "\n");

    return 0;
}

/**
 * @brief Close kbus device and destroy all created context
 * @retval 0 on success
 * @retval <0 on failure
 */
static int kbus_close(void)
{
    if (adi == NULL)
    {
        fprintf(stderr, "ADI not vaild\n");
        return -1;
    }
    dprintf(VERBOSE_STD, "KBUS_CLOSE\n");
    adi->CloseDevice(kbusDeviceId);
    adi->Exit();
    ldkc_KbusInfo_Destroy();
    kbus_freeModulesDescString(terminalCount);
    // remove /proc "/tmp" entry
    proc_removeEntry();
    return 0;
}

/**
 * @brief Map processdata bitcount to Word(16bit) register count
 * @return number of registers
 */
static uint16_t kbus_mapBitCountToWordRegister(uint16_t count)
{
    uint16_t n = count / 16;

    if (count % 16 > 0)
        n = n + 1;
    return n;
}

/**
 * @brief Getting the actual process data length in bits.
 * @return Number of bits for output
 */
static uint16_t kbus_getBitCount_Output(void)
{
    uint16_t n = 0;

    n = status.BitCountAnalogOutput;
    n += status.BitCountDigitalOutput;

    return n;
}

/**
 * @brief Getting the actual process data length in bits.
 * @return Number of bits for input
 */
static uint16_t kbus_getBitCount_Input(void)
{
    uint16_t n = 0;

    n = status.BitCountAnalogInput;
    n += status.BitCountDigitalInput;

    return n;
}

/**
 * @brief Helper function to initialize the kbus communication.
 * It runs kbus_open, kbus_setup, kbus_setConfig, kbus_getStatus,kbus_getTerminalInfo.
 * If everythins is succesfull the flag kbus_initialized is set and the filesystem
 * information about terminalCount and terminals are written by proc_createEntry.
 * @retval 0 on success
 * @retval <0 on failure
 */
static int kbus_setup(void)
{
    if (kbus_open() < 0)
        return -1;
    if (kbus_setMode(KBUS_APPLICATION_STATE) < 0)
        return -2;
    if (kbus_setConfig() < 0)
        return -3;
    if (kbus_getStatus() < 0)
        return -4;
    if (kbus_getTerminalInfo() < 0)
        return -5;

    kbus_initialized = TRUE;
    //Create /proc "/tmp" entry
    proc_createEntry(terminalCount, modules, terminalDescription);

    bytesToRead = utils_bitCountToByte(kbus_getBitCount_Input());
    bytesToWrite= utils_bitCountToByte(kbus_getBitCount_Output());

    return 0;
}

/**
 * @brief Helper function to reset the kbus. Will reset the flag kbus_initialized and execute
 * kbus_close and afterwards kbus_setup.
 * @retval 0 on success
 * @retval <0 on failure
 */
static int kbus_reset(void)
{
    kbus_initialized = FALSE;
    if (kbus_close() < 0)
        return -1;
    if (kbus_setup() < 0)
        return -2;

    return 0;
}

/**
 * @brief A endless loop to wait till kbus error is gone.
 * Error checking is done every 1sec.
 */
static void kbus_loopTilErrorGone(void)
{
    while (1)
    {
        uint32_t retval = 0;
        // Use function "libpackbus_Push" to trigger one KBUS cycle.
        if (adi->CallDeviceSpecificFunction("libpackbus_Push", &retval) != DAL_SUCCESS)
        {
            // CallDeviceSpecificFunction failed
            dprintf(VERBOSE_STD, "CallDeviceSpecificFunction failed\n");
        }
        //retval is alwaya non successfull because we are in error state.
        // Function 'libpackbus_Push' successfull
        //if (retval == DAL_SUCCESS)
        {
            adi->WatchdogTrigger();

            int error = kbus_getError();
            dprintf(VERBOSE_DEBUG, " !!!! KBUS ERROR: %d\n", error);
            if (error == 0) //no error
            {
                dprintf(VERBOSE_DEBUG, "NO KBUS ERROR\n");
                return;
            }

        }
        usleep(50*1000);
    }
}

static int taskId = 0; //not used

// process data
static uint8_t pd_in[4096];    // kbus input process data
static uint8_t pd_out[4096];   // kbus output process data

static void kbus_update(void)
{
    if(pthread_mutex_trylock(&kbus_update_mutex) != 0)
    {
        return; //Unable to lock mutex - process is active
    }

    //Error Check
    if (kbus_getError())
    {
        kbus_timerSetTime(&kbus_timerID, 0); //Deactivate timer
        dprintf(VERBOSE_DEBUG, "-------------------------- KBUS ERROR -------------------\n");
        kbus_loopTilErrorGone();
        modbus_clearAllMappings();
        kbus_reset();
        kbus_timerSetTime(&kbus_timerID, conf_kbus_cycle_ms); //Reset to orginal-timer cycle

    }
    else
    {
        // Flow:
        //  1) Initiate a KBUS cycle
        //  2) Watchdog Trigger
        //  3) Write
        //  4) Read
        //
        uint32_t retval = 0;
        // Use function "libpackbus_Push" to trigger one KBUS cycle.
        if (adi->CallDeviceSpecificFunction("libpackbus_Push", &retval) != DAL_SUCCESS)
        {
            // CallDeviceSpecificFunction failed
            dprintf(VERBOSE_STD, "CallDeviceSpecificFunction failed\n");
            goto exit;
        }

        // Function 'libpackbus_Push' successfull
        if (retval == DAL_SUCCESS)
        {

            adi->WatchdogTrigger();

            //Get Modbus write data copy it to KBUS
            int ret= modbus_copy_register_out(pd_out, sizeof(pd_out));
            if (ret < 0)
            {
                dprintf(VERBOSE_DEBUG, "[KBUS] Mapping write failed: %d\n", ret);
            }

            //Write KBUS
            adi->WriteStart(kbusDeviceId, taskId); // lock PD-out data
            adi->WriteBytes(kbusDeviceId, taskId, 0, bytesToWrite,
                            (uint8_t *) &pd_out[0]); // write input back
            adi->WriteEnd(kbusDeviceId, taskId); // unlock PD-out data

            adi->ReadStart(kbusDeviceId, taskId);       // lock PD-In data
            adi->ReadBytes(kbusDeviceId, taskId, 0, bytesToRead, (uint8_t *) &pd_in[0]);
            adi->ReadEnd(kbusDeviceId, taskId); // unlock PD-In data

            //Copy KBUS data to modbus read
            ret = modbus_copy_register_in((uint16_t *)pd_in, kbus_mapBitCountToWordRegister(kbus_getBitCount_Input()));
            if (ret < 0)
       {
                dprintf(VERBOSE_DEBUG, "[KBUS] Mapping read failed: %d\n", ret);
            }
        }
    }
exit:
    pthread_mutex_unlock(&kbus_update_mutex);
}

/**
 * @brief Force an async update of KBUS only if in coupler mode
 */
static void kbus_forceUpdate(void)
{
    // Only allow force update if coupler-mode is set.
    if (conf_operation_mode)
    {
        dprintf(VERBOSE_DEBUG, "KBUS Force Update\n");
        kbus_timerSetTime(&kbus_timerID, 0); //Deactivate timer
        kbus_update();
        kbus_timerSetTime(&kbus_timerID, conf_kbus_cycle_ms); //Reset to orginal-timer cycle
    }
}

/**
 * @brief Set the nice value of the kbus process
 * @param[in] priority Value to be set.
 * @retval 0 on success
 * @retval <0 on failure
 */
static int kbus_setRTPriority(int priority)
{
    int ret = 0;
    struct sched_param s_param;
    s_param.sched_priority = priority;
    ret = sched_setscheduler(0, SCHED_FIFO, &s_param);

    if (ret < 0)
    {
        dprintf(VERBOSE_DEBUG, "Set Priority failed: %d - %s", ret, strerror(errno));
        return -1;
    }

    dprintf(VERBOSE_DEBUG, "Set Priority: %d successfully\n", priority);
    return 0;
}

/**
 * @brief Start kbus thread
 * @retval 0 on success
 * @retval <0 on failure
 */
int kbus_start(void)
{
    if (kbus_setup() < 0)
    {
        return -1;
    }

    pthread_mutex_init(&kbus_update_mutex, NULL);
    modbus_registerMsgReceivedCallback(kbus_forceUpdate);
    kbus_setRTPriority(conf_kbus_priority);
    if (kbus_timerSetup(&kbus_timerID, conf_kbus_cycle_ms) < 0)
    {
        return -2;
    }
    return 0;
}

/**
 * @brief Stop kbus thread
 */
void kbus_stop(void)
{
    kbus_timerDelete(kbus_timerID);
    kbus_close(); // ignore return-value
    pthread_mutex_destroy(&kbus_update_mutex);
    kbus_initialized = FALSE;
    dprintf(VERBOSE_STD, "KBUS_STOP\n");
}

/**
 * @brief Get kbus error
 * @return Returns the actual kbus error code.
 */
int kbus_getError(void)
{
    if (ldkc_KbusInfo_GetStatus(&status) == KbusInfo_Failed)
    {
        dprintf(VERBOSE_DEBUG,"ldkc_KbusInfo_GetStatus() failed\n");
    }
    return status.ErrorCode;
}

/**
 * @brief Returns the process data offset for digital I/O Modules - Output
 * @return Offset in bytes to the first digital I/O Module
 */
unsigned int kbus_getDigitalByteOffsetOutput(void)
{
    return offset_output;
}

/**
 * @brief Returns the process data offset for digital I/O Modules - Input
 * @return Offset in bytes to the first digital I/O Module
 */
unsigned int kbus_getDigitalByteOffsetInput(void)
{
    return offset_input;
}

/**
 * @brief Returns the process data length to write in bytes
 * @return Number of bytes
 */
unsigned int kbus_getBytesToWrite(void)
{
    return bytesToWrite;
}

/**
 * @brief Returns the process data length to read in bytes
 * @return Number of bytes
 */
unsigned int kbus_getBytesToRead(void)
{
    return bytesToRead;
}

/**
 * @brief Copy terminal information to given pointer
 * @param[out] *cnt - pointer for terminal count
 * @param[out] *terminalInfo - pointer for I/O Module table
 * @param[in] sizeTerminalInfo - size of I/O Module table
 * @retval 0 on success
 * @retval <0 on failure
 */
int kbus_getTerminals(size_t *cnt, uint16_t *terminalInfo, size_t sizeTerminalInfo)
{
    if (sizeTerminalInfo < terminalCount)
        return -1;
    if (terminalInfo == NULL)
        return -2;
    if (cnt == NULL)
        return -3;

    //Copy count
    *cnt = terminalCount;

    //copy terminal info
    memset(terminalInfo, 0, sizeTerminalInfo);
    uint16_t i = 0;
    for (i = 0; i < terminalCount; i++)
    {
        terminalInfo[i] = modules[i].value;
    }

    return 0;
}

/**
 * @brief Returns status of flag: kbus_initialized
 * @retval 0: Not initialized
 * @retval 1: Initalized
 */
unsigned char kbus_getIsInitialized(void)
{
    return kbus_initialized;
}

/**
 * @brief Returns the PII/PIO bits for analog and digital
 * @param[out] table Holds the information about the PII/POO bits
 * table[0] PIO analog bits
 * table[1] PII analog bits
 * table[2] PIO digital bits
 * table[3] PII digital bits
 * @param[in] table_len Size of table
 * @retval -3: table pointer invalid
 * @retval -2: table size to small
 * @retval -1: Not initialized
 * @retval  0: success
 */
int kbus_getBitCounts(uint16_t *table, size_t table_len)
{
    if (kbus_initialized == FALSE)
    {
        return -1;
    }

    if (table_len < 4)
    {
        return -2;
    }

    if (table == NULL)
    {
        return -3;
    }

    table[0] = status.BitCountAnalogOutput;
    table[1] = status.BitCountAnalogInput;
    table[2] = status.BitCountDigitalOutput;
    table[3] = status.BitCountDigitalInput;

    return 0;

}

/**
 * @brief Handler function to set the correct behavior if OMS is switched
 * to stop.
 * @return Returns the error of kbus_setMode
 */
int kbus_ApplicationStateStop(void)
{
    //Set KBUS timer to 5ms to give I/OCheck more speed
    kbus_timerSetTime(&kbus_timerID, 5);
    return kbus_setMode(ApplicationState_Stopped);
}

/**
 * @brief Handler function to set the correct behavior if OMS is switched
 * to run.
 * @return Always 0
 */
int kbus_ApplicationStateRun(void)
{
    kbus_setMode(KBUS_APPLICATION_STATE);
    kbus_timerSetTime(&kbus_timerID, conf_kbus_cycle_ms); //Reset to orginal-timer cycle
    return 0;
}
