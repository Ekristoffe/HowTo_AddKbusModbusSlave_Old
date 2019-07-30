#ifndef __MODBUS_H__
#define __MODBUS_H__

#include <stdint.h>
#include <modbus/modbus.h>


/**
 * @name Function_codes
 * @brief Modbus function codes
 * @{
 */
#define _FC_READ_COILS                  0x01 // Internal Bits Or Physical coils
#define _FC_READ_DISCRETE_INPUTS        0x02 // Physical Discrete Inputs
#define _FC_READ_HOLDING_REGISTERS      0x03 // Internal Registers Or Physical Output Registers
#define _FC_READ_INPUT_REGISTERS        0x04 // Physical Input Registers
#define _FC_WRITE_SINGLE_COIL           0x05 // Internal Bits Or Physical coils
#define _FC_WRITE_SINGLE_REGISTER       0x06 // Internal Registers Or Physical Output Registers
#define _FC_READ_EXCEPTION_STATUS       0x07 // Diagnostics (Unsupported: Serial Line only)
#define _FC_DIAGNOSTICS                 0x08 // Diagnostics (Unsupported: Serial Line only)
#define _FC_GET_COMM_EVENT_COUNTER      0xOB // Diagnostics (Unsupported: Serial Line only)
#define _FC_GET_COMM_EVENT_LOG          0x0C // Diagnostics (Unsupporte: Serial Line onlyd)
#define _FC_WRITE_MULTIPLE_COILS        0x0F // Internal Bits Or Physical coils
#define _FC_WRITE_MULTIPLE_REGISTERS    0x10 // Internal Registers Or Physical Output Registers
#define _FC_REPORT_SLAVE_ID             0x11 // Diagnostics (Unsupported: Serial Line only)
#define _FC_READ_FILE_RECORD            0x14 // File record access (Unsupported: Not implemented in WAGO Slaves)
#define _FC_WRITE_FILE_RECORD           0x15 // File record access (Unsupported: Not implemented in WAGO Slaves)
#define _FC_MASK_WRITE_REGISTER         0x16 // Internal Registers Or Physical Output Registers (Unsupported: TODO)
#define _FC_WRITE_AND_READ_REGISTERS    0x17 // Internal Registers Or Physical Output Registers
#define _FC_READ_FIFO_QUEUE             0x18 // Internal Registers Or Physical Output Registers (Unsupported: Not implemented in WAGO Slaves)
#define _FC_READ_DEVICE_IDENTIFICATION  0x2B // Diagnostics (Unsupported: TODO ?)
/**
 * @}
 */

/**
 * @brief Start kbus thread
 * @retval 0 on success
 * @retval <0 on failure
 */
int modbus_start(void);

/**
 * @brief Stop kbus thread
 */
void modbus_stop(void);

/**
 * @brief Copy data to modbus register
 * @param[in] *source pointer to the source
 * @param[in] n number of words to be copied from source
 * @return number of registers copied from source
 */
int modbus_copy_register_in(uint16_t *source, size_t n);

/**
 * @brief Copy modbus register to datapointer
 * @param[out] *dest pointer to the destination
 * @param[in] n number of bytes to be copied from source
 * @return number of bytes copied from source
 */
int modbus_copy_register_out(uint8_t *dest, size_t n);

void modbus_registerMsgReceivedCallback(void (*funct)());
void modbus_ApplicationStateStop(void);
void modbus_ApplicationStateRun(void);
void modbus_clearAllMappings(void);
modbus_mapping_t *modbus_getWriteMapping(uint16_t *write_address);
modbus_mapping_t *modbus_getReadMapping(uint16_t *read_address);

#endif /* __MODBUS_H__ */
