#ifndef __MODBUS_H__
#define __MODBUS_H__

#include <stdint.h>
#include <modbus/modbus.h>


/**
 * @name Function_codes
 * @brief Modbus function codes
 * @{
 */
#define _FC_READ_COILS                0x01
#define _FC_READ_DISCRETE_INPUTS      0x02
#define _FC_READ_HOLDING_REGISTERS    0x03
#define _FC_READ_INPUT_REGISTERS      0x04
#define _FC_WRITE_SINGLE_COIL         0x05
#define _FC_WRITE_SINGLE_REGISTER     0x06
#define _FC_READ_EXCEPTION_STATUS     0x07
#define _FC_WRITE_MULTIPLE_COILS      0x0F
#define _FC_WRITE_MULTIPLE_REGISTERS  0x10
#define _FC_REPORT_SLAVE_ID           0x11
#define _FC_WRITE_AND_READ_REGISTERS  0x17
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
