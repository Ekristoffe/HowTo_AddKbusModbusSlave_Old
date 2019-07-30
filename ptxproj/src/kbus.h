#ifndef __KBUS_H__
#define __KBUS_H__

#include <stdint.h>

typedef struct 
{
    unsigned int series;
    unsigned int value;
    unsigned int spec1;
    unsigned int spec2;
    char *desc_str;
} module_desc_t;

int kbus_start(void);
void kbus_stop(void);

/**
 * @brief Returns the actual KBUS error
 * @return actual error
 * @retval 0 No error
 * @retval >0 actual error
 */
int kbus_getError(void);

/**
 * @return The offset to the digital data in byte
 */
unsigned int kbus_getDigitalByteOffsetOutput(void);
unsigned int kbus_getDigitalByteOffsetInput(void);

unsigned int kbus_getBytesToWrite(void);
unsigned int kbus_getBytesToRead(void);

unsigned char kbus_getIsInitialized(void);

int kbus_getTerminals(size_t *cnt, uint16_t *terminalInfo, size_t sizeTerminalInfo);
int kbus_getBitCounts(uint16_t *table, size_t table_len);
int kbus_ApplicationStateStop(void);
int kbus_ApplicationStateRun(void);
#endif /* __KBUS_H__ */
