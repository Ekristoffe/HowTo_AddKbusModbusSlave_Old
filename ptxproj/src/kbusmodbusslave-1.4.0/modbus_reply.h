#ifndef __MODBUS_REPLY_H__
#define __MODBUS_REPLY_H__

int modbus_reply_offset(modbus_t *ctx, const uint8_t *req, int req_length, modbus_mapping_t *mb_mapping, uint16_t address_offset);

int modbus_replyRegisterCallback( void (*callback)() );

#endif /* __MODBUS_REPLY_H__ */
