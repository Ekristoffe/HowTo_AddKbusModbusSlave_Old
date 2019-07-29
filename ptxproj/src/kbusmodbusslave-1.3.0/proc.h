#ifndef __PROC_H__
#define __PROC_H__

#include <ldkc_kbus_information.h>
#include "kbus.h"

int proc_createEntry(size_t terminalCnt, module_desc_t *modules, tldkc_KbusInfo_TerminalInfo *termDescription);

int proc_removeEntry(void);


#endif /* __PROC_H__ */
