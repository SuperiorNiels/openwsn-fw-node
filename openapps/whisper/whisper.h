#ifndef __WHISPER_H
#define __WHISPER_H

/**
\addtogroup AppCoAP
\{
\addtogroup whisper
\{
*/

#include "opencoap.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

typedef struct {
   coap_resource_desc_t desc;
   opentimers_id_t      timerId;
   uint16_t             timerPeriod;
   open_addr_t			whisperDioTarget;
   open_addr_t			whisperParentTarget;
   open_addr_t          whipserNextHopRoot;
   uint8_t 				state;
} whisper_vars_t;

//=========================== prototypes ======================================

void whisper_init(void);
void whisper_setState(uint8_t i);
uint8_t whisper_getState(void);
void whisper_log(char* msg, ...);
void whisper_print_address(open_addr_t* addr);
void whisper_task_remote(uint8_t* buf, uint8_t bufLen);
open_addr_t* whisper_getTargetParentAddress(void);
open_addr_t* whisper_getTargetAddress(void);
open_addr_t* whisper_getNextHopRoot(void);

/**
\}
\}
*/

#endif
