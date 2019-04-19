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
    open_addr_t	target;
    open_addr_t	parent;
    open_addr_t nextHop;
    dagrank_t   rank;
} whisper_dio_settings;

typedef struct {
    bool        acceptACKs;
    open_addr_t acceptACKaddr;
} whisper_ack_sniffing;

typedef struct {
    open_addr_t source;
} whisper_sixtop_request_settings;

typedef struct {
    coap_resource_desc_t desc;
    opentimers_id_t      timerId;
    uint16_t             timerPeriod;
    uint8_t 			 state;
    // Command variables
    whisper_ack_sniffing whisper_ack;
    whisper_dio_settings whisper_dio;
    whisper_sixtop_request_settings whisper_sixtop;
} whisper_vars_t;

//=========================== prototypes ======================================

void whisper_init(void);
void whisper_setState(uint8_t i);
uint8_t whisper_getState(void);
void whisper_task_remote(uint8_t* buf, uint8_t bufLen);

// Whipser Fake dio command
open_addr_t*    getWhisperDIOtarget();
open_addr_t*    getWhisperDIOparent();
open_addr_t*    getWhisperDIOnextHop();
dagrank_t       getWhisperDIOrank();

// Whisper sixtop
open_addr_t*    getWhisperSixtopSource();

// Whisper ACK Sniffing
bool whisperACKreceive(open_addr_t *l2_ack_addr);

// Logging (should be removed for openmote build, no printf..)
void whisper_log(char* msg, ...);
void whisper_print_address(open_addr_t* addr);

/**
\}
\}
*/

#endif
