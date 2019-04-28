/**
\brief Whisper module, runs on the (DAG)root only and is responsible for sending fake dio's
*/

#include <stdarg.h>

#include "opendefs.h"
#include "whisper.h"
#include "opencoap.h"
#include "openqueue.h"
#include "packetfunctions.h"
#include "openserial.h"
#include "openrandom.h"
#include "board.h"
#include "idmanager.h"
#include "msf.h"
#include "neighbors.h"
#include "schedule.h"
#include "IEEE802154E.h"
#include "scheduler.h"
#include "sixtop.h"
#include "leds.h"
#include "icmpv6rpl.h"

//=========================== defines =========================================

const uint8_t whisper_path0[] = "w";

//=========================== variables =======================================

whisper_vars_t whisper_vars;

//=========================== prototypes ======================================

owerror_t whisper_receive(OpenQueueEntry_t* msg,
						coap_header_iht*  coap_header,
						coap_option_iht*  coap_incomingOptions,
						coap_option_iht*  coap_outgoingOptions,
						uint8_t*          coap_outgoingOptionsLen);

void whisper_sendDone(OpenQueueEntry_t* msg, owerror_t error);

void whisper_timer_cb(opentimers_id_t id);

//=========================== public ==========================================

/**
\brief Initialize this module.
*/
void whisper_init() {
	whisper_log("Initializing whisper node.\n");

	whisper_vars.state = WHISPER_STATE_IDLE;

    whisper_vars.my_addr.type = ADDR_128B;
    memcpy(&whisper_vars.my_addr.addr_128b, 		idmanager_getMyID(ADDR_PREFIX)->prefix, 8);
    memcpy(&whisper_vars.my_addr.addr_128b[8], 	    idmanager_getMyID(ADDR_64B)->addr_64b, 8);

	// prepare the resource descriptor for the /w path
	whisper_vars.desc.path0len             = sizeof(whisper_path0)-1;
	whisper_vars.desc.path0val             = (uint8_t*)(&whisper_path0);
	whisper_vars.desc.path1len             = 0;
	whisper_vars.desc.path1val             = NULL;
	whisper_vars.desc.componentID          = COMPONENT_WHISPER;
	whisper_vars.desc.discoverable         = TRUE;
	whisper_vars.desc.callbackRx           = &whisper_receive;
	whisper_vars.desc.callbackSendDone     = &whisper_sendDone;

    whisper_setState(0);

	// register with the CoAP module
	opencoap_register(&whisper_vars.desc);

	// Start timer for data collection (on root)
    whisper_vars.timerPeriod = 5000; // 5 seconds
    whisper_vars.periodicTimer = opentimers_create(TIMER_GENERAL_PURPOSE, TASKPRIO_RPL);
    whisper_vars.oneshotTimer = opentimers_create(TIMER_GENERAL_PURPOSE, TASKPRIO_RPL);
    opentimers_scheduleIn(
            whisper_vars.periodicTimer,
            whisper_vars.timerPeriod,
            TIME_MS,
            TIMER_PERIODIC,
            whisper_timer_cb
    );
}

void whisper_setState(uint8_t i) {
	whisper_vars.state=i;
}

uint8_t whisper_getState() {
	return whisper_vars.state;
}

void whisper_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
    openqueue_freePacketBuffer(msg);
}

void whisper_timer_cb(opentimers_id_t id) {
    leds_error_toggle();
}

void whisper_task_remote(uint8_t* buf, uint8_t bufLen) {
    // Serial communication (Rx) can be used to provide out band communication between node and controller
}

owerror_t whisper_receive(OpenQueueEntry_t* msg,
						coap_header_iht*  coap_header,
						coap_option_iht*  coap_incomingOptions,
						coap_option_iht*  coap_outgoingOptions,
						uint8_t*          coap_outgoingOptionsLen)
{
	owerror_t outcome;

	switch (coap_header->Code) {
		case COAP_CODE_REQ_GET:
		    whisper_log("Received CoAP GET.\n");
			// To reuse the packetBuffer we need to reset the message payload
			msg->payload = &(msg->packet[127]);
			msg->length = 0;

			// Create data and
			const int8_t respose_payload[] = "Whisper node Loaded.";
			packetfunctions_reserveHeaderSize(msg,sizeof(respose_payload)-1);
			memcpy(&msg->payload[0],&respose_payload,sizeof(respose_payload)-1);

			// Set COAP header code
			coap_header->Code = COAP_CODE_RESP_CONTENT;
			outcome = E_SUCCESS;
			break;

        case COAP_CODE_REQ_PUT:
            whisper_log("Received CoAP PUT.\n");

            // Don's execute command if state is not idle
            if(whisper_vars.state == WHISPER_STATE_IDLE) {
                switch(msg->payload[1]) {
                    case 0x01:
                        whisper_log("Whisper fake dio command (remote)\n");
                        whisperDioCommand(msg->payload);
                        break;
                    case 0x02:
                        whisper_log("Whisper 6P command (remote).\n");
                        if(whisperSixtopParse(msg->payload)) whisperExecuteSixtop();
                        else whisper_log("Parsing command failed.\n");
                        break;
                    default:
                        break;
                }
            } else {
                whisper_log("Whisper node not idle, abort.\n");
                // Notify controller
                break;
            }

            // reset packet payload
            msg->payload                     = &(msg->packet[127]);
            msg->length                      = 0;
            coap_header->Code                = COAP_CODE_RESP_CHANGED;
            outcome                          = E_SUCCESS;

            break;
		default:
			outcome = E_FAIL;
			break;
	}

	return outcome;
}

// ---------------------- Whisper DIO --------------------------

open_addr_t* getWhisperDIOtarget() {
    return &whisper_vars.whisper_dio.target;
}

open_addr_t* getWhisperDIOparent() {
    return &whisper_vars.whisper_dio.parent;
}

open_addr_t* getWhisperDIOnextHop() {
    return &whisper_vars.whisper_dio.nextHop;
}

dagrank_t getWhisperDIOrank() {
    return whisper_vars.whisper_dio.rank;
}

open_addr_t* getWhisperSixtopSource() {
    return &whisper_vars.whisper_sixtop.source;
}

bool whisperACKreceive(open_addr_t *l2_ack_addr) {
    if(whisper_vars.whisper_ack.acceptACKs == TRUE) {
        if(packetfunctions_sameAddress(l2_ack_addr, &whisper_vars.whisper_ack.acceptACKaddr)) {
            whisper_log("ACK received.\n");
            whisper_vars.whisper_ack.acceptACKs = FALSE;
            return TRUE;
        }
    }
    return FALSE;
}

void whisperDioCommand(const uint8_t* command) {
    open_addr_t temp;

    // Target
    whisper_vars.my_addr.addr_128b[14] = command[2];
    whisper_vars.my_addr.addr_128b[15] = command[3];
    memcpy(&whisper_vars.whisper_dio.target, &whisper_vars.my_addr, sizeof(open_addr_t));

    // Parent
    whisper_vars.my_addr.addr_128b[14] = command[4];
    whisper_vars.my_addr.addr_128b[15] = command[5];
    memcpy(&whisper_vars.whisper_dio.parent, &whisper_vars.my_addr, sizeof(open_addr_t));

    // Next Hop == target
    memcpy(&whisper_vars.whisper_dio.nextHop, &whisper_vars.whisper_dio.target, sizeof(open_addr_t));

    whisper_vars.whisper_dio.rank = (uint16_t) ((uint16_t) command[8] << 8) | (uint16_t ) command[9];

    whisper_log("Sending fake DIO with rank %d.\n", whisper_vars.whisper_dio.rank);

    uint8_t result = send_WhisperDIO();

    // If DIO is send successfully (to lower layers) activate ACK sniffing if the parent is not myself
    if(result == E_SUCCESS && (idmanager_isMyAddress(&whisper_vars.whisper_dio.parent) == FALSE)) {
        whisper_vars.whisper_ack.acceptACKaddr.type = ADDR_64B;
        // Set ACK receiving ACK adderss to dio target
        packetfunctions_ip128bToMac64b(&whisper_vars.whisper_dio.target,&temp,&whisper_vars.whisper_ack.acceptACKaddr);
        whisper_vars.whisper_ack.acceptACKs = TRUE;
    }

    // Notify controller when ACK is received
}

// ---------------------- Whisper Sixtop --------------------------

bool whisperSixtopAddAutonomousCell() {
    uint8_t id_lsb = whisper_vars.whisper_sixtop.target.addr_64b[7];
    uint8_t id_msb = whisper_vars.whisper_sixtop.target.addr_64b[6];
    uint16_t slotOffset = msf_hashFunction_getSlotoffset((uint16_t) (256 * id_msb + id_lsb));

    if(schedule_isSlotOffsetAvailable(slotOffset)) {
        owerror_t outcome = schedule_addActiveSlot(
                slotOffset,                                                             // slot offset
                CELLTYPE_RX,                                                            // type of slot
                TRUE,                                                                  // shared?
                msf_hashFunction_getChanneloffset((uint16_t) (256 * id_msb + id_lsb)),  // channel offset
                &whisper_vars.whisper_sixtop.target                                     // neighbor
        );
        return (uint8_t) (outcome == E_SUCCESS);
    }

    return icmpv6rpl_isPreferredParent(&whisper_vars.whisper_sixtop.target);
}

void whisperSixtopRemoveAutonomousCell() {
    // Remove autonomous cell
    if (icmpv6rpl_isPreferredParent(&whisper_vars.whisper_sixtop.target) == FALSE) {
        uint8_t id_lsb = whisper_vars.whisper_sixtop.target.addr_64b[7];
        uint8_t id_msb = whisper_vars.whisper_sixtop.target.addr_64b[6];
        schedule_removeActiveSlot(msf_hashFunction_getSlotoffset((uint16_t) (256 * id_msb + id_lsb)),
                                  &whisper_vars.whisper_sixtop.target);
        whisper_log("Sixtop response received, removing autonomous cell of target.\n");
        return;
    }
    whisper_log("Not removing autonomous cell of target (parent).\n");
}

void whisperSixtopResonseReceive(open_addr_t* addr, uint8_t code) {
    if(whisper_vars.whisper_sixtop.waiting_for_response) {
        if (packetfunctions_sameAddress(addr, &whisper_vars.whisper_sixtop.target)) {

            switch(code) {
                case IANA_6TOP_RC_SUCCESS:
                    whisper_log("Whisper 6P command successful.\n");
                    // Notify controller
                    break;
                case IANA_6TOP_RC_SEQNUM_ERR:
                    whisper_log("Whisper 6P command wrong seqNum.\n");
                default:
                    whisper_log("Whisper 6P failed.\n");
                    // Notify controller
                    break;
            }

            whisperSixtopRemoveAutonomousCell();
            whisper_vars.whisper_sixtop.waiting_for_response = FALSE;
            whisper_vars.state = WHISPER_STATE_IDLE;

        } else {
            whisper_log("Sixtop response received from wrong address.\n");
        }
    } else {
        whisper_log("Not waiting for a 6p response, yet we got here...\n");
    }
}

bool whisperSixtopPacketAccept(ieee802154_header_iht *ieee802514_header) {
    if(whisper_vars.whisper_sixtop.waiting_for_response) {
        if (packetfunctions_sameAddress(&whisper_vars.whisper_sixtop.target, &ieee802514_header->src)) {
            if (ieee802514_header->frameType == IEEE154_TYPE_DATA) {
                whisper_log("Received a response packet from 6p target.\n");
                return ieee802514_header->ieListPresent;
            }
        }
    }
    return FALSE;
}

void whisperSixtopProcessIE(OpenQueueEntry_t* pkt) {
    uint16_t temp;
    sixtop_processIEs(pkt, &temp); // Process IE, this function should call whisperSixtopResonseReceive
}

bool whisperSixtopParse(const uint8_t* command) {
    open_addr_t temp;
    uint8_t slotOffset, channel;

    if(command[1] != 0x02) {
        whisper_log("Not a sixtop command, command parsing aborted\n");
        return FALSE;
    }

    // Command is a sixtop command, parse it
    whisper_vars.whisper_sixtop.request_type = command[2];

    // Target ID should be located at bytes 3-4, we use my_addr to construct the target address
    whisper_vars.my_addr.addr_128b[14] = command[3];
    whisper_vars.my_addr.addr_128b[15] = command[4];
    packetfunctions_ip128bToMac64b(&whisper_vars.my_addr,&temp,&whisper_vars.whisper_sixtop.target);

    // Source ID should be located at bytes 5-6, we use my_addr to construct the source address
    whisper_vars.my_addr.addr_128b[14] = command[5];
    whisper_vars.my_addr.addr_128b[15] = command[6];
    packetfunctions_ip128bToMac64b(&whisper_vars.my_addr,&temp,&whisper_vars.whisper_sixtop.source);

    // CellType
    whisper_vars.whisper_sixtop.cellType = command[7];
    switch(command[7]) {
        case CELLOPTIONS_TX:
        case CELLOPTIONS_RX:
            break;
        default:
            whisper_log("Cell Type not valid, command parsing aborted.\n");
            return FALSE;
    }

    // No need for extra parsing
    if(whisper_vars.whisper_sixtop.request_type == IANA_6TOP_CMD_CLEAR) {
        whisper_vars.whisper_sixtop.seqNum = 0; // does not matter
        return TRUE;
    }

    // SeqNum
    whisper_vars.whisper_sixtop.seqNum = command[8];
    if(whisper_vars.whisper_sixtop.seqNum == 255) {
        // SeqNum 255 ==> use internal stored seqNum
        whisper_vars.whisper_sixtop.seqNum = neighbors_getSequenceNumber(&whisper_vars.whisper_sixtop.target);
    }

    switch (whisper_vars.whisper_sixtop.request_type) {
        case IANA_6TOP_CMD_LIST:
        case IANA_6TOP_CMD_COUNT:
            // No cell creation needed
            return TRUE;
        default:
            break;
    }

    switch(command[9]) {
        case 0x00:
            whisper_vars.whisper_sixtop.cell.isUsed = FALSE;
        case 0x01:
            // Cell is defined in the command
            slotOffset = (uint16_t) (command[10] << 8) | (uint16_t ) command[11];
            channel = (uint16_t) (command[12] << 8) | (uint16_t ) command[13];
            whisper_vars.whisper_sixtop.cell.slotoffset       = slotOffset;
            whisper_vars.whisper_sixtop.cell.channeloffset    = channel;
            whisper_vars.whisper_sixtop.cell.isUsed           = TRUE;
            whisper_log("Adding cell with offset: %d and channel: %d\n", slotOffset, channel);
            break;
        case 0x02:
            // Choose a random cell
            whisper_log("Adding random cell.\n");
            slotOffset = openrandom_get16b() % schedule_getFrameLength();
            if(schedule_isSlotOffsetAvailable(slotOffset)==TRUE){
                whisper_vars.whisper_sixtop.cell.slotoffset       = slotOffset;
                whisper_vars.whisper_sixtop.cell.channeloffset    = openrandom_get16b() & 0x0F;
                whisper_vars.whisper_sixtop.cell.isUsed           = TRUE;
            }
            break;
        default:
            whisper_log("Invalid cell definition, command parsing aborted.\n");
            return FALSE;
    }

    return TRUE; // command parsed successfully
}

void whisperExecuteSixtop() {
    // Whisper_vars sixtop should be set correctly to run this command

    owerror_t request = E_FAIL;
    switch(whisper_vars.whisper_sixtop.request_type) {
        case IANA_6TOP_CMD_ADD:
            if(idmanager_isMyAddress(&whisper_vars.whisper_sixtop.target) ||
               idmanager_isMyAddress(&whisper_vars.whisper_sixtop.source)) {
                whisper_log("Adding cells with whisper node is not allowed at the moment.\n");
                return;
            }

            whisper_vars.whisper_ack.acceptACKaddr.type = ADDR_64B;
            // Set ACK receiving ACK adderss to dio target
            memcpy(&whisper_vars.whisper_ack.acceptACKaddr,&whisper_vars.whisper_sixtop.target, sizeof(open_addr_t));

            if(whisperSixtopAddAutonomousCell()) {
                whisper_log("Automonous cell to target successfully added.\n");
                whisper_vars.whisper_sixtop.waiting_for_response = TRUE;

                // call sixtop
                request = sixtop_request_Whisper(
                        IANA_6TOP_CMD_ADD,                   // code
                        &whisper_vars.whisper_sixtop.target, // neighbor
                        whisper_vars.whisper_sixtop.cellType,// cellOptions
                        &whisper_vars.whisper_sixtop.cell,    // cell
                        msf_getsfid(),                       // sfid
                        0,                                   // list command offset (not used)
                        0,                                   // list command maximum celllist (not used)
                        whisper_vars.whisper_sixtop.seqNum
                );
            } else {
                whisper_log("Failed to add cell to target. 6P response would not be received.\n");
            }
            break;
        case IANA_6TOP_CMD_DELETE:
            break;
        case IANA_6TOP_CMD_LIST:
            break;
        case IANA_6TOP_CMD_CLEAR:
            if(idmanager_isMyAddress(&whisper_vars.whisper_sixtop.target) ||
               idmanager_isMyAddress(&whisper_vars.whisper_sixtop.source)) {
                whisper_log("Clearing cells with whisper node is not allowed at the moment.\n");
                return;
            }

            whisper_vars.whisper_ack.acceptACKaddr.type = ADDR_64B;
            // Set ACK receiving ACK adderss to dio target
            memcpy(&whisper_vars.whisper_ack.acceptACKaddr,&whisper_vars.whisper_sixtop.target, sizeof(open_addr_t));

            if(whisperSixtopAddAutonomousCell()) {
                whisper_log("Automonous cell to target successfully added.\n");
                whisper_vars.whisper_sixtop.waiting_for_response = TRUE;

                // call sixtop
                request = sixtop_request_Whisper(
                        IANA_6TOP_CMD_CLEAR,                   // code
                        &whisper_vars.whisper_sixtop.target, // neighbor
                        whisper_vars.whisper_sixtop.cellType, // cellOptions
                        0,    // cell
                        msf_getsfid(),                       // sfid
                        0,                                   // list command offset (not used)
                        0,                                   // list command maximum celllist (not used)
                        whisper_vars.whisper_sixtop.seqNum
                );
            } else {
                whisper_log("Failed to add cell to target. 6P response would not be received.\n");
            }
            break;
        default:
            whisper_log("Unrecognized 6P command, abort.\n");
            return;
    }

    if(request == E_SUCCESS) {
        whisper_log("Sixtop request sent.\n");
        whisper_vars.whisper_ack.acceptACKs = TRUE;

        // Start timer to clean sixtop setting, in case no response is received
        whisper_vars.state = WHISPER_STATE_SIXTOP;
        opentimers_scheduleIn(
                whisper_vars.oneshotTimer,
                (uint32_t) 10000, // wait 5 seconds
                TIME_MS,
                TIMER_ONESHOT,
                whisperSixtopClearCb
        );
    }
    else whisper_log("Sixtop request not sent. Something went wrong.\n");
}

void whisperSixtopClearCb(opentimers_id_t id) {
    whisper_log("Clearing sixtop settings.\n");
    if(whisper_vars.whisper_sixtop.waiting_for_response) whisper_log("[SIXTOP] Waiting for response, timed out.\n");
    if(whisper_vars.whisper_ack.acceptACKs) whisper_log("[SIXTOP] Waiting for ACK, timed out.\n");

    whisperSixtopRemoveAutonomousCell();
    whisper_vars.whisper_sixtop.request_type = 0x00;
    whisper_vars.whisper_sixtop.seqNum = 0;
    whisper_vars.whisper_sixtop.waiting_for_response = FALSE;

    whisper_vars.state = WHISPER_STATE_IDLE;
}


// ============================================================================================
// Logging (should be removed for openmote build, no printf)
void whisper_log(char* msg, ...) {
	open_addr_t* my_id = idmanager_getMyID(ADDR_16B);
	printf("[%d] Whisper: \t", my_id->addr_64b[1]);

	char buf[100];
	va_list v1;
	va_start(v1, msg);
	vsnprintf(buf, sizeof(buf), msg, v1);
	va_end(v1);

	printf(buf);
}

void whisper_print_address(open_addr_t* addr) {
	uint8_t length = 4;
	uint8_t* start_addr = addr->addr_16b;
	switch (addr->type) {
		case ADDR_64B:
			length = 8;
			start_addr = addr->addr_64b;
			break;
		case ADDR_128B:
			length = 16;
			start_addr = addr->addr_128b;
			break;
		default: break;
	}

	for(uint8_t i = 0; i < length; i++) {
		printf("%02x", start_addr[i]);
		if(i < length - 1) printf(":");
	}
	printf("\n");
}




