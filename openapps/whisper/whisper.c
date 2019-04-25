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
    whisper_vars.timerId = opentimers_create(TIMER_GENERAL_PURPOSE, TASKPRIO_RPL);
    opentimers_scheduleIn(
            whisper_vars.timerId,
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
    // Serial communication (Rx) can be used to privide out band communication between node and controller
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

            open_addr_t my_addr;
            my_addr.type = ADDR_128B;
            memcpy(&my_addr.addr_128b, 		idmanager_getMyID(ADDR_PREFIX)->prefix, 8);
            memcpy(&my_addr.addr_128b[8], 	idmanager_getMyID(ADDR_64B)->addr_64b, 8);

            switch(msg->payload[1]) {
                case 0x01:
                    whisper_log("Whisper fake dio command (remote)\n");
                    whisperDioCommand(msg->payload, &my_addr);
                    break;
            	case 0x02:
            		whisper_log("Whisper 6P command (remote).\n");
                    whisperSixTopCommand(msg->payload, &my_addr);
					break;
                default:
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

void whisperDioCommand(const uint8_t* command, open_addr_t* my_addr) {
    open_addr_t temp;

    // Target
    my_addr->addr_128b[14] = command[2];
    my_addr->addr_128b[15] = command[3];
    memcpy(&whisper_vars.whisper_dio.target, &my_addr, sizeof(open_addr_t));

    // Parent
    my_addr->addr_128b[14] = command[4];
    my_addr->addr_128b[15] = command[5];
    memcpy(&whisper_vars.whisper_dio.parent, &my_addr, sizeof(open_addr_t));

    // Next Hop == target
    memcpy(&whisper_vars.whisper_dio.nextHop, &whisper_vars.whisper_dio.target, sizeof(open_addr_t));

    whisper_vars.whisper_dio.rank = (uint16_t) ((uint16_t) command[8] << 8) | (uint16_t ) command[9];

    whisper_vars.whisper_ack.acceptACKaddr.type = ADDR_64B;
    // Set ACK receiving ACK adderss to dio target
    packetfunctions_ip128bToMac64b(&whisper_vars.whisper_dio.target,&temp,&whisper_vars.whisper_ack.acceptACKaddr);

    whisper_log("Sending fake DIO with rank %d.\n", whisper_vars.whisper_dio.rank);

    uint8_t result = send_WhisperDIO();

    // If DIO is send successfully (to lower layers) activate ACK sniffing if the parent is not myself
    if(result == E_SUCCESS && (idmanager_isMyAddress(&whisper_vars.whisper_dio.parent) == FALSE)) {
        whisper_vars.whisper_ack.acceptACKs = TRUE;
    }

    /*uint8_t data[2];
    data[0] = 0x01; // indicate fake dio send from root
    data[1] = result;
    //openserial_sendWhisper(data, 2);*/
}

// ---------------------- Whisper Sixtop --------------------------

bool whisperAddSixtopCellSchedule() {
    uint8_t id_lsb = whisper_vars.whisper_sixtop.target.addr_64b[7];
    uint8_t id_msb = whisper_vars.whisper_sixtop.target.addr_64b[6];
    uint16_t slotOffset = msf_hashFunction_getSlotoffset((uint16_t) (256 * id_msb + id_lsb));

    if(schedule_isSlotOffsetAvailable(slotOffset)) {
        owerror_t outcome = schedule_addActiveSlot(
                slotOffset,                                                             // slot offset
                CELLTYPE_RX,                                                            // type of slot
                FALSE,                                                                  // shared?
                msf_hashFunction_getChanneloffset((uint16_t) (256 * id_msb + id_lsb)),  // channel offset
                &whisper_vars.whisper_sixtop.target                                     // neighbor
        );
        return (uint8_t) (outcome == E_SUCCESS);
    }

    return icmpv6rpl_isPreferredParent(&whisper_vars.whisper_sixtop.target);
};

void whisperCheckSixtopResponseAddr(open_addr_t* addr) {
    if(whisper_vars.whisper_sixtop.waiting_for_response) {
        if (packetfunctions_sameAddress(addr, &whisper_vars.whisper_sixtop.target)) {
            if (icmpv6rpl_isPreferredParent(&whisper_vars.whisper_sixtop.target) == FALSE) {
                uint8_t id_lsb = whisper_vars.whisper_sixtop.target.addr_64b[7];
                uint8_t id_msb = whisper_vars.whisper_sixtop.target.addr_64b[6];
                schedule_removeActiveSlot(msf_hashFunction_getSlotoffset((uint16_t) (256 * id_msb + id_lsb)),
                                          &whisper_vars.whisper_sixtop.target);
                whisper_log("Sixtop response received, removing autonomous cell of target.\n");
            } else
                whisper_log("Not removing autonomous cell of target (parent).\n");
        }
    }
}

bool whisper_SixTopPacketAccept(ieee802154_header_iht* ieee802514_header) {
    if(packetfunctions_sameAddress(&whisper_vars.whisper_sixtop.target,&ieee802514_header->src)) {
        if(ieee802514_header->frameType == IEEE154_TYPE_DATA) {
            whisper_log("Received a response packet from 6p target.\n");
            return ieee802514_header->ieListPresent;
        }
    }
    return FALSE;
}

void whisperSixTopCommand(const uint8_t* command, open_addr_t* my_addr) {
    open_addr_t temp;
    cellInfo_ht cellList[1]; // only 1 cell with each command
    uint16_t slotOffset, channel;
    uint8_t cellOptions;

    owerror_t request = E_FAIL;
    switch(command[2]) {
        case IANA_6TOP_CMD_ADD:
            // Target
            my_addr->addr_128b[14] = command[3];
            my_addr->addr_128b[15] = command[4];
            packetfunctions_ip128bToMac64b(my_addr,&temp,&whisper_vars.whisper_sixtop.target);

            // Source
            my_addr->addr_128b[14] = command[5];
            my_addr->addr_128b[15] = command[6];
            whisper_vars.whisper_sixtop.source.type = ADDR_64B;
            packetfunctions_ip128bToMac64b(my_addr,&temp,&whisper_vars.whisper_sixtop.source);

            whisper_vars.whisper_ack.acceptACKaddr.type = ADDR_64B;
            // Set ACK receiving ACK adderss to dio target
            memcpy(&whisper_vars.whisper_ack.acceptACKaddr,&whisper_vars.whisper_sixtop.target, sizeof(open_addr_t));

            switch(command[7]) {
                case 0x01:
                    cellOptions = CELLOPTIONS_TX;
                    break;
                case 0x02:
                    cellOptions = CELLOPTIONS_RX;
                    break;
                default:
                    whisper_log("Not a valid celloption type, command aborted.\n");
                    return;
            }

            // Cell creation
            switch(command[8]) {
                case 0x01:
                    // Cell is defined in the command
                    slotOffset = (uint16_t) (command[9] << 8) | (uint16_t ) command[10];
                    channel = (uint16_t) (command[11] << 8) | (uint16_t ) command[12];
                    if(schedule_isSlotOffsetAvailable(slotOffset)==TRUE){
                        cellList[0].slotoffset       = slotOffset;
                        cellList[0].channeloffset    = channel;
                        cellList[0].isUsed           = TRUE;
                        whisper_log("Adding cell with offset: %d and channel: %d\n", slotOffset, channel);
                        break;
                    }
                    whisper_log("Defined cell not available, command aborted.\n");
                    return;
                case 0x02:
                    // Choose a random cell
                    whisper_log("Adding random cell.\n");
                    msf_candidateAddCellList(cellList, 1);
                    break;
                default:
                    whisper_log("Invalid cell definition, command aborted.\n");
                    return;
            }

            if(whisperAddSixtopCellSchedule()) {
                whisper_log("Automonous cell to target successfully added.\n");
                whisper_vars.whisper_sixtop.waiting_for_response = TRUE;

                // call sixtop
                request = sixtop_request_Whisper(
                        IANA_6TOP_CMD_ADD,                   // code
                        &whisper_vars.whisper_sixtop.target, // neighbor
                        1,                                   // number cells
                        cellOptions,                         // cellOptions
                        cellList,                            // celllist to add
                        NULL,                                // celllist to delete (not used)
                        msf_getsfid(),                       // sfid
                        0,                                   // list command offset (not used)
                        0                                    // list command maximum celllist (not used)
                );
            }
            break;
        case IANA_6TOP_CMD_DELETE:
            // Target
            my_addr->addr_128b[14] = command[3];
            my_addr->addr_128b[15] = command[4];
            packetfunctions_ip128bToMac64b(my_addr,&temp,&whisper_vars.whisper_sixtop.target);

            // Source
            my_addr->addr_128b[14] = command[5];
            my_addr->addr_128b[15] = command[6];
            whisper_vars.whisper_sixtop.source.type = ADDR_64B;
            packetfunctions_ip128bToMac64b(my_addr,&temp,&whisper_vars.whisper_sixtop.source);

            whisper_vars.whisper_ack.acceptACKaddr.type = ADDR_64B;
            // Set ACK receiving ACK adderss to dio target
            memcpy(&whisper_vars.whisper_ack.acceptACKaddr,&whisper_vars.whisper_sixtop.target, sizeof(open_addr_t));


            switch(command[7]) {
                case 0x01:
                    cellOptions = CELLOPTIONS_TX;
                    break;
                case 0x02:
                    cellOptions = CELLOPTIONS_RX;
                    break;
                default:
                    whisper_log("Not a valid celloption type, command aborted.\n");
                    return;
            }

            // Cell creation
            switch(command[8]) {
                case 0x01:
                    // Cell is defined in the command
                    slotOffset = (uint16_t) (command[9] << 8) | (uint16_t ) command[10];
                    channel = (uint16_t) (command[11] << 8) | (uint16_t ) command[12];
                    if(schedule_isSlotOffsetAvailable(slotOffset)==FALSE){
                        cellList[0].slotoffset       = slotOffset;
                        cellList[0].channeloffset    = channel;
                        cellList[0].isUsed           = TRUE;
                        whisper_log("Removing cell with offset: %d and channel: %d\n", slotOffset, channel);
                        break;
                    }
                    whisper_log("Defined cell not used, command aborted.\n");
                    return;
                case 0x02:
                    // Choose a random cell
                    whisper_log("Removing random cells scheduled by whisper is not possible atm.\n");
                    return;
                default:
                    whisper_log("Invalid cell definition, command aborted.\n");
                    return;
            }

            if(whisperAddSixtopCellSchedule()) {
                whisper_log("Automonous cell to target successfully added.\n");
                whisper_vars.whisper_sixtop.waiting_for_response = TRUE;

                // call sixtop
                request = sixtop_request_Whisper(
                        IANA_6TOP_CMD_DELETE,                   // code
                        &whisper_vars.whisper_sixtop.target, // neighbor
                        1,                                   // number cells
                        cellOptions,                         // cellOptions
                        NULL,                            // celllist to add (not used)
                        cellList,                                // celllist to delete
                        msf_getsfid(),                       // sfid
                        0,                                   // list command offset (not used)
                        0                                    // list command maximum celllist (not used)
                );
            }
            break;
        case IANA_6TOP_CMD_LIST:
            // Target
            my_addr->addr_128b[14] = command[3];
            my_addr->addr_128b[15] = command[4];
            packetfunctions_ip128bToMac64b(my_addr,&temp,&whisper_vars.whisper_sixtop.target);

            // Source
            my_addr->addr_128b[14] = command[5];
            my_addr->addr_128b[15] = command[6];
            whisper_vars.whisper_sixtop.source.type = ADDR_64B;
            packetfunctions_ip128bToMac64b(my_addr,&temp,&whisper_vars.whisper_sixtop.source);

            whisper_vars.whisper_ack.acceptACKaddr.type = ADDR_64B;
            // Set ACK receiving ACK adderss to dio target
            memcpy(&whisper_vars.whisper_ack.acceptACKaddr,&whisper_vars.whisper_sixtop.target, sizeof(open_addr_t));

            if(whisperAddSixtopCellSchedule()) {
                whisper_log("Automonous cell to target successfully added.\n");
                whisper_vars.whisper_sixtop.waiting_for_response = TRUE;

                // call sixtop
                request = sixtop_request_Whisper(
                        IANA_6TOP_CMD_LIST,                  // code
                        &whisper_vars.whisper_sixtop.target,// neighbor
                        0,                                  // number cells
                        0,                     // cellOptions
                        NULL,                       // celllist to add
                        NULL,                               // celllist to delete (not used)
                        msf_getsfid(),                      // sfid
                        0,                                  // list command offset (not used)
                        0                                   // list command maximum celllist (not used)
                );
            }
            break;
        case IANA_6TOP_CMD_CLEAR:
            // Target
            my_addr->addr_128b[14] = command[3];
            my_addr->addr_128b[15] = command[4];
            packetfunctions_ip128bToMac64b(my_addr,&temp,&whisper_vars.whisper_sixtop.target);

            // Source
            my_addr->addr_128b[14] = command[5];
            my_addr->addr_128b[15] = command[6];
            whisper_vars.whisper_sixtop.source.type = ADDR_64B;
            packetfunctions_ip128bToMac64b(my_addr,&temp,&whisper_vars.whisper_sixtop.source);

            whisper_vars.whisper_ack.acceptACKaddr.type = ADDR_64B;
            // Set ACK receiving ACK adderss to dio target
            memcpy(&whisper_vars.whisper_ack.acceptACKaddr,&whisper_vars.whisper_sixtop.target, sizeof(open_addr_t));

            if(whisperAddSixtopCellSchedule()) {
                whisper_log("Automonous cell to target successfully added.\n");
                whisper_vars.whisper_sixtop.waiting_for_response = TRUE;

                // call sixtop
                request = sixtop_request_Whisper(
                        IANA_6TOP_CMD_CLEAR,                  // code
                        &whisper_vars.whisper_sixtop.target, // neighbor
                        0,                                   // number cells
                        0,                                   // cellOptions
                        NULL,                                // celllist to add
                        NULL,                                // celllist to delete (not used)
                        msf_getsfid(),                       // sfid
                        0,                                   // list command offset (not used)
                        0                                    // list command maximum celllist (not used)
                );
            }
            break;
        default:
            whisper_log("Unrecognized 6P command, abort.\n");
            return;
    }

    if(request == E_SUCCESS) {
        whisper_log("Sixtop request sent.\n");
        whisper_vars.whisper_ack.acceptACKs = TRUE;
    }
    else whisper_log("Sixtop request not sent.\n");
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




