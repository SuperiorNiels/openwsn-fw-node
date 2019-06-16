/**
\brief Whisper module
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
void whisper_init(void) {
	whisper_log("Initializing whisper node.\n");

	whisper_vars.state = WHISPER_STATE_IDLE;
    whisper_vars.whisper_propagating_dio.active = FALSE;
	whisper_vars.whisper_propagating_dio.period = 25; // depended on the DIO period in RPL settings (10s normal)
	whisper_vars.whisper_propagating_dio.counter = 0;

    // eui_addr = is to store the eui, so we can easily construct addresses by just setting the correct id
    whisper_vars.eui_addr.type = ADDR_128B;
    memcpy(&whisper_vars.eui_addr.addr_128b, 		idmanager_getMyID(ADDR_PREFIX)->prefix, 8);
    memcpy(&whisper_vars.eui_addr.addr_128b[8], 	    idmanager_getMyID(ADDR_64B)->addr_64b, 8);
    whisper_vars.eui_addr.addr_128b[0] = 0xbb;
    whisper_vars.eui_addr.addr_128b[1] = 0xbb; // necessary to make dios work
    memcpy(&whisper_vars.my_addr, &whisper_vars.eui_addr, sizeof(open_addr_t));

    // set controller address
    whisper_vars.controller_addr.type = ADDR_128B;
    memcpy(&whisper_vars.controller_addr.addr_128b[0], &ipAddr_ringmaster, sizeof(open_addr_t));

	// prepare the resource descriptor for the /w path
	whisper_vars.desc.path0len             = sizeof(whisper_path0)-1;
	whisper_vars.desc.path0val             = (uint8_t*)(&whisper_path0);
	whisper_vars.desc.path1len             = 0;
	whisper_vars.desc.path1val             = NULL;
	whisper_vars.desc.componentID          = COMPONENT_WHISPER;
	whisper_vars.desc.discoverable         = TRUE;
	whisper_vars.desc.callbackRx           = &whisper_receive;
	whisper_vars.desc.callbackSendDone     = &whisper_sendDone;

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

uint8_t whisper_getState(void) {
	return whisper_vars.state;
}

void whisper_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
    if(error == E_SUCCESS) {
        if (whisper_vars.state == WHISPER_STATE_WAIT_COAP) {
            // Coap PUT received and coap message is correctly handled
            whisperExecuteCommand();
        }
    }

    openqueue_freePacketBuffer(msg);
}

void whisperExecuteCommand() {
    switch (whisper_vars.payloadBuffer[1]) {
        case WHISPER_COMMAND_DIO:
            whisper_log("Whisper fake dio command (remote)\n");
            whisperDioCommand(whisper_vars.payloadBuffer);
            break;
        case WHISPER_COMMAND_SIXTOP:
            whisper_log("Whisper 6P command (remote).\n");
            if (whisperSixtopParse(whisper_vars.payloadBuffer)) whisperExecuteSixtop();
            else whisper_log("Parsing command failed.\n");
            break;
        case WHISPER_COMMAND_LINK_INFO:
            whisper_log("Whisper 6P stored link information: \n");
            // Cancel the timer
            opentimers_cancel(whisper_vars.oneshotTimer);
            whisper_vars.state = WHISPER_STATE_IDLE;
            for (uint8_t i = 0; i < SIXTOP_MAX_LINK_SNIFFING; i++) {
                if (whisper_vars.neighbors.sixtop[i].active == TRUE) {
                    whisper_log("Link %d -> %d, seqNum: %d\n",
                                whisper_vars.neighbors.sixtop[i].srcId[1],
                                whisper_vars.neighbors.sixtop[i].destId[1],
                                whisper_vars.neighbors.sixtop[i].seqNum);
                }
            }
            break;
        case WHISPER_COMMAND_NEIGHBOURS:
            whisper_log("Whisper get neighbours command.\n");
            // Cancel timer
            opentimers_cancel(whisper_vars.oneshotTimer);
            whisper_vars.state = WHISPER_STATE_IDLE;
            uint8_t payload[60]; // 2 bytes per possible neigbour
            payload[0] = 0x04; // indicate response to get neighbours command
            payload[1] = idmanager_getMyID(ADDR_64B)->addr_64b[6];
            payload[2] = idmanager_getMyID(ADDR_64B)->addr_64b[7];
            uint8_t length = getNeighborsList(&payload[3]);
            sendCoapResponseToController(payload, length + 3);
            break;
        case WHISPER_COMMNAD_TOGGLE_DIO:
            whisper_togglePropagatingDios();
            whisper_log("Toggle send propagating dios: %d.\n", whisper_vars.whisper_propagating_dio.active);
            break;
        case WHISPER_COMMNAD_SET_RANKTOSEND:
            whisper_setRankForNeighbour(whisper_vars.payloadBuffer);
            break;                
        default:
            whisper_log("Unknown command %d \n", whisper_vars.payloadBuffer[1]);
            break;
    }
}

void whisperClearStateCb(opentimers_id_t id) {
    if (whisper_vars.whisper_ack.acceptACKs) whisper_log("Waiting for ACK, timed out.\n");

    switch(whisper_vars.state) {
        case WHISPER_STATE_SIXTOP:
            whisper_log("Clearing sixtop settings.\n");
            if (whisper_vars.whisper_sixtop.waiting_for_response) whisper_log("Waiting for response, timed out.\n");
            whisper_vars.whisper_sixtop.waiting_for_response = FALSE;
            break;
        case WHISPER_STATE_DIO:
            whisper_log("Clearing whisper dio settings\n");
            if(whisper_vars.whisper_ack.acceptACKs ) {
                // No ACK received
                uint8_t result[2] = {WHISPER_COMMAND_DIO, E_FAIL};
                sendCoapResponseToController(result, 2);
                whisper_vars.state = WHISPER_STATE_IDLE;
            }
            break;
        case WHISPER_STATE_WAIT_COAP:
            whisper_log("Waiting for CoAP, command aborted.\n");
            break;
        default:
            break;
    }

    // Always go back to idle
    whisper_vars.state = WHISPER_STATE_IDLE;
    whisper_vars.whisper_ack.acceptACKs = FALSE;
    whisper_vars.whisper_sixtop.waiting_for_response = FALSE;
    openqueue_freeWhisperPackets(); // free all packets created by whisper (not response packets)
}

void whisper_timer_cb(opentimers_id_t id) {
    // Check if the neigbour list has changed, add the autonomous cell to each neigbour
    neighbors_updateAutonomousCells();
    leds_debug_toggle();
}

void whisper_task_remote(uint8_t* buf, uint8_t bufLen) {
    // Serial communication (Rx) can be used to provide out band communication between node and controller

    // Dont's do anything new when not, idle
    if(whisper_vars.state != WHISPER_STATE_IDLE)
        return;

    whisper_log("Received Serial command.\n");

    if(bufLen <= 30) {
        memcpy(whisper_vars.payloadBuffer, buf, bufLen + 1);
        whisper_vars.state = WHISPER_STATE_WAIT_COAP;
        // Run sendDone to start the command FIXME: should be done differently in the future
        whisperExecuteCommand();
        // Start timer to clean up (in any event)
        opentimers_scheduleIn(
                whisper_vars.oneshotTimer,
                (uint32_t) 10000, // wait 10 seconds
                TIME_MS,
                TIMER_ONESHOT,
                whisperClearStateCb
        );
    } else {
        whisper_log("Message payload to long.\n");
    }
}

owerror_t whisper_receive(OpenQueueEntry_t* msg,
						coap_header_iht*  coap_header,
						coap_option_iht*  coap_incomingOptions,
						coap_option_iht*  coap_outgoingOptions,
						uint8_t*          coap_outgoingOptionsLen)
{
	owerror_t outcome;

    // Dont's do anything new when not, idle
    if(whisper_vars.state != WHISPER_STATE_IDLE) {
        // reset packet payload
        msg->payload                     = &(msg->packet[127]);
        msg->length                      = 0;
        coap_header->Code                = COAP_CODE_RESP_CHANGED;
        return E_FAIL;
    }

	switch (coap_header->Code) {
        case COAP_CODE_REQ_PUT:
            whisper_log("Received CoAP PUT.\n");

            if(msg->length <= 30) {
                memcpy(whisper_vars.payloadBuffer, msg->payload, msg->length);
                whisper_vars.state = WHISPER_STATE_WAIT_COAP;
                // Start timer to clean up (in any event)
                opentimers_scheduleIn(
                        whisper_vars.oneshotTimer,
                        (uint32_t) 10000, // wait 10 seconds
                        TIME_MS,
                        TIMER_ONESHOT,
                        whisperClearStateCb
                );
            } else {
                whisper_log("Message payload to long.\n");
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

open_addr_t* getWhisperDIOtarget(void) {
    return &whisper_vars.whisper_dio.target;
}

open_addr_t* getWhisperDIOparent(void) {
    return &whisper_vars.whisper_dio.parent;
}

open_addr_t* getWhisperDIOnextHop(void) {
    return &whisper_vars.whisper_dio.nextHop;
}

dagrank_t getWhisperDIOrank(void) {
    return whisper_vars.whisper_dio.rank;
}

open_addr_t* getWhisperSixtopSource(void) {
    return &whisper_vars.whisper_sixtop.source;
}

bool whisperACKreceive(ieee802154_header_iht* ieee802154_header) {
    if(whisper_vars.whisper_ack.acceptACKs == TRUE) {
        if(packetfunctions_sameAddress(&ieee802154_header->src, &whisper_vars.whisper_ack.ACKsrc) &&
            packetfunctions_sameAddress(&ieee802154_header->dest, &whisper_vars.whisper_ack.ACKdest)) {
            whisper_log("ACK received.\n");
            whisper_vars.whisper_ack.acceptACKs = FALSE;

            if(whisper_vars.state == WHISPER_STATE_DIO) {
                uint8_t result[2] = {WHISPER_COMMAND_DIO, E_SUCCESS};
                sendCoapResponseToController(result, 2);
                whisper_vars.state = WHISPER_STATE_IDLE;
            }

            return TRUE;
        }
    }
    return FALSE;
}

void whisperDioCommand(const uint8_t* command) {
    open_addr_t temp;
    whisper_vars.state = WHISPER_STATE_DIO;

    // Target
    whisper_vars.eui_addr.addr_128b[14] = command[2];
    whisper_vars.eui_addr.addr_128b[15] = command[3];
    memcpy(&whisper_vars.whisper_dio.target, &whisper_vars.eui_addr, sizeof(open_addr_t));

    // Parent
    whisper_vars.eui_addr.addr_128b[14] = command[4];
    whisper_vars.eui_addr.addr_128b[15] = command[5];
    memcpy(&whisper_vars.whisper_dio.parent, &whisper_vars.eui_addr, sizeof(open_addr_t));

    // Next Hop == target
    memcpy(&whisper_vars.whisper_dio.nextHop, &whisper_vars.whisper_dio.target, sizeof(open_addr_t));

    whisper_vars.whisper_dio.rank = (uint16_t) (command[8] << 8) | (uint16_t ) command[9];

    whisper_log("Sending fake DIO with rank %d.\n", whisper_vars.whisper_dio.rank);

    uint8_t result = send_WhisperDIO();

    // If DIO is send successfully (to lower layers) activate ACK sniffing if the parent is not myself
    if(result == E_SUCCESS && (idmanager_isMyAddress(&whisper_vars.whisper_dio.parent) == FALSE)) {
        // Set ACK addresses
        whisper_vars.whisper_ack.ACKsrc.type = ADDR_64B;
        packetfunctions_ip128bToMac64b(&whisper_vars.whisper_dio.target,&temp,&whisper_vars.whisper_ack.ACKsrc);
        whisper_vars.whisper_ack.ACKdest.type = ADDR_64B;
        packetfunctions_ip128bToMac64b(&whisper_vars.whisper_dio.parent,&temp,&whisper_vars.whisper_ack.ACKdest);
        whisper_vars.whisper_ack.acceptACKs = TRUE;
    }
}

// ---------------------- Whisper propagating DIOs --------------------------

void whisper_sendPropagatingDios() {
    uint8_t i;
    neighborRow_t row;

    if(!idmanager_getIsDAGroot()) return;

    if(whisper_vars.whisper_propagating_dio.counter < whisper_vars.whisper_propagating_dio.period) {
        whisper_vars.whisper_propagating_dio.counter++;
        return;
    }

    whisper_log("Send propagating dios.\n");
    whisper_vars.whisper_propagating_dio.counter = 0;

    // Loop through neighbors, if the sequence number != 0 the neighbour is a connected child node so we send a dio
    for(i = 0; i < MAXNUMNEIGHBORS; i++) {
        row = *neighbors_getNeighborRow(i);

        if(row.stableNeighbor && row.used) { // && row.sequenceNumber != 0 (when the dios only need to be sent to the direct (connected) neighbours)
            // Send dio
            // Copy neighbor address in eui_addr
            memcpy(&whisper_vars.eui_addr.addr_128b[8], row.addr_64b.addr_64b, 8);

            // Target
            memcpy(&whisper_vars.whisper_dio.target, &whisper_vars.eui_addr, sizeof(open_addr_t));

            // Parent (me)
            memcpy(&whisper_vars.whisper_dio.parent, &whisper_vars.my_addr, sizeof(open_addr_t));

            // Next Hop
            memcpy(&whisper_vars.whisper_dio.nextHop, &whisper_vars.eui_addr, sizeof(open_addr_t));

            whisper_vars.whisper_dio.rank = (row.rankToSend == DEFAULTDAGRANK) ?  icmpv6rpl_getMyDAGrank() : row.rankToSend;

            uint8_t result = send_WhisperDIO();

            if(result == E_SUCCESS) {
                whisper_log("Dio sent to: "); whisper_print_address(&row.addr_64b);
            }
        }
    }
}

bool whisper_getSendNormalDio() {
    if(idmanager_getIsDAGroot() == FALSE) return FALSE; // never send dio when not root
    return (bool) !whisper_vars.whisper_propagating_dio.active;
}

void whisper_setDIOPeriod(uint16_t period) {
    // TODO
}

void whisper_togglePropagatingDios() {
    if(idmanager_getIsDAGroot()) {
        whisper_vars.whisper_propagating_dio.active = (bool) !whisper_vars.whisper_propagating_dio.active;
        whisper_log("Dio sent rate: %d\n", whisper_vars.whisper_propagating_dio.period);
    } else {
        whisper_log("Propagating dios not possible (not root).\n");
    }
}

void whisper_setRankForNeighbour(uint8_t* command) {
    open_addr_t neighbour, temp;
    dagrank_t rank;

    // Target
    whisper_vars.eui_addr.addr_128b[14] = command[2];
    whisper_vars.eui_addr.addr_128b[15] = command[3];
    memcpy(&whisper_vars.whisper_dio.target, &whisper_vars.eui_addr, sizeof(open_addr_t));

    packetfunctions_ip128bToMac64b(&whisper_vars.eui_addr, &temp, &neighbour);

    rank = (uint16_t) (command[4] << 8) | (uint16_t) command[5];

    whisper_log("Updating rank to: %d for neighbour: ", rank);
    whisper_print_address(&neighbour);

    neighbors_setRankToSend(&neighbour, rank);
}

// ---------------------- Whisper Sixtop --------------------------

void whisperSixtopResonseReceive(OpenQueueEntry_t* pkt, uint8_t code, uint8_t ptr, uint8_t length) {
    if(whisper_vars.state != WHISPER_STATE_SIXTOP) {
        whisper_log("Wrong state, abort.\n");
        return;
    }

    uint8_t responeLength;
    uint8_t pktLen = length;
    open_addr_t addr = pkt->l2_nextORpreviousHop;
    uint8_t result[2];

    result[0] = WHISPER_COMMAND_SIXTOP;
    if(whisper_vars.whisper_sixtop.waiting_for_response) {
        if (packetfunctions_sameAddress(&addr, &whisper_vars.whisper_sixtop.target)) {

            // Clean up
            whisper_vars.whisper_sixtop.waiting_for_response = FALSE;
            opentimers_cancel(whisper_vars.oneshotTimer); // cancel the timer

            // Check response
            switch(code) {
                case IANA_6TOP_RC_SUCCESS:
                    whisper_log("Whisper 6P command successful.\n");
                    if(whisper_vars.whisper_sixtop.request_type == IANA_6TOP_CMD_CLEAR) {
                        uint8_t srcID[2] = {whisper_vars.whisper_sixtop.source.addr_64b[6], whisper_vars.whisper_sixtop.source.addr_64b[7]};
                        uint8_t destID[2] = {whisper_vars.whisper_sixtop.target.addr_64b[6], whisper_vars.whisper_sixtop.target.addr_64b[7]};
                        updateSixtopLinkSeqNum(srcID, destID, 0);
                    }
                    result[1] = E_SUCCESS;
                    sendCoapResponseToController(result, 2);
                    break;
                case IANA_6TOP_RC_EOL: // FIXME: for now only process the cells when the response is EOL, this means only the cells in this packet will be sent to the controller
                    whisper_log("Whisper 6P List response received.\n");

                    uint8_t to_controller[CELLLIST_MAX_LEN * sizeof(uint8_t) * 2];
                    memset(to_controller, 0, CELLLIST_MAX_LEN * sizeof(uint8_t) * 2);

                    responeLength = 0;
                    to_controller[responeLength] = WHISPER_COMMAND_SIXTOP;
                    responeLength++;

                    while(pktLen>0) {
                        to_controller[responeLength]        =  *((uint8_t*)(pkt->payload)+ptr);
                        to_controller[responeLength + 1]    = (*((uint8_t*)(pkt->payload)+ptr+1))<<8;
                        to_controller[responeLength + 2]    =  *((uint8_t*)(pkt->payload)+ptr+2);
                        to_controller[responeLength + 3]    = (*((uint8_t*)(pkt->payload)+ptr+3))<<8;
                        ptr    += 4;
                        pktLen -= 4;
                        responeLength += 4;
                    }

                    sendCoapResponseToController(to_controller, responeLength);
                    break;
                case IANA_6TOP_RC_SEQNUM_ERR:
                    whisper_log("Whisper 6P command wrong seqNum.\n");
                default:
                    whisper_log("Whisper 6P failed.\n");
                    result[1] = E_FAIL;   
                    sendCoapResponseToController(result, 2);   

                    if( (whisper_vars.whisper_sixtop.stabilize_link == FALSE || !WHISPER_6P_STABILIZE_LINKS) 
                        || WHISPER_6P_FAST_CLEAR_LINKS) {
                        // We can not fix the link by sending a dummy request to the other node of the link
                        // Clear the link
                        whisper_vars.whisper_sixtop.request_type = IANA_6TOP_CMD_CLEAR;
                        // Target and source are the same, reactivate stabilize link and command parsed correclty
                        whisper_vars.whisper_sixtop.command_parsed = TRUE;
                        whisper_vars.whisper_sixtop.stabilize_link = TRUE;
                        whisper_log("Unfixable error! Fixing link by sending clear.");
                        whisperExecuteSixtop();
                        // Restart the cleanup timer
                        opentimers_scheduleIn(
                                whisper_vars.oneshotTimer,
                                (uint32_t) 10000, // wait 10 seconds
                                TIME_MS,
                                TIMER_ONESHOT,
                                whisperClearStateCb
                        );
                        return;
                    }
                    break;
            }

            if(WHISPER_6P_STABILIZE_LINKS && whisper_vars.whisper_sixtop.stabilize_link) {
                whisperStabilizeLink(code);
                // Restart the cleanup timer
                opentimers_scheduleIn(
                        whisper_vars.oneshotTimer,
                        (uint32_t) 10000, // wait 10 seconds
                        TIME_MS,
                        TIMER_ONESHOT,
                        whisperClearStateCb
                );
            } else {
                // Send response to controller and finish up the command
                whisper_log("6P command finished.\n");
                whisper_vars.state = WHISPER_STATE_IDLE;
            }

        } else {
            whisper_log("Sixtop response received from wrong address.\n");
        }
    } else {
        whisper_log("Not waiting for a 6p response, yet we got here...\n");
    }
}

void whisperStabilizeLink(const uint8_t response_type) {
    open_addr_t temp;

    // If stabilize link is false return
    if(whisper_vars.whisper_sixtop.stabilize_link == FALSE) return;
    else whisper_vars.whisper_sixtop.stabilize_link = FALSE;

    // Always swap target and source
    memcpy(&temp, &whisper_vars.whisper_sixtop.target, sizeof(open_addr_t));
    memcpy(&whisper_vars.whisper_sixtop.target, &whisper_vars.whisper_sixtop.source, sizeof(open_addr_t));
    memcpy(&whisper_vars.whisper_sixtop.source, &temp, sizeof(open_addr_t));

    switch (response_type) {
        case IANA_6TOP_RC_SUCCESS:
            // The previous request was successfull, perform the same request on the other node of the link
            // Previous request type is set in whisper vars (sixtop)
            // TODO: The correct cell should be sent to the other node, the selected  cell(s) can be extracted from the response packet
            // If cell type is RX or TX, reverse it
            if(whisper_vars.whisper_sixtop.cellType == CELLOPTIONS_RX) whisper_vars.whisper_sixtop.cellType = CELLOPTIONS_TX;
            else whisper_vars.whisper_sixtop.cellType = CELLOPTIONS_RX;
            whisper_log("Link stabilize is active, sending request to other node of the link.\n");
            break;
        case IANA_6TOP_RC_EOL:
            // List command successfull, however the seqnum is unstable now, fix it by sending fake list request to other ndoe
        default:
            // Previous request failed, fix link by sending dummy (emtpy 6p list) request to the other node.
            whisper_log("Sending fake (empty) 6p list request to other node to fix seqNum.\n");
            whisper_vars.whisper_sixtop.stabilize_link = FALSE;
            whisper_vars.whisper_sixtop.request_type = IANA_6TOP_CMD_LIST;
            whisper_vars.whisper_sixtop.listMaxCells = 0;
            whisper_vars.whisper_sixtop.listOffset = 0;
            break;
    }

    whisper_vars.whisper_sixtop.command_parsed = TRUE;
    whisperExecuteSixtop();
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

void whisperSixtopProcessIE(void) {
    uint16_t temp;
    OpenQueueEntry_t* pkt = openqueue_sixtopGetReceivedPacket();
    sixtop_processIEs(pkt, &temp);
    // Always drop the packet here, the packet is not meant to be processed by this node (normally)
    openqueue_freePacketBuffer(pkt); // Process IE, this function should call whisperSixtopResonseReceive
}

bool whisperSixtopParse(const uint8_t* command) {
    open_addr_t temp;
    uint8_t slotOffset, channel;

    whisper_vars.whisper_sixtop.command_parsed = FALSE;

    if(command[1] != 0x02) {
        whisper_log("Not a sixtop command, command parsing aborted\n");
        return FALSE;
    }

    whisper_vars.whisper_sixtop.stabilize_link = WHISPER_6P_STABILIZE_LINKS; // TRUE: send 6p request to each node of the link

    // Command is a sixtop command, parse it
    whisper_vars.whisper_sixtop.request_type = command[2];
    switch(whisper_vars.whisper_sixtop.request_type) {
        case IANA_6TOP_CMD_ADD:
        case IANA_6TOP_CMD_DELETE:
        case IANA_6TOP_CMD_CLEAR:
        case IANA_6TOP_CMD_LIST:
            break;
        default:
            whisper_log("Whisper does not support this 6P command. (yet) \n");
            return FALSE;
    }

    // Target ID should be located at bytes 3-4, we use eui_addr to construct the target address
    whisper_vars.eui_addr.addr_128b[14] = command[3];
    whisper_vars.eui_addr.addr_128b[15] = command[4];
    packetfunctions_ip128bToMac64b(&whisper_vars.eui_addr,&temp,&whisper_vars.whisper_sixtop.target);

    // Source ID should be located at bytes 5-6, we use eui_addr to construct the source address
    whisper_vars.eui_addr.addr_128b[14] = command[5];
    whisper_vars.eui_addr.addr_128b[15] = command[6];
    packetfunctions_ip128bToMac64b(&whisper_vars.eui_addr,&temp,&whisper_vars.whisper_sixtop.source);

    // Stop parsing early when command is clear
    if(whisper_vars.whisper_sixtop.request_type == IANA_6TOP_CMD_CLEAR) {
        whisper_vars.whisper_sixtop.command_parsed = TRUE;
        return TRUE;
    }

    // CellType
    whisper_vars.whisper_sixtop.cellType = command[7];
    switch(command[7]) {
        case CELLOPTIONS_TX:
        case CELLOPTIONS_RX:
        case CELLOPTIONS_SHARED:
            break;
        default:
            whisper_log("Cell Type not valid, command parsing aborted.\n");
            return FALSE;
    }

    // Stop parsing early when command is list
    if(whisper_vars.whisper_sixtop.request_type == IANA_6TOP_CMD_LIST) {
        whisper_vars.whisper_sixtop.listMaxCells = (uint16_t) (command[8] << 8) | (uint16_t ) command[9];
        whisper_vars.whisper_sixtop.listOffset = (uint16_t) (command[10] << 8) | (uint16_t ) command[11];
        // No cell creation needed
        whisper_vars.whisper_sixtop.command_parsed = TRUE;
        return TRUE;
    }

    switch(command[8]) {
        case 0x01:
            // Cell is defined in the command
            slotOffset = (uint16_t) (command[9] << 8) | (uint16_t ) command[10];
            channel = (uint16_t) (command[11] << 8) | (uint16_t ) command[12];
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

    whisper_vars.whisper_sixtop.command_parsed = TRUE;
    return TRUE; // command parsed successfully
}

void whisperExecuteSixtop(void) {
    // The command should be parsed successfully in the whisper_vars
    if(whisper_vars.whisper_sixtop.command_parsed == FALSE) {
        whisper_log("Command not parsed correctly, not executing 6P\n.");
        return;
    }

    owerror_t request = E_FAIL;

    if(idmanager_isMyAddress(&whisper_vars.whisper_sixtop.target) ||
       idmanager_isMyAddress(&whisper_vars.whisper_sixtop.source)) {
        whisper_log("Whisper 6P commands with whisper node (me) as target is not allowed.\n");
        return;
    }

    if(schedule_getAutonomousTxRxCellUnicastNeighbor(&whisper_vars.whisper_sixtop.target)) {
        // Set ACK addresses
        whisper_vars.whisper_ack.ACKsrc.type = ADDR_64B;
        memcpy(&whisper_vars.whisper_ack.ACKsrc,&whisper_vars.whisper_sixtop.target, sizeof(open_addr_t));
        whisper_vars.whisper_ack.ACKdest.type = ADDR_64B;
        memcpy(&whisper_vars.whisper_ack.ACKdest,&whisper_vars.whisper_sixtop.source, sizeof(open_addr_t));
        whisper_vars.whisper_sixtop.waiting_for_response = TRUE;

        // Get seqNum
        uint8_t srcID[2] = {whisper_vars.whisper_sixtop.source.addr_64b[6], whisper_vars.whisper_sixtop.source.addr_64b[7]};
        uint8_t destID[2] = {whisper_vars.whisper_sixtop.target.addr_64b[6], whisper_vars.whisper_sixtop.target.addr_64b[7]};

        uint8_t seqNum;
        if (getSixtopLinkSeqNum(srcID, destID, &seqNum) == FALSE) {
            whisper_log("No link found in memory, sending 6P command with seqNum 0.\n");
            seqNum = 0;
            addSixtopLink(srcID, destID, 1);
        } else {
            whisper_log("Sending 6P command with seqNum: %d\n", seqNum);
            updateSixtopLinkSeqNum(srcID, destID, seqNum + 1);
        }

        // call sixtop
        request = sixtop_request_Whisper(
                whisper_vars.whisper_sixtop.request_type,  // code
                &whisper_vars.whisper_sixtop.target,       // neighbor
                whisper_vars.whisper_sixtop.cellType,      // cellOptions
                &whisper_vars.whisper_sixtop.cell,         // cell
                msf_getsfid(),                             // sfid
                whisper_vars.whisper_sixtop.listOffset,    // list command offset
                whisper_vars.whisper_sixtop.listMaxCells,  // list command maximum celllist
                seqNum                                     // Sequence number of 6P message
        );
    } else {
        whisper_log("No cell to target. 6P response would not be received.\n");
    }

    whisper_vars.whisper_sixtop.command_parsed = FALSE; // require the command to be (re-)parsed

    if(request == E_SUCCESS) {
        whisper_vars.state = WHISPER_STATE_SIXTOP;
        whisper_vars.whisper_ack.acceptACKs = TRUE;
        whisper_log("Sixtop request sent.\n");
    }
    else whisper_log("Sixtop request not sent. Something went wrong.\n");
}

void whisperGetNeighborInfoFromSixtop(ieee802154_header_iht* header, OpenQueueEntry_t* msg) {
    if( (idmanager_isMyAddress(&header->src) == FALSE && idmanager_isMyAddress(&header->dest) == FALSE) &&
        (isNeighbor(&header->src) && isNeighbor(&header->dest)) &&
        (header->ieListPresent == TRUE && header->frameType == IEEE154_TYPE_DATA))
    {
        uint8_t ptr = 0;
        uint8_t temp_8b, subtypeid, code, sfid, seqNum, version,type;
        uint16_t temp_16b;

        // First check if the IE header is valid
        temp_8b     = *((uint8_t*)(msg->payload)+ptr);
        ptr++;
        temp_16b    = temp_8b + ((*((uint8_t*)(msg->payload)+ptr))<<8);
        ptr++;
        // check ietf ie group id, type
        if ((temp_16b & IEEE802154E_DESC_LEN_PAYLOAD_ID_TYPE_MASK) != (IANA_IETF_IE_GROUP_ID | IANA_IETF_IE_TYPE))
            return;

        // check 6p subtype Id
        subtypeid = *((uint8_t*)(msg->payload)+ptr);
        ptr += 1;
        if (subtypeid != IANA_6TOP_SUBIE_ID) return;

        // check 6p version
        temp_8b = *((uint8_t*)(msg->payload)+ptr);
        ptr += 1;
        // 6p doesn't define type 3
        if (temp_8b>>IANA_6TOP_TYPE_SHIFT == 3) return;
        version    = temp_8b &  IANA_6TOP_VESION_MASK;
        type       = temp_8b >> IANA_6TOP_TYPE_SHIFT;

        // get 6p code
        code = *((uint8_t*)(msg->payload)+ptr);
        ptr += 1;

        // get 6p sfid
        sfid = *((uint8_t*)(msg->payload)+ptr);
        ptr += 1;

        // get 6p seqNum
        seqNum =  *((uint8_t*)(msg->payload)+ptr) & 0xff;

        if(type == SIXTOP_CELL_REQUEST && sfid == msf_getsfid()) {
            if (code != IANA_6TOP_CMD_CLEAR) {
                // Store the seqNum for the link
                seqNum++;
                whisperUpdateOrAddSixtopLinkInfo(&header->src, &header->dest, seqNum);
                whisperUpdateOrAddSixtopLinkInfo(&header->dest, &header->src, seqNum);
            } else {
                // Clear command
                whisper_log("Received clear request.\n");
                whisperUpdateOrAddSixtopLinkInfo(&header->src, &header->dest, 0);
                whisperUpdateOrAddSixtopLinkInfo(&header->dest, &header->src, 0);
            }
        }
    }
}

// ------------ Helper functions ------------------

void whisperUpdateOrAddSixtopLinkInfo(open_addr_t* src, open_addr_t* dest, uint8_t seqNum) {
    uint8_t srcID[2] = {src->addr_64b[6], src->addr_64b[7]};
    uint8_t destID[2] = {dest->addr_64b[6], dest->addr_64b[7]};

    for(uint8_t i = 0; i < SIXTOP_MAX_LINK_SNIFFING; i++) {
        if(isMatchingSixtopLink(i, srcID, destID)) {
            // update seqNum
            whisper_log("Updating link: %d -> %d, seqNum: %d.\n", srcID[1], destID[1], seqNum);
            whisper_vars.neighbors.sixtop[i].seqNum = seqNum;
            whisper_vars.neighbors.sixtop[i].active = TRUE;
            return;
        }
    }

    // If we get here the link is not found, add it
    if(addSixtopLink(srcID, destID, seqNum) == FALSE) {
        whisper_log("Received sixtop packet from neighbor link, could not store it.\n");
    }
}

bool isMatchingSixtopLink(uint8_t link_index, const uint8_t* srcID, const uint8_t* destID) {
    if ((srcID[0] == whisper_vars.neighbors.sixtop[link_index].srcId[0] &&
         srcID[1] == whisper_vars.neighbors.sixtop[link_index].srcId[1] ) &&
        (destID[0] == whisper_vars.neighbors.sixtop[link_index].destId[0] &&
         destID[1] == whisper_vars.neighbors.sixtop[link_index].destId[1]))
        return TRUE;

    return FALSE;
}

bool addSixtopLink(const uint8_t* srcID, const uint8_t* destID, uint8_t seqNum) {
    for(uint8_t i = 0; i < SIXTOP_MAX_LINK_SNIFFING; i++) {
        if(whisper_vars.neighbors.sixtop[i].active == FALSE) {
            whisper_vars.neighbors.sixtop[i].srcId[0] = srcID[0];
            whisper_vars.neighbors.sixtop[i].srcId[1] = srcID[1];
            whisper_vars.neighbors.sixtop[i].destId[0] = destID[0];
            whisper_vars.neighbors.sixtop[i].destId[1] = destID[1];
            whisper_vars.neighbors.sixtop[i].seqNum = seqNum;
            whisper_vars.neighbors.sixtop[i].active = TRUE;
            whisper_log("Adding link: %d -> %d, seqNum: %d.\n", srcID[1], destID[1], seqNum);
            return TRUE;
        }
    }
    return FALSE;
}

bool getSixtopLinkSeqNum(const uint8_t* srcID, const uint8_t* destID, uint8_t* seqNum) {
    for(uint8_t i = 0; i < SIXTOP_MAX_LINK_SNIFFING; i++) {
        if(isMatchingSixtopLink(i, srcID, destID) && whisper_vars.neighbors.sixtop[i].active) {
            *seqNum = whisper_vars.neighbors.sixtop[i].seqNum;
            return TRUE;
        }
    }
    return FALSE;
}

void updateSixtopLinkSeqNum(const uint8_t* srcID, const uint8_t* destID, uint8_t seqNum) {
    for(uint8_t i = 0; i < SIXTOP_MAX_LINK_SNIFFING; i++) {
        if(isMatchingSixtopLink(i, srcID, destID)) {
            whisper_log("Updating link: %d -> %d, seqNum: %d.\n", srcID[1], destID[1], seqNum);
            whisper_vars.neighbors.sixtop[i].seqNum = seqNum;
            return;
        }
    }
}

void removeAllSixtopLinksNeighbor(open_addr_t* neighbor) {
    uint8_t ID[2] = {neighbor->addr_64b[6], neighbor->addr_64b[7]};
    for(uint8_t i = 0; i < SIXTOP_MAX_LINK_SNIFFING; i++) {
        if ((ID[0] == whisper_vars.neighbors.sixtop[i].srcId[0] &&
            ID[1] == whisper_vars.neighbors.sixtop[i].srcId[1] ) ||
            (ID[0] == whisper_vars.neighbors.sixtop[i].destId[0] &&
            ID[1] == whisper_vars.neighbors.sixtop[i].destId[1]))
        {
            whisper_vars.neighbors.sixtop[i].seqNum = 0;
            whisper_vars.neighbors.sixtop[i].active = FALSE;
            return;
        }
    }
}

void sendCoapResponseToController(uint8_t *payload, uint8_t length) {
    OpenQueueEntry_t* pkt;
    coap_option_iht options[2];
    open_addr_t temp;

    // Do net send when no parent
    if(icmpv6rpl_getPreferredParentEui64(&temp) == FALSE)
        return;

    // create a CoAP packet
    pkt = openqueue_getFreePacketBuffer(COMPONENT_WHISPER);
    if (pkt==NULL) {
        openserial_printError(
                COMPONENT_WHISPER,
                ERR_NO_FREE_PACKET_BUFFER,
                (errorparameter_t)0,
                (errorparameter_t)0
        );
        return;
    }

    if(whisper_vars.state == WHISPER_STATE_WAIT_COAP)
        return;

    // take ownership over packet
    pkt->creator = COMPONENT_WHISPER;
    pkt->owner = COMPONENT_WHISPER;

    // CoAP payload
    packetfunctions_reserveHeaderSize(pkt,length);
    memcpy(&pkt->payload[0], payload, length);

    // set location-path option
    options[0].type = COAP_OPTION_NUM_URIPATH;
    options[0].length = sizeof(whisper_path0) - 1;
    options[0].pValue = (uint8_t *) whisper_path0;

    // set content-type option
    uint8_t medtype = COAP_MEDTYPE_APPOCTETSTREAM;
    options[1].type = COAP_OPTION_NUM_CONTENTFORMAT;
    options[1].length = 1;
    options[1].pValue = &medtype;

    // metadata
    pkt->l4_destination_port = 61620; // whisper controller port
    memcpy(&pkt->l3_destinationAdd, &whisper_vars.controller_addr, sizeof(open_addr_t));

    pkt->is6pFake = FALSE;
    pkt->isDioFake = FALSE;

    // send
    uint8_t outcome = opencoap_send(
            pkt,
            COAP_TYPE_NON,
            COAP_CODE_REQ_POST,
            1, // token len
            options,
            2, // options len
            &whisper_vars.desc
    );

    // avoid overflowing the queue if fails
    if(outcome==E_FAIL) {
        openqueue_freePacketBuffer(pkt);
    } else {
        whisper_log("Response message sent.\n");
    }
}

// ============================================================================================
// Logging (should be removed for openmote build, no printf)
void whisper_log(char* msg, ...) {
	open_addr_t my_id = *idmanager_getMyID(ADDR_16B);

	char state[20];
	switch(whisper_vars.state) {
        case WHISPER_STATE_IDLE: strcpy(state, "IDLE");
            break;
        case WHISPER_STATE_SIXTOP: strcpy(state, "SIXTOP");
            break;
        case WHISPER_STATE_DIO: strcpy(state, "DIO");
            break;
        case WHISPER_STATE_WAIT_COAP: strcpy(state, "WAIT_COAP");
            break;
        default: strcpy(state, "UNKNOWN");
            break;
	}

	printf("[%s] [%d] whisper_node - ", state, my_id.addr_64b[1]);

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
