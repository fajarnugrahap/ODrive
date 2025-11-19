#ifndef __CAN_UAVCAN_HPP
#define __CAN_UAVCAN_HPP

#include "canbus.hpp"
#include "axis.hpp"

#include <libcanard/canard.h>
#include <libcanard/drivers/stm32/canard_stm32.h>
#include <dsdl_generated/include/dronecan_msgs.h>

void sendNodeStatus(bool isFailEnggaged);
void sendActuatorStatus();

void handleGetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer);
void handleServoCommand(CanardInstance *ins, CanardRxTransfer *transfer);

void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer);
bool shouldAcceptTransfer(const CanardInstance *ins,
        						uint64_t *out_data_type_signature,
								uint16_t data_type_id,
								CanardTransferType transfer_type,
								uint8_t source_node_id);

class UAVCAN {
    public:
        void init();
        void process();
    
}
;

#endif // __CAN_UAVCAN_HPP