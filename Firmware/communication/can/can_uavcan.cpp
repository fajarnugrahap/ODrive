#include "can_uavcan.hpp"
#include <libcanard/canard.h>

static CanardInstance _canard;
static uint8_t memoryPool[1024];
CanardCANFrame rxFrame;

void UAVCAN::init(){
	canardInit(&_canard,
			memoryPool,
			sizeof(memoryPool),
			onTransferReceived,
			shouldAcceptTransfer,
			NULL);
	canardSetLocalNodeID(&_canard, 1); 
}

void sendNodeStatus(bool isFailEnggaged){
	uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
	struct uavcan_protocol_NodeStatus nodeStatus;

	nodeStatus.uptime_sec 					= HAL_GetTick() / 1000;
	if(!isFailEnggaged){
		nodeStatus.health					= UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
	}
	else{
		nodeStatus.health					= UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR;
	}
	nodeStatus.mode							= UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
	nodeStatus.sub_mode						= 0;
	nodeStatus.vendor_specific_status_code	= 0;

	uint32_t len = uavcan_protocol_NodeStatus_encode(&nodeStatus, buffer);

	static uint8_t transferId;

	canardBroadcast(&_canard,
					UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
					UAVCAN_PROTOCOL_NODESTATUS_ID,
					&transferId,
					CANARD_TRANSFER_PRIORITY_LOW,
					buffer,
					len);
}

void sendActuatorStatus(){
	uavcan_equipment_actuator_Status actuatorStatus;
	uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE];

	actuatorStatus.actuator_id 		= 1;
	actuatorStatus.position 		= 9;
	actuatorStatus.torque			= 5;
	actuatorStatus.velocity			= 0;
	actuatorStatus.control_power	= 0;
	actuatorStatus.error_status		= 0;
	actuatorStatus.temperature		= 0;
	actuatorStatus.power_rating_pct = 0;

	uint32_t len = uavcan_equipment_actuator_Status_encode(&actuatorStatus, buffer);

	static uint8_t transferId;

	canardBroadcast(&_canard,
					UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE,
					UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID,
					&transferId,
					CANARD_TRANSFER_PRIORITY_MEDIUM,
					buffer,
					len);
}





void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer){
	if(transfer->transfer_type == CanardTransferTypeRequest){
		switch(transfer->data_type_id){
			case UAVCAN_PROTOCOL_GETNODEINFO_ID:{
				handleGetNodeInfo(ins, transfer);
				break;
			}
		}
	}
	else if(transfer->transfer_type == CanardTransferTypeBroadcast){
		switch(transfer->data_type_id){
			case UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID:{
				handleServoCommand(ins, transfer);
				break;
			}
		}
	}
}

bool shouldAcceptTransfer(const CanardInstance *ins,
        						uint64_t *out_data_type_signature,
								uint16_t data_type_id,
								CanardTransferType transfer_type,
								uint8_t source_node_id){
	switch(data_type_id){
		case UAVCAN_PROTOCOL_GETNODEINFO_ID:{
			*out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
			return true;
		}
		case UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID:{
			*out_data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE;
			return true;
		}
	}
	return false;
}

// Provide weak/no-op handlers so firmware builds even if the application
// doesn't implement these. If you implement these elsewhere, replace
// or remove these stubs.
// void handleGetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer) {
// 	(void)ins; (void)transfer;
// }

// void handleServoCommand(CanardInstance *ins, CanardRxTransfer *transfer) {
// 	(void)ins; (void)transfer;
// }