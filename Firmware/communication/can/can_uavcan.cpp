#include <canard.h>
#include <drivers/stm32/canard_stm32.h>



static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer){
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

static bool shouldAcceptTransfer(const CanardInstance *ins,
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