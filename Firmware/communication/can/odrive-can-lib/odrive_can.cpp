/*
 * odrive_can.cpp
 *
 *  Created on: Oct 27, 2023
 *      Author: Dragonship
 */

#include "odrive_can.h"

void ODriveCanHandler::init(CAN_HandleTypeDef* phcan){
	hcan = phcan;
	HAL_CAN_Start(hcan);
}

void ODriveCanHandler::addODriveInstance(ODriveCan* podrivecan){
	odrive_instance_vector.push_back(podrivecan);
}


HAL_StatusTypeDef ODriveCanHandler::setFilterConfig(CAN_FilterTypeDef* pfilter_config){
	HAL_StatusTypeDef status;
	status = HAL_CAN_ConfigFilter(hcan, pfilter_config);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	return status;
}

void ODriveCanHandler::distributeData(){
	CAN_RxHeaderTypeDef rx_header;
	uint8_t data[8];
	uint32_t parsed_odrive_id;
	uint32_t parsed_cmd_id;
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0 , &rx_header, data);
	parsed_odrive_id = rx_header.StdId >> 5;
	parsed_cmd_id = rx_header.StdId & 0x1F;

	for(auto& it: odrive_instance_vector){
		if(parsed_odrive_id == it->getODriveId()){
			it->filterMethod(parsed_cmd_id, data);
			break;
		}
	}
}

void ODriveCanHandler::updateTimestamp(){
	timestamp = HAL_GetTick();
	is_timeout = false;
}

void ODriveCanHandler::setTimeout(uint32_t _timeout){
	timeout = _timeout;
}

bool ODriveCanHandler::checkTimeout(){
	if(HAL_GetTick() - timestamp > timeout){
		is_timeout = true;
	}else{
		is_timeout = false;
	}
	return is_timeout;
}

ODriveErr_t ODriveCan::parseError(uint8_t recv_data[]){
	ODriveErr_t err;

	err.active_err = (ODriveError)(uint32_t)(recv_data[0] | (recv_data[1] << 8) | (recv_data[2] << 16) | (recv_data[3] << 24));
	err.disarm_reason = (ODriveError)(uint32_t)(recv_data[4] | (recv_data[5] << 8) | (recv_data[6] << 16) | (recv_data[7] << 24));

	this->err = err;
	return err;
}

ODriveIq_t ODriveCan::parseIq(uint8_t recv_data[]){
	ODriveIq_t odriveIq;

	union TypeConverter_t data;
	data.i_integer =  (uint32_t)(recv_data[0] | (recv_data[1] << 8) | (recv_data[2] << 16) | (recv_data[3] << 24));
	odriveIq.Iq_setpoint = data.f_float;
	data.i_integer = (uint32_t)(recv_data[4] | (recv_data[5] << 8) | (recv_data[6] << 16) | (recv_data[7] << 24));
	odriveIq.Iq_measured = data.f_float;

	this->Iq = odriveIq;
	return odriveIq;
}

ODriveTemp_t ODriveCan::parseTemperature(uint8_t recv_data[]){
	ODriveTemp_t temp;

	union TypeConverter_t data;
	data.i_integer = (uint32_t)(recv_data[0] | (recv_data[1] << 8) | (recv_data[2] << 16) | (recv_data[3] << 24));
	temp.fet_temp = data.f_float;
	data.i_integer = (uint32_t)(recv_data[4] | (recv_data[5] << 8) | (recv_data[6] << 16) | (recv_data[7] << 24));
	temp.motor_temp = data.f_float;

	this->temp = temp;
	return temp;
}

ODriveBus_t ODriveCan::parseBusVoltageCurrent(uint8_t recv_data[]){
	ODriveBus_t odrivebus;

	union TypeConverter_t data;
	data.i_integer =  (uint32_t)(recv_data[0] | (recv_data[1] << 8) | (recv_data[2] << 16) | (recv_data[3] << 24));
	odrivebus.bus_voltage = data.f_float;
	data.i_integer = (uint32_t)(recv_data[4] | (recv_data[5] << 8) | (recv_data[6] << 16) | (recv_data[7] << 24));
	odrivebus.bus_current = data.f_float;

	this->bus = odrivebus;
	return odrivebus;
}

ODriveTorque_t ODriveCan::parseTorque(uint8_t recv_data[]){
	ODriveTorque_t odrivetorque;

	union TypeConverter_t data;
	data.i_integer =  (uint32_t)(recv_data[0] | (recv_data[1] << 8) | (recv_data[2] << 16) | (recv_data[3] << 24));
	odrivetorque.torque_target = data.f_float;
	data.i_integer = (uint32_t)(recv_data[4] | (recv_data[5] << 8) | (recv_data[6] << 16) | (recv_data[7] << 24));
	odrivetorque.torque_estimate = data.f_float;

	this->torque = odrivetorque;
	return odrivetorque;
}

ODriveHeartbeat_t ODriveCan::parseHeartbeat(uint8_t* recv_data){
	ODriveHeartbeat_t heartbeat;

	heartbeat.active_err = (ODriveError)(uint32_t)(recv_data[0] | (recv_data[1] << 8));
	heartbeat.disarm_reason = (ODriveError)(uint32_t)(recv_data[2] | (recv_data[3] << 8));
	heartbeat.axis_state = (ODriveAxisState)(uint32_t)recv_data[4];
	heartbeat.procedure_result = (ODriveProcedureResult)(uint32_t)recv_data[5];
	heartbeat.trajectory_done_flag = recv_data[6];

	this->heartbeat = heartbeat;
	return heartbeat;
}

ODriveEncoderEstimate_t ODriveCan::parseEncoderEstimate(uint8_t* recv_data){
	ODriveEncoderEstimate_t encoder_est;

	union TypeConverter_t data;
	data.i_integer =  (uint32_t)(recv_data[0] | (recv_data[1] << 8) | (recv_data[2] << 16) | (recv_data[3] << 24));
	encoder_est.pos_estimate = data.f_float;
	data.i_integer = (uint32_t)(recv_data[4] | (recv_data[5] << 8) | (recv_data[6] << 16) | (recv_data[7] << 24));
	encoder_est.vel_estimate = data.f_float;

	this->encoder_est = encoder_est;
	return encoder_est;
}

ODriveErr_t ODriveCan::getError(bool send_command){
	if(!send_command){
		return err;
	}

	uint32_t std_id;
	uint8_t data[8] = {0,0,0,0,0,0,0,0};

	std_id = odrive_id << 5 | GET_ERROR;
	sendData(std_id, data, sizeof(data), CAN_RTR_REMOTE);

	return err;
}

ODriveIq_t ODriveCan::getIq(bool send_command){
	if(!send_command){
		return Iq;
	}

	uint32_t std_id;
	uint8_t data[8] = {0,0,0,0,0,0,0,0};

	std_id = odrive_id << 5 | GET_IQ;
	sendData(std_id, data, sizeof(data), CAN_RTR_REMOTE);

	return Iq;
}

ODriveTemp_t ODriveCan::getTemperature(bool send_command){
	if(!send_command){
		return temp;
	}

	uint32_t std_id;
	uint8_t data[8] = {0,0,0,0,0,0,0,0};

	std_id = odrive_id << 5 | GET_TEMPERATURE;
	sendData(std_id, data, sizeof(data), CAN_RTR_REMOTE);

	return temp;
}

ODriveBus_t ODriveCan::getBusVoltageCurrent(bool send_command){
	if(!send_command){
		return bus;
	}

	uint32_t std_id;
	uint8_t data[8] = {0,0,0,0,0,0,0,0};

	std_id = odrive_id << 5 | GET_BUS_VOLTAGE_CURRENT;
	sendData(std_id, data, sizeof(data), CAN_RTR_REMOTE);

	return bus;
}

ODriveTorque_t ODriveCan::getTorque(bool send_command){
	if(!send_command){
		return torque;
	}

	uint32_t std_id;
	uint8_t data[8] = {0,0,0,0,0,0,0,0};

	std_id = odrive_id << 5 | GET_TORQUES;
	sendData(std_id, data, sizeof(data), CAN_RTR_REMOTE);

	return torque;
}

ODrivePower_t ODriveCan::getPower(bool send_command){
	if(!send_command){
		return power;
	}

	uint32_t std_id;
	uint8_t data[8] = {0,0,0,0,0,0,0,0};

	std_id = odrive_id << 5 | GET_POWERS;
	sendData(std_id, data, sizeof(data), CAN_RTR_REMOTE);

	return power;
}

ODriveHeartbeat_t ODriveCan::getHeartbeat(bool send_command){
	if(!send_command){
		return heartbeat;
	}

	uint32_t std_id;
	uint8_t data[8] = {0,0,0,0,0,0,0,0};

	std_id = odrive_id << 5 | HEARTBEAT;
	sendData(std_id, data, sizeof(data), CAN_RTR_REMOTE);

	return heartbeat;
}

ODriveEncoderEstimate_t ODriveCan::getEncoderEstimate(bool send_command){
	if(!send_command){
		return encoder_est;
	}

	uint32_t std_id;
	uint8_t data[8] = {0,0,0,0,0,0,0,0};

	std_id = odrive_id << 5 | GET_ENCODER_ESTIMATES;
	sendData(std_id, data, sizeof(data), CAN_RTR_REMOTE);

	return encoder_est;
}

ODriveAxisState ODriveCan::getAxisState(){
	return heartbeat.axis_state;
}

void ODriveCan::filterMethod(uint32_t cmd_id, uint8_t data[]){
	switch(cmd_id){
	case GET_VERSION:
		break;
	case HEARTBEAT:
		parseHeartbeat(data);
		break;
	case GET_ERROR:
		parseError(data);
		break;
	case TX_SDO:
		break;
	case GET_ENCODER_ESTIMATES:
		parseEncoderEstimate(data);
		break;
	case GET_IQ:
		parseIq(data);
		break;
	case GET_TEMPERATURE:
		parseTemperature(data);
		break;
	case GET_BUS_VOLTAGE_CURRENT:
		parseBusVoltageCurrent(data);
		break;
	case GET_TORQUES:
		parseTorque(data);
		break;
	default:
		break;
	}
}

void ODriveCan::setAxisState(ODriveAxisState axis_state){
	uint32_t std_id;
	uint8_t data[8];
	uint32_t axis_state_uint32 = (uint32_t)axis_state;
	std_id = odrive_id << 5 | SET_AXIS_STATE;

	data[0] = axis_state_uint32;
	data[1] = (axis_state_uint32 >> 8) & 0xFF;
	data[2] = (axis_state_uint32 >> 16) & 0xFF;
	data[3] = (axis_state_uint32 >> 24) & 0xFF;
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;
	sendData(std_id, data, sizeof(data), CAN_RTR_DATA);

}

void ODriveCan::setControllerMode(ODriveControlMode control_mode, ODriveInputMode input_mode){
	uint32_t std_id;
	uint8_t data[8];
	uint32_t control_mode_uint32_t = (uint32_t)control_mode;
	uint32_t input_mode_uint32_t = (uint32_t)input_mode;
	std_id = odrive_id << 5 | SET_CONTROLLER_MODE;

	data[0] = control_mode_uint32_t;
	data[1] = (control_mode_uint32_t >> 8) & 0xFF;
	data[2] = (control_mode_uint32_t >> 16) & 0xFF;
	data[3] = (control_mode_uint32_t >> 24) & 0xFF;
	data[4] = input_mode_uint32_t;
	data[5] = (input_mode_uint32_t >> 8) & 0xFF;
	data[6] = (input_mode_uint32_t >> 16) & 0xFF;
	data[7] = (input_mode_uint32_t >> 24) & 0xFF;
	sendData(std_id, data, sizeof(data), CAN_RTR_DATA);
}

void ODriveCan::setInputPos(float input_pos){
	setInputPos(input_pos, 0);
}

void ODriveCan::setInputPos(float input_pos, int16_t input_vel_ff){
	setInputPos(input_pos, input_vel_ff, 0);
}

void ODriveCan::setInputPos(float input_pos, int16_t input_vel_ff, int16_t input_torque_ff){
	uint32_t std_id;
	uint8_t data[8];
	union TypeConverter_t input_pos_;
	input_pos_.f_float = input_pos;
	uint32_t input_vel_ff_uint32_t = (uint32_t)input_vel_ff;
	uint32_t input_torque_ff_uint32_t = (uint32_t)input_torque_ff;
	std_id = odrive_id << 5 | SET_INPUT_POS;

	data[0] = input_pos_.i_integer;
	data[1] = (input_pos_.i_integer >> 8) & 0xFF;
	data[2] = (input_pos_.i_integer >> 16) & 0xFF;
	data[3] = (input_pos_.i_integer >> 24) & 0xFF;
	data[4] = input_vel_ff_uint32_t;
	data[5] = (input_vel_ff_uint32_t >> 8) & 0xFF;
	data[6] = input_torque_ff_uint32_t;
	data[7] = (input_torque_ff_uint32_t >> 8) & 0xFF;
	sendData(std_id, data, sizeof(data), CAN_RTR_DATA);
}

void ODriveCan::setInputVel(float input_vel, float input_torque_ff){
	uint32_t std_id;
	uint8_t data[8];
	union TypeConverter_t input_vel_;
	union TypeConverter_t input_torque_ff_;

	input_vel_.f_float = input_vel;
	input_torque_ff_.f_float = input_torque_ff;

	std_id = odrive_id << 5 | SET_INPUT_VEL;

	data[0] = input_vel_.i_integer;
	data[1] = (input_vel_.i_integer >> 8) & 0xFF;
	data[2] = (input_vel_.i_integer >> 16) & 0xFF;
	data[3] = (input_vel_.i_integer >> 24) & 0xFF;
	data[4] = input_torque_ff_.i_integer;
	data[5] = (input_torque_ff_.i_integer >> 8) & 0xFF;
	data[6] = (input_torque_ff_.i_integer >> 16) & 0xFF;
	data[7] = (input_torque_ff_.i_integer >> 24) & 0xFF;
	sendData(std_id, data, sizeof(data), CAN_RTR_DATA);
}

void ODriveCan::setInputTorque(float input_torque){
	uint32_t std_id;
	uint8_t data[8];
	union TypeConverter_t input_torque_;

	input_torque_.f_float = input_torque;

	std_id = odrive_id << 5 | SET_INPUT_TORQUE;

	data[0] = input_torque_.i_integer;
	data[1] = (input_torque_.i_integer >> 8) & 0xFF;
	data[2] = (input_torque_.i_integer >> 16) & 0xFF;
	data[3] = (input_torque_.i_integer >> 24) & 0xFF;
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;
	sendData(std_id, data, sizeof(data), CAN_RTR_DATA);
}

void ODriveCan::setLimits(float vel_limit, float current_limit){
	uint32_t std_id;
	uint8_t data[8];
	union TypeConverter_t vel_limit_;
	union TypeConverter_t current_limit_;

	vel_limit_.f_float = vel_limit;
	current_limit_.f_float = current_limit;

	std_id = odrive_id << 5 | SET_LIMITS;

	data[0] = vel_limit_.i_integer;
	data[1] = (vel_limit_.i_integer >> 8) & 0xFF;
	data[2] = (vel_limit_.i_integer >> 16) & 0xFF;
	data[3] = (vel_limit_.i_integer >> 24) & 0xFF;
	data[4] = current_limit_.i_integer;
	data[5] = (current_limit_.i_integer >> 8) & 0xFF;
	data[6] = (current_limit_.i_integer >> 16) & 0xFF;
	data[7] = (current_limit_.i_integer >> 24) & 0xFF;
	sendData(std_id, data, sizeof(data), CAN_RTR_DATA);
}

void ODriveCan::setTrajVelLimits(float traj_vel_limit){
	uint32_t std_id;
	uint8_t data[8];
	union TypeConverter_t traj_vel_limit_;

	traj_vel_limit_.f_float = traj_vel_limit;

	std_id = odrive_id << 5 | SET_TRAJ_VEL_LIMIT;

	data[0] = traj_vel_limit_.i_integer;
	data[1] = (traj_vel_limit_.i_integer >> 8) & 0xFF;
	data[2] = (traj_vel_limit_.i_integer >> 16) & 0xFF;
	data[3] = (traj_vel_limit_.i_integer >> 24) & 0xFF;
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;
	sendData(std_id, data, sizeof(data), CAN_RTR_DATA);
}

void ODriveCan::setTrajAccelLimits(float traj_accel_limit, float traj_decel_limit){
	uint32_t std_id;
	uint8_t data[8];
	union TypeConverter_t traj_accel_limit_;
	union TypeConverter_t traj_decel_limit_;

	traj_accel_limit_.f_float = traj_accel_limit;
	traj_decel_limit_.f_float = traj_decel_limit;

	std_id = odrive_id << 5 | SET_TRAJ_ACCEL_LIMIT;

	data[0] = traj_accel_limit_.i_integer;
	data[1] = (traj_accel_limit_.i_integer >> 8) & 0xFF;
	data[2] = (traj_accel_limit_.i_integer >> 16) & 0xFF;
	data[3] = (traj_accel_limit_.i_integer >> 24) & 0xFF;
	data[4] = traj_decel_limit_.i_integer;
	data[5] = (traj_decel_limit_.i_integer >> 8) & 0xFF;
	data[6] = (traj_decel_limit_.i_integer >> 16) & 0xFF;
	data[7] = (traj_decel_limit_.i_integer >> 24) & 0xFF;
	sendData(std_id, data, sizeof(data), CAN_RTR_DATA);
}

void ODriveCan::setTrajInertia(float traj_inertia){
	uint32_t std_id;
	uint8_t data[8];
	union TypeConverter_t traj_inertia_;

	traj_inertia_.f_float = traj_inertia;

	std_id = odrive_id << 5 | SET_TRAJ_INERTIA;

	data[0] = traj_inertia_.i_integer;
	data[1] = (traj_inertia_.i_integer >> 8) & 0xFF;
	data[2] = (traj_inertia_.i_integer >> 16) & 0xFF;
	data[3] = (traj_inertia_.i_integer >> 24) & 0xFF;
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;
	sendData(std_id, data, sizeof(data), CAN_RTR_DATA);
}

void ODriveCan::setAbsolutePosition(float position){
	uint32_t std_id;
	uint8_t data[8];
	union TypeConverter_t position_;

	position_.f_float = position;

	std_id = odrive_id << 5 | SET_ABSOLUTE_POSITION;

	data[0] = position_.i_integer;
	data[1] = (position_.i_integer >> 8) & 0xFF;
	data[2] = (position_.i_integer >> 16) & 0xFF;
	data[3] = (position_.i_integer >> 24) & 0xFF;
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;
	sendData(std_id, data, sizeof(data), CAN_RTR_DATA);
}

void ODriveCan::setPosGain(float pos_gain){
	uint32_t std_id;
	uint8_t data[8];
	union TypeConverter_t pos_gain_;

	pos_gain_.f_float = pos_gain;

	std_id = odrive_id << 5 | SET_POS_GAIN;

	data[0] = pos_gain_.i_integer;
	data[1] = (pos_gain_.i_integer >> 8) & 0xFF;
	data[2] = (pos_gain_.i_integer >> 16) & 0xFF;
	data[3] = (pos_gain_.i_integer >> 24) & 0xFF;
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;
	sendData(std_id, data, sizeof(data), CAN_RTR_DATA);
}

void ODriveCan::setVelGains(float vel_gain, float vel_integrator_gain){
	uint32_t std_id;
	uint8_t data[8];
	union TypeConverter_t vel_gain_;
	union TypeConverter_t vel_integrator_gain_;

	vel_gain_.f_float = vel_gain;
	vel_integrator_gain_.f_float = vel_integrator_gain;

	std_id = odrive_id << 5 | SET_VEL_GAINS;

	data[0] = vel_gain_.i_integer;
	data[1] = (vel_gain_.i_integer >> 8) & 0xFF;
	data[2] = (vel_gain_.i_integer >> 16) & 0xFF;
	data[3] = (vel_gain_.i_integer >> 24) & 0xFF;
	data[4] = vel_integrator_gain_.i_integer;
	data[5] = (vel_integrator_gain_.i_integer >> 8) & 0xFF;
	data[6] = (vel_integrator_gain_.i_integer >> 16) & 0xFF;
	data[7] = (vel_integrator_gain_.i_integer >> 24) & 0xFF;
	sendData(std_id, data, sizeof(data), CAN_RTR_DATA);
}

void ODriveCan::enterDFUMode(){
	uint32_t std_id;
	uint8_t data[8] = {0,0,0,0,0,0,0,0};

	std_id = odrive_id << 5 | ENTER_DFU_MODE;
	sendData(std_id, data, sizeof(data), CAN_RTR_DATA);
}

void ODriveCan::clearErrors(){
	uint32_t std_id;
	uint8_t data[8] = {0,0,0,0,0,0,0,0};

	std_id = odrive_id << 5 | CLEAR_ERRORS;
	sendData(std_id, data, sizeof(data), CAN_RTR_DATA);
}

void ODriveCan::reboot(){
	uint32_t std_id;
	uint8_t data[8] = {0,0,0,0,0,0,0,0};

	std_id = odrive_id << 5 | REBOOT;
	sendData(std_id, data, sizeof(data), CAN_RTR_DATA);
}

void ODriveCan::estop(){
	uint32_t std_id;
	uint8_t data[8] = {0,0,0,0,0,0,0,0};

	std_id = odrive_id << 5 | ESTOP;
	sendData(std_id, data, sizeof(data), CAN_RTR_DATA);
}

bool ODriveCan::sendData(uint32_t id, const uint8_t *pdata, uint8_t len, uint32_t rtr_frame){
	uint32_t mailbox;
	CAN_TxHeaderTypeDef tx_header; 					//CAN Bus Receive Header
	uint8_t send_buffer[8];  						//CAN Bus Receive Buffer

	if (len > 8) {
			len = 8;
	}

	tx_header.DLC = len;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = rtr_frame;
	tx_header.StdId = id;
	tx_header.TransmitGlobalTime = DISABLE;

	memcpy(send_buffer, pdata, len);

	if(HAL_CAN_AddTxMessage(this->hcan,&tx_header, send_buffer,&mailbox) == HAL_OK)
		return true;
	return false;
}

