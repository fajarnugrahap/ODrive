/*
 * odrive-can.h
 *
 *  Created on: Oct 27, 2023
 *      Author: Dragonship
 */

#ifndef ODRIVE_CAN_LIB_ODRIVE_CAN_H_
#define ODRIVE_CAN_LIB_ODRIVE_CAN_H_

#include "main.h"

#include <vector>
#include "string.h"
#include "odrive_enums.h"

union TypeConverter_t{
        uint32_t i_integer;
        float f_float;
};

typedef struct{
	ODriveError active_err;
	ODriveError disarm_reason;
}ODriveErr_t;

typedef struct{
	float Iq_setpoint;
	float Iq_measured;
}ODriveIq_t;

typedef struct{
	float fet_temp;
	float motor_temp;
}ODriveTemp_t;

typedef struct{
	float bus_voltage;
	float bus_current;
}ODriveBus_t;

typedef struct{
	float torque_target;
	float torque_estimate;
}ODriveTorque_t;

typedef struct{
	float electrical_power;
	float mechanical_power;
}ODrivePower_t;

typedef struct{
	ODriveError active_err;
	ODriveError disarm_reason;
	ODriveAxisState axis_state;
	ODriveProcedureResult procedure_result;
	bool trajectory_done_flag;
}ODriveHeartbeat_t;

typedef struct{
	float pos_estimate;
	float vel_estimate;
}ODriveEncoderEstimate_t;

class ODriveCan;

class ODriveCanHandler{
public:
	void init(CAN_HandleTypeDef* phcan);
	HAL_StatusTypeDef setFilterConfig(CAN_FilterTypeDef* pfilter_config);
	void addODriveInstance(ODriveCan *podrivecan);
	void distributeData();
	void updateTimestamp();
	void setTimeout(uint32_t _timeout = 2000);	//set timeout in ms
	bool checkTimeout();

	std::vector<ODriveCan*> getODriveVector();
private:

	std::vector<ODriveCan*> odrive_instance_vector;
	CAN_HandleTypeDef* hcan;
	uint32_t timeout;	//in ms
	bool is_timeout;	//check if the messages received or not
	uint32_t timestamp;
};

class ODriveCan{
public:
	ODriveCan(uint32_t odrive_id_, CAN_HandleTypeDef* phcan){
		odrive_id = odrive_id_;
		hcan = phcan;
	}

	uint32_t getODriveId(){
		return odrive_id;
	}
	ODriveErr_t parseError(uint8_t recv_data[]);
	ODriveIq_t parseIq(uint8_t recv_data[]);
	ODriveTemp_t parseTemperature(uint8_t recv_data[]);
	ODriveBus_t parseBusVoltageCurrent(uint8_t recv_data[]);
	ODriveTorque_t parseTorque(uint8_t recv_data[]);
	ODriveHeartbeat_t parseHeartbeat(uint8_t* recv_data);
	ODriveEncoderEstimate_t parseEncoderEstimate(uint8_t* recv_data);

	ODriveErr_t getError(bool send_command = false);
	ODriveIq_t getIq(bool send_command = false);
	ODriveTemp_t getTemperature(bool send_command = false);
	ODriveBus_t getBusVoltageCurrent(bool send_command = false);
	ODriveTorque_t getTorque(bool send_command = false);
	ODrivePower_t getPower(bool send_command = false);
	ODriveHeartbeat_t getHeartbeat(bool send_command = false);
	ODriveEncoderEstimate_t getEncoderEstimate(bool send_command = false);

	ODriveAxisState getAxisState();

	void filterMethod(uint32_t cmd_id, uint8_t data[]);
	void setAxisState(ODriveAxisState axis_state);
	void setControllerMode(ODriveControlMode control_mode, ODriveInputMode input_mode);
	void setInputPos(float input_pos);
	void setInputPos(float input_pos, int16_t input_vel_ff);
	void setInputPos(float input_pos, int16_t input_vel_ff, int16_t input_torque_ff);
	void setInputVel(float input_vel, float input_torque_ff);
	void setInputTorque(float input_torque);
	void setLimits(float vel_limit, float current_limit);
	void setTrajVelLimits(float traj_vel_limit);
	void setTrajAccelLimits(float traj_accel_limit, float traj_decel_limit);
	void setTrajInertia(float traj_inertia);
	void setAbsolutePosition(float position);
	void setPosGain(float pos_gain);
	void setVelGains(float vel_gain, float vel_integrator_gain);

	void enterDFUMode();
	void clearErrors();
	void reboot();
	void estop();

private:
	uint32_t odrive_id;
	uint32_t cmd_id;

	ODriveErr_t err;
	ODriveTemp_t temp;
	ODriveHeartbeat_t heartbeat;
	ODriveBus_t bus;
	ODriveEncoderEstimate_t encoder_est;
	ODriveIq_t Iq;
	ODriveTorque_t torque;
	ODrivePower_t power;


	CAN_HandleTypeDef *hcan;
	CAN_FilterTypeDef *filter_config;
	CAN_RxHeaderTypeDef rx_header;
	CAN_TxHeaderTypeDef tx_header;

	bool sendData(uint32_t id, const uint8_t *pdata, uint8_t len, uint32_t rtr_frame);
};



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif /* ODRIVE_CAN_LIB_ODRIVE_CAN_H_ */
