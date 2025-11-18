#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define UAVCAN_EQUIPMENT_LOADCELL_SENSOR_LOADCELLDATA_MAX_SIZE 3
#define UAVCAN_EQUIPMENT_LOADCELL_SENSOR_LOADCELLDATA_SIGNATURE (0x8E7A6B45FF47D731ULL)
#define UAVCAN_EQUIPMENT_LOADCELL_SENSOR_LOADCELLDATA_ID 21000

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class uavcan_equipment_loadcell_sensor_LoadCellData_cxx_iface;
#endif

struct uavcan_equipment_loadcell_sensor_LoadCellData {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = uavcan_equipment_loadcell_sensor_LoadCellData_cxx_iface;
#endif
    uint8_t loadcell_id;
    float measurement;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t uavcan_equipment_loadcell_sensor_LoadCellData_encode(struct uavcan_equipment_loadcell_sensor_LoadCellData* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_loadcell_sensor_LoadCellData_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_loadcell_sensor_LoadCellData* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_loadcell_sensor_LoadCellData_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_loadcell_sensor_LoadCellData* msg, bool tao);
static inline bool _uavcan_equipment_loadcell_sensor_LoadCellData_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_loadcell_sensor_LoadCellData* msg, bool tao);
void _uavcan_equipment_loadcell_sensor_LoadCellData_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_loadcell_sensor_LoadCellData* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->loadcell_id);
    *bit_ofs += 8;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->measurement);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
}

/*
 decode uavcan_equipment_loadcell_sensor_LoadCellData, return true on failure, false on success
*/
bool _uavcan_equipment_loadcell_sensor_LoadCellData_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_loadcell_sensor_LoadCellData* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->loadcell_id);
    *bit_ofs += 8;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->measurement = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_loadcell_sensor_LoadCellData sample_uavcan_equipment_loadcell_sensor_LoadCellData_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
BROADCAST_MESSAGE_CXX_IFACE(uavcan_equipment_loadcell_sensor_LoadCellData, UAVCAN_EQUIPMENT_LOADCELL_SENSOR_LOADCELLDATA_ID, UAVCAN_EQUIPMENT_LOADCELL_SENSOR_LOADCELLDATA_SIGNATURE, UAVCAN_EQUIPMENT_LOADCELL_SENSOR_LOADCELLDATA_MAX_SIZE);
#endif
#endif
