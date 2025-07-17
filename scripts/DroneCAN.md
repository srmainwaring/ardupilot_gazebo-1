# Gazebo to DroneCAN

DroneCAN messages supported by ArduPilot

The ArduPilot wiki [DroneCAN Setup](https://ardupilot.org/copter/docs/common-uavcan-setup-advanced.html) page describes how to configure DroneCAN peripherals.

ADSB Receiver

Airspeed

- AP_Airspeed/AP_Airspeed_DroneCAN
- uavcan_equipment_air_data_RawAirData
- dronecan_sensors_hygrometer_Hygrometer

Barometer

- AP_Baro/AP_Baro_DroneCAN
- uavcan_equipment_air_data_StaticPressure
- uavcan_equipment_air_data_StaticTemperature

Battery Monitor

- AP_BattMonitor/AP_BattMonitor_DroneCAN
- uavcan_equipment_power_BatteryInfo
- ardupilot_equipment_power_BatteryInfoAux
- mppt_Stream

Compass

- AP_Compass/AP_Compass_DroneCAN
- uavcan_equipment_ahrs_MagneticFieldStrength
- uavcan_equipment_ahrs_MagneticFieldStrength2
- dronecan_sensors_magnetometer_MagneticFieldStrengthHiRes

EFI

- AP_EFI/AP_EFI_DroneCAN
- uavcan_equipment_ice_reciprocating_Status

GPS

- AP_GPS/AP_GPS_DroneCAN
- uavcan_equipment_gnss_Fix2
- uavcan_equipment_gnss_Auxiliary
- ardupilot_gnss_Heading
- ardupilot_gnss_Status
- ardupilot_gnss_MovingBaselineData
- ardupilot_gnss_RelPosHeading

LED

- AP_Notify/DroneCAN_RGB_LED
- outgoing only

OpticalFlow

- AP_OpticalFlow/AP_OpticalFlow_HereFlow
- com_hex_equipment_flow_Measurement

Proximity

- AP_Proximity/AP_Proximity_DroneCAN
- ardupilot_equipment_proximity_sensor_Proximity

Rangefinder

- AP_Rangefinder/AP_Rangefinder_DroneCAN
- uavcan_equipment_range_sensor_Measurement

RCProtocol

- AP_RCProtocol/AP_RCProtocol_DroneCAN
- dronecan_sensors_rc_RCInput

RPM

- AP_RPM/AP_RPM_DroneCAN
- dronecan_sensors_rpm_RPM

Temperature

- AP_TemperatureSensor/AP_TemperatureSensor_DroneCAN
- uavcan_equipment_device_Temperature


Buzzer

Safety Switch

DroneCAN Adapter Node

ESC

Servo

General message handlers

- AP_DroneCAN/AP_DroneCAN

Publishers

- uavcan_equipment_indication_LightsCommand
- uavcan_equipment_indication_BeepCommand
- uavcan_equipment_gnss_RTCMStream
- com_xacti_CopterAttStatus
- com_xacti_GimbalControlData
- com_xacti_GnssStatus
- uavcan_equipment_hardpoint_Command
- uavcan_protocol_NodeStatus
- dronecan_protocol_CanStats
- dronecan_protocol_Stats
- uavcan_equipment_actuator_ArrayCommand
- uavcan_equipment_esc_RawCommand
- ardupilot_indication_SafetyState
- uavcan_equipment_safety_ArmingStatus
- ardupilot_indication_NotifyState
- com_himark_servo_ServoCmd
- uavcan_equipment_gnss_Fix2
- uavcan_equipment_gnss_Auxiliary
- ardupilot_gnss_Heading
- ardupilot_gnss_Status
- com_hobbywing_esc_RawCommand
- com_hobbywing_esc_GetEscID

Subscribers

- ardupilot_indication_Button
- ardupilot_equipment_trafficmonitor_TrafficReport
- uavcan_equipment_actuator_Status
- uavcan_equipment_esc_Status
- uavcan_equipment_esc_StatusExtended
- uavcan_protocol_debug_LogMessage
- com_himark_servo_ServoInfo
- com_volz_servo_ActuatorStatus
- uavcan_protocol_param_GetSetResponse
- uavcan_protocol_param_ExecuteOpcodeResponse
- uavcan_protocol_RestartNodeResponse
- uavcan_protocol_GetNodeInfoRequest
- dronecan_protocol_FlexDebug
- com_hobbywing_esc_GetEscID
- com_hobbywing_esc_StatusMsg1
- com_hobbywing_esc_StatusMsg2


