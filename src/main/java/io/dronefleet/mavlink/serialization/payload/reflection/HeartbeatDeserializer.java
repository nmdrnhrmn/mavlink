package io.dronefleet.mavlink.serialization.payload.reflection;

import io.dronefleet.mavlink.minimal.Heartbeat;
import io.dronefleet.mavlink.minimal.MavType;
import io.dronefleet.mavlink.minimal.MavAutopilot;
import io.dronefleet.mavlink.minimal.MavModeFlag;
import io.dronefleet.mavlink.minimal.MavState;
import io.dronefleet.mavlink.util.EnumValue;

public class HeartbeatDeserializer {

    private static final MavType[] MAV_TYPE_LOOKUP = {
        MavType.MAV_TYPE_GENERIC,           // 0
        MavType.MAV_TYPE_FIXED_WING,        // 1
        MavType.MAV_TYPE_QUADROTOR,         // 2
        MavType.MAV_TYPE_COAXIAL,           // 3
        MavType.MAV_TYPE_HELICOPTER,        // 4
        MavType.MAV_TYPE_ANTENNA_TRACKER,   // 5
        MavType.MAV_TYPE_GCS,               // 6
        MavType.MAV_TYPE_AIRSHIP,           // 7
        MavType.MAV_TYPE_FREE_BALLOON,      // 8
        MavType.MAV_TYPE_ROCKET,            // 9
        MavType.MAV_TYPE_GROUND_ROVER,      // 10
        MavType.MAV_TYPE_SURFACE_BOAT,      // 11
        MavType.MAV_TYPE_SUBMARINE,         // 12
        MavType.MAV_TYPE_HEXAROTOR,         // 13
        MavType.MAV_TYPE_OCTOROTOR,         // 14
        MavType.MAV_TYPE_TRICOPTER,         // 15
        MavType.MAV_TYPE_FLAPPING_WING,     // 16
        MavType.MAV_TYPE_KITE,              // 17
        MavType.MAV_TYPE_ONBOARD_CONTROLLER,// 18
        MavType.MAV_TYPE_VTOL_TAILSITTER_DUOROTOR,     // 19
        MavType.MAV_TYPE_VTOL_TAILSITTER_QUADROTOR,    // 20
        MavType.MAV_TYPE_VTOL_TILTROTOR,    // 21
        MavType.MAV_TYPE_VTOL_FIXEDROTOR,    // 22
        MavType.MAV_TYPE_VTOL_TAILSITTER,    // 23
        MavType.MAV_TYPE_VTOL_TILTWING,    // 24
        MavType.MAV_TYPE_VTOL_RESERVED5,    // 25
        MavType.MAV_TYPE_GIMBAL,            // 26
        MavType.MAV_TYPE_ADSB,              // 27
        MavType.MAV_TYPE_PARAFOIL,          // 28
        MavType.MAV_TYPE_DODECAROTOR,       // 29
        MavType.MAV_TYPE_CAMERA,            // 30
        MavType.MAV_TYPE_CHARGING_STATION,  // 31
        MavType.MAV_TYPE_FLARM,             // 32
        MavType.MAV_TYPE_SERVO,             // 33
        MavType.MAV_TYPE_ODID,              // 34
        MavType.MAV_TYPE_DECAROTOR,         // 35
        MavType.MAV_TYPE_BATTERY,           // 36
        MavType.MAV_TYPE_PARACHUTE,         // 37
        MavType.MAV_TYPE_LOG,               // 38
        MavType.MAV_TYPE_OSD,               // 39
        MavType.MAV_TYPE_IMU,               // 40
        MavType.MAV_TYPE_GPS,               // 41
        MavType.MAV_TYPE_WINCH              // 42
    };

    private static final MavAutopilot[] MAV_AUTOPILOT_LOOKUP = {
        MavAutopilot.MAV_AUTOPILOT_GENERIC,                                 // 0
        MavAutopilot.MAV_AUTOPILOT_RESERVED,                                // 1
        MavAutopilot.MAV_AUTOPILOT_SLUGS,                                   // 2
        MavAutopilot.MAV_AUTOPILOT_ARDUPILOTMEGA,                           // 3
        MavAutopilot.MAV_AUTOPILOT_OPENPILOT,                               // 4
        MavAutopilot.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY,                  // 5
        MavAutopilot.MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY, // 6
        MavAutopilot.MAV_AUTOPILOT_GENERIC_MISSION_FULL,                    // 7
        MavAutopilot.MAV_AUTOPILOT_INVALID,                                 // 8
        MavAutopilot.MAV_AUTOPILOT_PPZ,                                     // 9
        MavAutopilot.MAV_AUTOPILOT_UDB,                                     // 10
        MavAutopilot.MAV_AUTOPILOT_FP,                                      // 11
        MavAutopilot.MAV_AUTOPILOT_PX4,                                     // 12
        MavAutopilot.MAV_AUTOPILOT_SMACCMPILOT,                             // 13
        MavAutopilot.MAV_AUTOPILOT_AUTOQUAD,                                // 14
        MavAutopilot.MAV_AUTOPILOT_ARMAZILA,                                // 15
        MavAutopilot.MAV_AUTOPILOT_AEROB,                                   // 16
        MavAutopilot.MAV_AUTOPILOT_ASLUAV,                                  // 17
        MavAutopilot.MAV_AUTOPILOT_SMARTAP,                                 // 18
        MavAutopilot.MAV_AUTOPILOT_AIRRAILS,                                 // 19
        MavAutopilot.MAV_AUTOPILOT_REFLEX,                                // 20
    };

    private static final MavState[] MAV_STATE_LOOKUP = {
        MavState.MAV_STATE_UNINIT,              // 0
        MavState.MAV_STATE_BOOT,                // 1
        MavState.MAV_STATE_CALIBRATING,         // 2
        MavState.MAV_STATE_STANDBY,             // 3
        MavState.MAV_STATE_ACTIVE,              // 4
        MavState.MAV_STATE_CRITICAL,            // 5
        MavState.MAV_STATE_EMERGENCY,           // 6
        MavState.MAV_STATE_POWEROFF,            // 7
        MavState.MAV_STATE_FLIGHT_TERMINATION   // 8
    };

    public Heartbeat deserialize(byte[] payload) {
        if (payload == null || payload.length < 9) {
            return null;
        }

        long customMode = ((payload[0] & 0xFFL)) |
                         ((payload[1] & 0xFFL) << 8) |
                         ((payload[2] & 0xFFL) << 16) |
                         ((payload[3] & 0xFFL) << 24);
        
        int typeValue = payload[4] & 0xFF;
        EnumValue<MavType> type = (typeValue < MAV_TYPE_LOOKUP.length) 
            ? EnumValue.of(MAV_TYPE_LOOKUP[typeValue])
            : EnumValue.create(typeValue);
        
        int autopilotValue = payload[5] & 0xFF;
        EnumValue<MavAutopilot> autopilot = (autopilotValue < MAV_AUTOPILOT_LOOKUP.length)
            ? EnumValue.of(MAV_AUTOPILOT_LOOKUP[autopilotValue])
            : EnumValue.create(autopilotValue);
        
        int baseModeValue = payload[6] & 0xFF;
        EnumValue<MavModeFlag> baseMode = EnumValue.create(baseModeValue);
        
        int systemStatusValue = payload[7] & 0xFF;
        EnumValue<MavState> systemStatus = (systemStatusValue < MAV_STATE_LOOKUP.length)
            ? EnumValue.of(MAV_STATE_LOOKUP[systemStatusValue])
            : EnumValue.create(systemStatusValue);
        
        int mavlinkVersion = payload[8] & 0xFF;
        
        return Heartbeat.builder()
                .type(type)
                .autopilot(autopilot)
                .baseMode(baseMode)
                .customMode(customMode)
                .systemStatus(systemStatus)
                .mavlinkVersion(mavlinkVersion)
                .build();
    }
}
