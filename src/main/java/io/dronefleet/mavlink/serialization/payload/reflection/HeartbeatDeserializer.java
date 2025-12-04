package io.dronefleet.mavlink.serialization.payload.reflection;

import io.dronefleet.mavlink.minimal.Heartbeat;
import io.dronefleet.mavlink.minimal.MavType;
import io.dronefleet.mavlink.minimal.MavAutopilot;
import io.dronefleet.mavlink.minimal.MavModeFlag;
import io.dronefleet.mavlink.minimal.MavState;
import io.dronefleet.mavlink.util.EnumValue;

public class HeartbeatDeserializer {
    public Heartbeat deserialize(byte[] payload) {
        if (payload == null || payload.length < 9) {
            return null;
        }

        int typeValue = payload[0] & 0xFF;
        EnumValue<MavType> type = EnumValue.create(typeValue);
        
        int autopilotValue = payload[1] & 0xFF;
        EnumValue<MavAutopilot> autopilot = EnumValue.create(autopilotValue);
        
        int baseModeValue = payload[2] & 0xFF;
        EnumValue<MavModeFlag> baseMode = EnumValue.create(baseModeValue);
        
        long customMode = ((payload[3] & 0xFFL)) |
                         ((payload[4] & 0xFFL) << 8) |
                         ((payload[5] & 0xFFL) << 16) |
                         ((payload[6] & 0xFFL) << 24);
        
        int systemStatusValue = payload[7] & 0xFF;
        EnumValue<MavState> systemStatus = EnumValue.create(systemStatusValue);
        
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
