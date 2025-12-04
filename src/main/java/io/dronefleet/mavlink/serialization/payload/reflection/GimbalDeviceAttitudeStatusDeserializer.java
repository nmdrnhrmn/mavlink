package io.dronefleet.mavlink.serialization.payload.reflection;

import io.dronefleet.mavlink.common.GimbalDeviceAttitudeStatus;
import io.dronefleet.mavlink.common.GimbalDeviceFlags;
import io.dronefleet.mavlink.common.GimbalDeviceErrorFlags;
import io.dronefleet.mavlink.util.EnumValue;

import java.util.Arrays;
import java.util.List;

public class GimbalDeviceAttitudeStatusDeserializer {
    public GimbalDeviceAttitudeStatus deserialize(byte[] payload) {
        if (payload == null || payload.length < 40) {
            return null;
        }

        int targetSystem = payload[0] & 0xFF;
        int targetComponent = payload[1] & 0xFF;
        
        long timeBootMs = ((payload[2] & 0xFFL)) |
                         ((payload[3] & 0xFFL) << 8) |
                         ((payload[4] & 0xFFL) << 16) |
                         ((payload[5] & 0xFFL) << 24);
        
        int flagsValue = ((payload[6] & 0xFF)) |
                        ((payload[7] & 0xFF) << 8);
        EnumValue<GimbalDeviceFlags> flags = EnumValue.create(flagsValue);
        
        float q0 = Float.intBitsToFloat(
            ((payload[8] & 0xFF)) |
            ((payload[9] & 0xFF) << 8) |
            ((payload[10] & 0xFF) << 16) |
            ((payload[11] & 0xFF) << 24));
        
        float q1 = Float.intBitsToFloat(
            ((payload[12] & 0xFF)) |
            ((payload[13] & 0xFF) << 8) |
            ((payload[14] & 0xFF) << 16) |
            ((payload[15] & 0xFF) << 24));
        
        float q2 = Float.intBitsToFloat(
            ((payload[16] & 0xFF)) |
            ((payload[17] & 0xFF) << 8) |
            ((payload[18] & 0xFF) << 16) |
            ((payload[19] & 0xFF) << 24));
        
        float q3 = Float.intBitsToFloat(
            ((payload[20] & 0xFF)) |
            (
                (payload[21] & 0xFF) << 8) |
            ((payload[22] & 0xFF) << 16) |
            ((payload[23] & 0xFF) << 24));
        
        List<Float> q = Arrays.asList(q0, q1, q2, q3);
        
        float angularVelocityX = Float.intBitsToFloat(
            ((payload[24] & 0xFF)) |
            ((payload[25] & 0xFF) << 8) |
            ((payload[26] & 0xFF) << 16) |
            ((payload[27] & 0xFF) << 24));
        
        float angularVelocityY = Float.intBitsToFloat(
            ((payload[28] & 0xFF)) |
            ((payload[29] & 0xFF) << 8) |
            ((payload[30] & 0xFF) << 16) |
            ((payload[31] & 0xFF) << 24));
        
        float angularVelocityZ = Float.intBitsToFloat(
            ((payload[32] & 0xFF)) |
            ((payload[33] & 0xFF) << 8) |
            ((payload[34] & 0xFF) << 16) |
            ((payload[35] & 0xFF) << 24));
        
        int failureFlagsValue = ((payload[36] & 0xFF)) |
                               ((payload[37] & 0xFF) << 8) |
                               ((payload[38] & 0xFF) << 16) |
                               ((payload[39] & 0xFF) << 24);
        EnumValue<GimbalDeviceErrorFlags> failureFlags = EnumValue.create(failureFlagsValue);
        
        float deltaYaw = 0.0f;
        float deltaYawVelocity = 0.0f;
        
        if (payload.length >= 44) {
            deltaYaw = Float.intBitsToFloat(
                ((payload[40] & 0xFF)) |
                ((payload[41] & 0xFF) << 8) |
                ((payload[42] & 0xFF) << 16) |
                ((payload[43] & 0xFF) << 24));
        }
        
        if (payload.length >= 48) {
            deltaYawVelocity = Float.intBitsToFloat(
                ((payload[44] & 0xFF)) |
                ((payload[45] & 0xFF) << 8) |
                ((payload[46] & 0xFF) << 16) |
                ((payload[47] & 0xFF) << 24));
        }
        
        return GimbalDeviceAttitudeStatus.builder()
                .targetSystem(targetSystem)
                .targetComponent(targetComponent)
                .timeBootMs(timeBootMs)
                .flags(flags)
                .q(q)
                .angularVelocityX(angularVelocityX)
                .angularVelocityY(angularVelocityY)
                .angularVelocityZ(angularVelocityZ)
                .failureFlags(failureFlags)
                .deltaYaw(deltaYaw)
                .deltaYawVelocity(deltaYawVelocity)
                .build();
    }
}
