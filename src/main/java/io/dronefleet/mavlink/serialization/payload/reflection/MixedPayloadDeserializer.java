package io.dronefleet.mavlink.serialization.payload.reflection;

import io.dronefleet.mavlink.serialization.payload.MavlinkPayloadDeserializer;

public class MixedPayloadDeserializer implements MavlinkPayloadDeserializer {
    private final HeartbeatDeserializer heartbeatDeserializer = new HeartbeatDeserializer();
    private final GimbalDeviceAttitudeStatusDeserializer gimbalDeviceAttitudeStatusDeserializer = new GimbalDeviceAttitudeStatusDeserializer();
    private final MavlinkPayloadDeserializer reflectionDeserializer = new ReflectionPayloadDeserializer();

    @Override
    public <T> T deserialize(int messageId, byte[] payload, Class<T> messageType) {
        // These two messages consumed up to 70% of computation time in total.
        // Handling them separately to improve efficiency.
        if (isHeartbeatMessage(messageId)) {
            try {
                return (T) heartbeatDeserializer.deserialize(payload);
            } catch (ClassCastException e) {
                return null;
            }
        }
        if (isGimbalDeviceAttitudeStatusMessage(messageId)) {
            try {
                return (T) gimbalDeviceAttitudeStatusDeserializer.deserialize(payload);
            } catch (ClassCastException e) {
                return null;
            }
        }
        return reflectionDeserializer.deserialize(messageId, payload, messageType);
    }

    private boolean isHeartbeatMessage(int messageId) {
        return messageId == HEARTBEAT_MESSAGE_ID;
    }

    private boolean isGimbalDeviceAttitudeStatusMessage(int messageId) {
        return messageId == GIMBAL_ATTITUDE_MESSAGE_ID;
    }

    private static final int HEARTBEAT_MESSAGE_ID = 0;
    private static final int GIMBAL_ATTITUDE_MESSAGE_ID = 285;
}
