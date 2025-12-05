package io.dronefleet.mavlink.serialization.payload;

public interface MavlinkPayloadDeserializer {
    <T> T deserialize(int messageId, byte[] payload, Class<T> messageType);
}
