package io.dronefleet.mavlink.serialization.payload;

import java.util.function.Consumer;

public interface MavlinkPayloadDeserializer {
    <T> T deserialize(int messageId, byte[] payload, Class<T> messageType, Consumer<String> debugPrintFunction);
}
