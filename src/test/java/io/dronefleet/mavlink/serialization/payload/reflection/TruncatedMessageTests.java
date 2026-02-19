package io.dronefleet.mavlink.serialization.payload.reflection;

public class TruncatedMessageTests {
/*
    private final ReflectionPayloadDeserializer deserializer = new ReflectionPayloadDeserializer();

    @Test
    public void testTruncatedPayloadDeserialization() {
        byte[] payload = DatatypeConverter.parseHexBinary("ff");
        MissionAck message = deserializer.deserialize(0, payload, MissionAck.class);

        assertEquals(255, message.targetSystem());
        assertEquals(0, message.targetComponent());
        assertEquals(EnumValue.of(MavMissionResult.MAV_MISSION_ACCEPTED), message.type());
        assertEquals(EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION), message.missionType());
    }

    @Test
    public void testNonTruncatedPayloadDeserialization() {
        byte[] payload = DatatypeConverter.parseHexBinary("ff000000");
        MissionAck message = deserializer.deserialize(0, payload, MissionAck.class);

        assertEquals(255, message.targetSystem());
        assertEquals(0, message.targetComponent());
        assertEquals(EnumValue.of(MavMissionResult.MAV_MISSION_ACCEPTED), message.type());
        assertEquals(EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION), message.missionType());
    }*/
}
