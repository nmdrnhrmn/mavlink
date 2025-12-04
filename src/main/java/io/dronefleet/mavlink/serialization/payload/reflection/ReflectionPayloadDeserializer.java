package io.dronefleet.mavlink.serialization.payload.reflection;

import io.dronefleet.mavlink.annotations.MavlinkFieldInfo;
import io.dronefleet.mavlink.annotations.MavlinkMessageBuilder;
import io.dronefleet.mavlink.annotations.MavlinkMessageInfo;
import io.dronefleet.mavlink.serialization.MavlinkSerializationException;
import io.dronefleet.mavlink.serialization.payload.MavlinkPayloadDeserializer;
import io.dronefleet.mavlink.util.EnumValue;
import io.dronefleet.mavlink.util.WireFieldInfoComparator;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.ParameterizedType;
import java.lang.reflect.Type;
import java.math.BigInteger;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;

public class ReflectionPayloadDeserializer implements MavlinkPayloadDeserializer {

    private static final WireFieldInfoComparator wireComparator = new WireFieldInfoComparator();

    private static final long MINUTE_IN_MILLIS = 6000;
    private static long timeOfSnapshot = 0;

    private static final Map<Integer, MessageIdExecutionStatisticsEntry> parsingStats = new HashMap<>();

    private void dumpParsingStatsToConsole() {
        if (parsingStats.isEmpty()) {
            System.out.println("No parsing statistics available.");
            return;
        }

        long totalMessages = parsingStats.values().stream()
            .mapToInt(MessageIdExecutionStatisticsEntry::getReceivedTimes)
            .sum();
        long totalTime = parsingStats.values().stream()
            .mapToLong(MessageIdExecutionStatisticsEntry::getCombinedParsingTime)
            .sum();

        System.out.println("\n╔═══════════════════════════════════════════════════════════════════════════════════════╗");
        System.out.println("║                          MAVLink Parsing Statistics                                   ║");
        System.out.println("╠════════════╦═══════════════╦═══════════════════╦═══════════════════════╦══════════════╣");
        System.out.println("║ Message ID ║ Times Parsed  ║  Total Time (ms)  ║   Avg Time (ms)       ║   % of Time  ║");
        System.out.println("╠════════════╬═══════════════╬═══════════════════╬═══════════════════════╬══════════════╣");

        parsingStats.entrySet().stream()
            .sorted((e1, e2) -> Long.compare(e2.getValue().getCombinedParsingTime(), e1.getValue().getCombinedParsingTime()))
            .forEach(entry -> {
                int messageId = entry.getKey();
                MessageIdExecutionStatisticsEntry stats = entry.getValue();
                long totalTimeForMsg = stats.getCombinedParsingTime();
                int count = stats.getReceivedTimes();
                double avgTime = (double) totalTimeForMsg / count;
                double percentOfTotal = (double) totalTimeForMsg / totalTime * 100;

                System.out.printf("║ %-10d ║ %-13d ║ %-17d ║ %-21.3f ║ %-12.1f ║%n", 
                    messageId, count, totalTimeForMsg, avgTime, percentOfTotal);
            });

        double avgTimeOverall = (double) totalTime / totalMessages;
        System.out.println("╠════════════╩═══════════════╩═══════════════════╩═══════════════════════╩══════════════╣");
        System.out.printf("║ TOTAL: %d messages parsed in %d ms (avg: %.3f ms/msg)                        ║%n", 
            totalMessages, totalTime, avgTimeOverall);
        System.out.println("╚═══════════════════════════════════════════════════════════════════════════════════════╝\n");
    }

    @Override
    public <T> T deserialize(int messageId, byte[] payload, Class<T> messageType) {
        long enteredMethodTime = System.currentTimeMillis();
        if (enteredMethodTime - timeOfSnapshot >= MINUTE_IN_MILLIS) {
            timeOfSnapshot = enteredMethodTime;
            dumpParsingStatsToConsole();
            parsingStats.clear();
        }
        MavlinkMessageInfo message = messageType.getAnnotation(MavlinkMessageInfo.class);
        if (message == null) {
            throw new IllegalArgumentException(String.format(
                "class %s is not annotated with @MavlinkMessageInfo", messageType.getName()));
        }

        try {
            Object builder = Arrays.stream(messageType.getMethods())
                .filter(m -> m.isAnnotationPresent(MavlinkMessageBuilder.class))
                .findFirst()
                .orElseThrow(() -> new MavlinkSerializationException(
                    "Message " + messageType.getName() + " does not have a builder"))
                .invoke(null);

            AtomicInteger nextOffset = new AtomicInteger();
            Arrays.stream(builder.getClass().getMethods())
                .filter(m -> m.isAnnotationPresent(MavlinkFieldInfo.class))
                .sorted((a, b) -> {
                    MavlinkFieldInfo fa = a.getAnnotation(MavlinkFieldInfo.class);
                    MavlinkFieldInfo fb = b.getAnnotation(MavlinkFieldInfo.class);
                    return wireComparator.compare(fa, fb);
                })
                .forEach(method -> {
                    MavlinkFieldInfo field = method.getAnnotation(MavlinkFieldInfo.class);

                    int length = field.unitSize() * Math.max(field.arraySize(), 1);
                    int offset = nextOffset.getAndAccumulate(length, (a, b) -> a + b);

                    byte[] data = new byte[length];
                    if (offset < payload.length) {
                        int copyLength = Math.max(
                            Math.min(length, payload.length - offset),
                            0
                        );
                        System.arraycopy(payload, offset, data, 0, copyLength);
                    }

                    Type fieldType = Optional.of(method.getGenericParameterTypes())
                        .filter(types -> types.length == 1)
                        .map(types -> types[0])
                        .orElseThrow(() -> new MavlinkSerializationException(
                            "Method " + method.getName() + " of " + builder.getClass().getName()
                                + " is annotated with @MavlinkFieldInfo, however does not " +
                                "accept a single parameter."));
                    try {
                        method.invoke(builder, deserialize(fieldType, data, 0, data.length, field));
                    } catch (IllegalAccessException | InvocationTargetException e) {
                        e.printStackTrace();
                    }
                });

            //noinspection unchecked
            return (T) builder.getClass().getMethod("build").invoke(builder);
        } catch (IllegalAccessException | InvocationTargetException | NoSuchMethodException e) {
            e.printStackTrace();
        } finally {
            long methodExecutionTime = System.currentTimeMillis() - enteredMethodTime;
            MessageIdExecutionStatisticsEntry stats;
            MessageIdExecutionStatisticsEntry currentStats = parsingStats.get(messageId);
            if (currentStats == null) {
                stats = new MessageIdExecutionStatisticsEntry(methodExecutionTime);
            } else {
                stats = currentStats.produceNextAddingParsingTime(methodExecutionTime);
            }
            parsingStats.put(messageId, stats);
        }
        return null;
    }

    private Object deserialize(Type fieldType, byte[] data, int offset, int length, MavlinkFieldInfo field) {
        if (fieldType instanceof Class) {
            Class fieldClass = (Class<?>) fieldType;
            if (int.class.isAssignableFrom(fieldClass) || Integer.class.isAssignableFrom(fieldClass)) {
                return (int) integerValue(data, offset, length, field.signed());
            } else if (long.class.isAssignableFrom(fieldClass) || Long.class.isAssignableFrom(fieldClass)) {
                return integerValue(data, offset, length, field.signed());
            } else if (float.class.isAssignableFrom(fieldClass) || Float.class.isAssignableFrom(fieldClass)) {
                return floatValue(data, offset);
            } else if (double.class.isAssignableFrom(fieldClass) || Double.class.isAssignableFrom(fieldClass)) {
                return doubleValue(data, offset);
            } else if (String.class.isAssignableFrom(fieldClass)) {
                return stringValue(data);
            } else if (byte[].class.isAssignableFrom(fieldClass)) {
                return data;
            } else if (BigInteger.class.isAssignableFrom(fieldClass)) {
                return bigIntValue(data);
            }
        } else if (fieldType instanceof ParameterizedType) {
            ParameterizedType parameterizedType = (ParameterizedType) fieldType;
            Class<?> fieldClass = (Class<?>) parameterizedType.getRawType();
            if (EnumValue.class.isAssignableFrom(fieldClass)) {
                return enumValue(field.enumType(), data, offset, length, field.signed());
            } else if (List.class.isAssignableFrom(fieldClass)) {
                return listValue((Class<?>) parameterizedType.getActualTypeArguments()[0], data, field);
            }
        }
        return null;
    }

    private long unsignedIntegerValue(byte[] data, int offset, int length) {
        long value = 0;
        for (int i = offset; i < Math.min(data.length, length); i++) {
            value = value | ((long) ((data[i] & 0xff)) << ((i - offset) * 8));
        }
        return value;
    }

    private long signedIntegerValue(byte[] data, int offset, int length) {
        long value = unsignedIntegerValue(data, offset, length);
        int signBitIndex = (length - offset) * Byte.SIZE - 1;
        if ((value >> signBitIndex) == 1) {
            value |= (-1L << signBitIndex);
        }
        return value;
    }

    private long integerValue(byte[] data, int offset, int length, boolean signed) {
        if (signed) {
            return signedIntegerValue(data, offset, length);
        } else {
            return unsignedIntegerValue(data, offset, length);
        }
    }

    private double doubleValue(byte[] data, int offset) {
        return ByteBuffer.wrap(data)
            .order(ByteOrder.LITTLE_ENDIAN)
            .getDouble(offset);
    }

    private float floatValue(byte[] data, int offset) {
        return ByteBuffer.wrap(data)
            .order(ByteOrder.LITTLE_ENDIAN)
            .getFloat(offset);
    }

    private String stringValue(byte[] data) {
        for (int i = 0; i < data.length; i++) {
            if (data[i] == 0) {
                return new String(data, 0, i, StandardCharsets.UTF_8);
            }
        }
        return new String(data, StandardCharsets.UTF_8);
    }


    private BigInteger bigIntValue(byte[] data) {
        // Invert to big-endian, for BigInteger constructor
        for (int i = 0; i < data.length / 2; ++i) {
            byte tmp = data[i];
            data[i] = data[data.length - 1 - i];
            data[data.length - 1 - i] = tmp;
        }

        return new BigInteger(data);
    }

    private Object enumValue(Class<?> enumType, byte[] data, int offset, int length, boolean signed) {
        return EnumValue.create(
            (Class<? extends Enum>) enumType,
            (int) integerValue(data, offset, length, signed));
    }

    private List<?> listValue(Class<?> listType, byte[] data, MavlinkFieldInfo field) {
        int unitSize = field.unitSize();
        List<Object> result = new ArrayList<>(data.length / unitSize);
        for (int offset = 0; offset < data.length; offset += unitSize) {
            Object value = deserialize(listType, data, offset, offset + unitSize, field);
            result.add(value);
        }
        return Collections.unmodifiableList(result);
    }
}

class MessageIdExecutionStatisticsEntry {
    private final int receivedTimes;
    private final long combinedParsingTime;

    public MessageIdExecutionStatisticsEntry(int receivedTimes, long combinedParsingTime) {
        this.receivedTimes = receivedTimes;
        this.combinedParsingTime = combinedParsingTime;
    }

    public MessageIdExecutionStatisticsEntry(long timeOfFirstParsing) {
        this.receivedTimes = 1;
        this.combinedParsingTime = timeOfFirstParsing;
    }

    public MessageIdExecutionStatisticsEntry produceNextAddingParsingTime(long parsingTime) {
        return new MessageIdExecutionStatisticsEntry(
            receivedTimes + 1,
            combinedParsingTime + parsingTime
        );
    }

    public int getReceivedTimes() {
        return receivedTimes;
    }

    public long getCombinedParsingTime() {
        return combinedParsingTime;
    }
}