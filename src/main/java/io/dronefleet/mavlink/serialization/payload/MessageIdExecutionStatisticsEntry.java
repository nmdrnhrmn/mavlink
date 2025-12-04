package io.dronefleet.mavlink.serialization.payload;

public class MessageIdExecutionStatisticsEntry {
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
