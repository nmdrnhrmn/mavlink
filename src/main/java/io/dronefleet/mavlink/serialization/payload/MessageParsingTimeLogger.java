package io.dronefleet.mavlink.serialization.payload;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

public class MessageParsingTimeLogger {
    private static final long DUMP_INTERVAL_MS = 30000;
    private static final double PERCENT_OF_TOTAL_THRESHOLD_TO_BE_VISIBLE_IN_STATS = 5;

    private final Map<Integer, MessageIdExecutionStatisticsEntry> parsingStats = new HashMap<>();
    private long timeOfSnapshot = 0;
    private long enteredMethodTime = 0;

    public void recordStartParsingTimeAndDumpResults(Consumer<String> printFunction) {
        enteredMethodTime = System.currentTimeMillis();
        if (enteredMethodTime - timeOfSnapshot >= DUMP_INTERVAL_MS) {
            dumpParsingStatsToConsole(printFunction);
            timeOfSnapshot = enteredMethodTime;
            parsingStats.clear();
        }
    }

    public void endTiming(int messageId) {
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

    private void dumpParsingStatsToConsole(Consumer<String> printFunction) {
        if (parsingStats.isEmpty()) {
            printFunction.accept("No parsing statistics available.");
            return;
        }

        long totalMessages = parsingStats.values().stream()
            .mapToInt(MessageIdExecutionStatisticsEntry::getReceivedTimes)
            .sum();
        long totalTime = parsingStats.values().stream()
            .mapToLong(MessageIdExecutionStatisticsEntry::getCombinedParsingTime)
            .sum();

        StringBuilder sb = new StringBuilder();
        sb.append("\n=== BEGIN Mavlink Parsing Stats ===\n\n");

        parsingStats.entrySet().stream()
            .sorted((e1, e2) -> Long.compare(e2.getValue().getCombinedParsingTime(), e1.getValue().getCombinedParsingTime()))
            .forEach(entry -> {
                int messageId = entry.getKey();
                MessageIdExecutionStatisticsEntry stats = entry.getValue();
                long totalTimeForMsg = stats.getCombinedParsingTime();
                int count = stats.getReceivedTimes();
                double avgTime = (double) totalTimeForMsg / count;
                double percentOfTotal = (double) totalTimeForMsg / totalTime * 100;
                if (percentOfTotal >= PERCENT_OF_TOTAL_THRESHOLD_TO_BE_VISIBLE_IN_STATS) {
                    sb.append(String.format("ID %d, x%d msg, %dms (%.2f avg), %.1f%% of parsing time%n",
                        messageId, count, totalTimeForMsg, avgTime, percentOfTotal));
                }
            });

        long percentOfTotal = totalTime * 100 / DUMP_INTERVAL_MS;
        double avgTimeOverall = (double) totalTime / totalMessages;
        sb.append(String.format("\nTOTAL: x%d msg in %dms (%.2f avg), %d%% of total CPU time\n",
            totalMessages, totalTime, avgTimeOverall, percentOfTotal));
        sb.append("\n=== END Mavlink Parsing Stats ===\n");

        printFunction.accept(sb.toString());
    }
}
