package io.dronefleet.mavlink.serialization.payload;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

public class MessageTimer {
    private static final long DUMP_INTERVAL_MS = 30000;

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
        sb.append("\n╔═══════════════════════════════════════════════════════════════════════════════════════╗\n");
        sb.append("║                          MAVLink Parsing Statistics                                   ║\n");
        sb.append("╠════════════╦═══════════════╦═══════════════════╦═══════════════════════╦══════════════╣\n");
        sb.append("║ Message ID ║ Times Parsed  ║  Total Time (ms)  ║   Avg Time (ms)       ║   % of Time  ║\n");
        sb.append("╠════════════╬═══════════════╬═══════════════════╬═══════════════════════╬══════════════╣\n");

        parsingStats.entrySet().stream()
            .sorted((e1, e2) -> Long.compare(e2.getValue().getCombinedParsingTime(), e1.getValue().getCombinedParsingTime()))
            .forEach(entry -> {
                int messageId = entry.getKey();
                MessageIdExecutionStatisticsEntry stats = entry.getValue();
                long totalTimeForMsg = stats.getCombinedParsingTime();
                int count = stats.getReceivedTimes();
                double avgTime = (double) totalTimeForMsg / count;
                double percentOfTotal = (double) totalTimeForMsg / totalTime * 100;

                sb.append(String.format("║ %-10d ║ %-13d ║ %-17d ║ %-21.3f ║ %-12.1f ║%n",
                    messageId, count, totalTimeForMsg, avgTime, percentOfTotal));
            });

        double avgTimeOverall = (double) totalTime / totalMessages;
        sb.append("╠════════════╩═══════════════╩═══════════════════╩═══════════════════════╩══════════════╣\n");
        sb.append(String.format("║ TOTAL: %d messages parsed in %d ms (avg: %.3f ms/msg)                             ║%n",
            totalMessages, totalTime, avgTimeOverall));
        sb.append("╚═══════════════════════════════════════════════════════════════════════════════════════╝\n");

        printFunction.accept(sb.toString());
    }
}
