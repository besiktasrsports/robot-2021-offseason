package frc.sneakylib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.File;
import java.io.IOException;
import java.io.Writer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public class SneakyLogger {
    private static SneakyLogger INSTANCE = new SneakyLogger();
    private static String loggingLocation;

    public static SneakyLogger getInstance() {

        if (RobotBase.isReal() == true) {
            loggingLocation = "/home/lvuser/logs/";
        } else {
            loggingLocation = "C:/Users/Emre/Desktop/";
        }

        return INSTANCE;
    }

    private final List<LogSource> dataSources = new ArrayList<>();
    private Path file;

    private SneakyLogger() {

        File usb1 = new File("/media/sda1/");
        if (usb1.exists()) {
            loggingLocation = "/media/sda1/logs/";
        }
    }

    private void createLogDirectory() throws IOException {
        File logDirectory = new File(loggingLocation);
        if (!logDirectory.exists()) {
            Files.createDirectory(Paths.get(loggingLocation));
        }
    }

    private void createFile() {
        Writer output = null;
        try {
            createLogDirectory();
            if (DriverStation.getInstance().isFMSAttached()) {
                file =
                        Paths.get(
                                loggingLocation
                                        + DriverStation.getInstance().getEventName()
                                        + "_"
                                        + DriverStation.getInstance().getMatchType()
                                        + DriverStation.getInstance().getMatchNumber()
                                        + ".csv");
            } else {
                file = Paths.get(loggingLocation + "test.csv");
            }
            if (Files.exists(file)) {
                Files.delete(file);
            }
            Files.createFile(file);
            saveTitles();
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            if (output != null) {
                try {
                    output.close();
                } catch (IOException e) {
                }
            }
        }
    }

    public void addSource(String name, String level, Supplier<Object> supplier) {
        dataSources.add(new LogSource(name, level, supplier));
    }

    public void saveLogs() {
        try {
            if (file == null) {
                createFile();
            }

            StringBuilder data = new StringBuilder();
            data.append(Instant.now().toString()).append(",");
            data.append(DriverStation.getInstance().getMatchTime()).append(",");
            data.append(getLevels());
            data.append(getValues());

            Files.write(file, Collections.singletonList(data.toString()), StandardOpenOption.APPEND);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void saveTitles() throws IOException {
        StringBuilder titles = new StringBuilder();
        titles.append("Timestamp,");
        titles.append("match_time,");
        titles
                .append(dataSources.stream().map(t -> t.name).collect(Collectors.joining(",")))
                .append(",");
        Files.write(file, Collections.singletonList(titles.toString()), StandardOpenOption.APPEND);
    }

    private String getLevels() {
        return ("[" + dataSources.stream().map(l -> l.level).collect(Collectors.joining(",")) + "] ");
    }

    private String getValues() {
        return dataSources.stream()
                .map(s -> s.supplier.get())
                .map(Object::toString)
                .collect(Collectors.joining(","));
    }

    private class LogSource {
        private final String name;
        private final String level;
        private final Supplier<Object> supplier;

        public LogSource(String name, String level, Supplier<Object> supplier) {
            this.name = name;
            this.level = level;
            this.supplier = supplier;
        }
    }
}
