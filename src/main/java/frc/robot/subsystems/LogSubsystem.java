package frc.robot.subsystems; 
 
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
 
import edu.wpi.first.wpilibj.DriverStation;
 
public class LogSubsystem {
    public String m_subsystemName;
    private static LogSubsystem INSTANCE = new LogSubsystem();
    public String m_level;
    public static final String RED = "\033[0;31m";     // RED
    public static final String BLUE = "\033[0;34m";    // BLUE
    
    public static LogSubsystem getInstance() {
        return INSTANCE;
    }
 
 
    public enum Level {
        kRobot, kInfo, kWarning, kLibrary
    }
    private final List<LogSource> dataSources = new ArrayList<>();
    private Path file;
 
    private String loggingLocation = "/home/lvuser/logs/";
 
    private LogSubsystem() {
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
                file = Paths.get(loggingLocation + "_" + m_subsystemName + DriverStation.getInstance().getEventName()
                        + "_" + DriverStation.getInstance().getMatchType()
                        + DriverStation.getInstance().getMatchNumber() + ".csv");
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
 
    public void addSource(String name, String subsystemName, Supplier<Object> supplier) {
    
        dataSources.add(new LogSource(name,  subsystemName,supplier));
    }
 
    public void saveLogs() {
        try {
            if (file == null) {
                createFile();
            }
 
            StringBuilder data = new StringBuilder();
            data.append(m_level);
            data.append(Instant.now().toString()).append(",");
            data.append(DriverStation.getInstance().getMatchTime()).append(",");
            data.append(getValues());
            Files.write(file, Collections.singletonList(data.toString()), StandardOpenOption.APPEND);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
 
    private void saveTitles() throws IOException {
        StringBuilder titles = new StringBuilder();
        titles.append("Level");
        titles.append("Timestamp,");
        titles.append("match_time,");
        titles.append(dataSources.stream().map(t -> t.name).collect(Collectors.joining(","))).append(",");
        Files.write(file, Collections.singletonList(titles.toString()), StandardOpenOption.APPEND);
    }
 
    private String getValues() {
        return dataSources.stream().map(s -> s.supplier.get()).map(Object::toString).collect(Collectors.joining(","));
    }
    
    public String setLevel(Level level){
        
 
        // Turn enum level into string
        switch (level) {
        case kInfo:
            m_level = "INFO: ";
            break;
        case kWarning:
             m_level = "WARNING: ";
            break;
        case kRobot:
             m_level = "ROBOT: ";
            break;
        case kLibrary:
            m_level = "LIBRARY: ";
            break;
        default:
            m_level = "UNK: ";
            break;
        }
        
        return m_level;
    }
    private class LogSource {
        private final String name;
        private final Supplier<Object> supplier;
        
        public LogSource(String name, String subsystemName,Supplier<Object> supplier) {
            this.name = name;
            this.supplier = supplier;
            m_subsystemName = subsystemName;
        }
   }
   
   
}
    
