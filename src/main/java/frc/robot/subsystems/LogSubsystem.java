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
    private String loggingLocation = "C:\\Users\\grika\\OneDrive\\Belgeler\\simulateFile\\OUTPUT";
   // TODO: set location of saved file


    public static LogSubsystem getInstance() {
        return INSTANCE;
    }
 
 
    private final List<LogSource> dataSources = new ArrayList<>();
    private final List<LevelSource> levelData = new ArrayList<>();
    private Path file;

    private LogSubsystem() {
        
    
         
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
                file = Paths.get( loggingLocation + "_" + m_subsystemName + DriverStation.getInstance().getEventName()
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
 
    public void addSource( String level,String name, String subsystemName, Supplier<Object> supplier) {
    
        dataSources.add(new LogSource( name,  subsystemName,supplier));
        levelData.add(new LevelSource(level));

    }
 
  


    public void saveLogs() {
        try {
            if (file == null) {
                createFile();
            }
 
            StringBuilder data = new StringBuilder();
            data.append(Instant.now().toString()).append(",");
            
            data.append(DriverStation.getInstance().getMatchTime()).append(",");
            data.append(getValues()).append(",");
            data.append(getLevel());
            Files.write(file, Collections.singletonList(data.toString()), StandardOpenOption.APPEND);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
 
    private void saveTitles() throws IOException {
        StringBuilder titles = new StringBuilder();
        titles.append("Timestamp,");
        titles.append("match_time,");
        titles.append(dataSources.stream().map(t -> t.name).collect(Collectors.joining(","))).append(",");
        titles.append(dataSources.stream().map(N -> N.name+"_LEVEL").collect(Collectors.joining(","))).append(",");
        Files.write(file, Collections.singletonList(titles.toString()), StandardOpenOption.APPEND);
    }
 
    private String getValues() {
        return dataSources.stream().map(s -> s.supplier.get()).map(Object::toString).collect(Collectors.joining(","));
    }
    
    public final  String getLevel() {

        return (levelData.stream().map(l -> l.level).collect(Collectors.joining(",")));
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
   
       
    public  class LevelSource {

        public  final String level;
    
        public LevelSource( String level) {
            
            this.level=level;
          

      
        }
}
}