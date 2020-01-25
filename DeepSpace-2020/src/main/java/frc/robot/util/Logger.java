package frc.robot.util;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.RobotController;

public class Logger {
    FileWriter fw;
    Map<String, Object> entries = new HashMap<>();
    boolean firstWrite = true;
    String subSystem;

    public Logger(String subSystem) {
        this.subSystem = subSystem;
    }

    public void clear() {
        entries = new HashMap<>();
        firstWrite = true;
        String path = "/home/lvuser/" + subSystem + RobotController.getFPGATime() + ".csv";
        try {
            File f = new File(path);
            if (f.exists()) {
                f.delete();
            }
            fw = new FileWriter(f, true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void entry(String key, Object value) {
        entries.put(key, value);
    }

    public void writeEntries() {
        if (firstWrite) {
            String titles = entries.keySet().stream()
                .sorted().collect(Collectors.joining(", "));
            try {
                System.out.println(titles);
                fw.write(titles);
                fw.write("\r\n");
            } catch (IOException e) {
                e.printStackTrace();
            }
            firstWrite = false;
        }
        String values = entries.keySet().stream()
            .sorted()
            .map(k -> entries.get(k))
            .map(Object::toString)
            .collect(Collectors.joining(", "));
        try {
            System.out.println(values);
            fw.write(values);
            fw.write("\r\n");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void closeFileWriter() {
        try {
            fw.flush();
            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
}