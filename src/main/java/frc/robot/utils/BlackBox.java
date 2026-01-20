// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.utils; // Change to your package name

import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

public class BlackBox {
    public class DataRecorder {

        /**
         * Writes a comma-separated line of data to a file in the deploy directory.
         * 
         * @param fileName   The name of the file (e.g., "test_data.csv")
         * @param dataPoints Variable arguments of data to record (numbers, strings,
         *                   etc.)
         */
        public static void recordData(String fileName, Object... dataPoints) {
            // 1. Get the path to the deploy directory
            File deployDir = Filesystem.getDeployDirectory();
            File dataFile = new File(deployDir, fileName);

            // 2. Build the CSV string
            StringBuilder sb = new StringBuilder();

            // Optional: Add a timestamp as the first column
            String timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("HH:mm:ss.SSS"));
            sb.append(timestamp).append(",");

            for (int i = 0; i < dataPoints.length; i++) {
                sb.append(dataPoints[i]);
                // Add comma if not the last item
                if (i < dataPoints.length - 1) {
                    sb.append(",");
                }
            }
            // Add a new line at the end
            sb.append("\n");

            // 3. Write to file using try-with-resources (automatically closes file)
            try (BufferedWriter writer = new BufferedWriter(new FileWriter(dataFile, true))) {
                writer.write(sb.toString());
                System.out.println("Data saved to: " + dataFile.getAbsolutePath());
            } catch (IOException e) {
                System.err.println("Failed to write data to deploy directory!");
                e.printStackTrace();
            }
        }
    }
}