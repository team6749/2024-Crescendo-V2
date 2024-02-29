// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.json.*;

public class NoteDetection extends SubsystemBase {
    /** Creates a new NoteDetection. */
    public NoteDetection() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        try {
            
            NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

            String json = table.getEntry("json").getString(null);

            if (json == null) {
                return; // prevent json parser from killing itself

            }
            ObjectMapper mapper = new ObjectMapper();

            JsonNode root = mapper.readTree(json);

            JsonNode results = root.("/Results/Detector");
            

            

        } catch (Exception e) {
            // TODO: handle exception
        }
    }
}
