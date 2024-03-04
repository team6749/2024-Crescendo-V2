// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.notemodel.Note;
import frc.robot.notemodel.VisionJSON;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.json.*;

public class NoteDetection extends SubsystemBase {

    public List<Note> rememberedNotes = new ArrayList<Note>();

    int nextId = 0;

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

            VisionJSON visionResults = mapper.readValue(json, VisionJSON.class);

            // JsonNode root = mapper.readTree("{ \"name\": \"Joe\", \"age\": 13 }");

            // root.withArray("/Results/Detector");

            List<Note> detectedNotes = visionResults.results.notes;

            List<Note> matchedNotes = new ArrayList<Note>();

            for (Note rememberedNote : rememberedNotes) {
                Note closest = rememberedNote.findClosestNote(detectedNotes);

                if (closest.distanceToNote(rememberedNote) > 0.1) {
                    // this note is too far away from all remembered notes, it must be a new one
                    continue;
                }

                closest.id = rememberedNote.id; // replace the id of the new note with the old one
                detectedNotes.remove(closest); // remove this note from the detected notes because we claimed it
                matchedNotes.add(closest);
            }
            for (Note note : detectedNotes) {
                // all newly detected notes will not have a unique id (basically)
                note.id = nextId;
                nextId++;
                matchedNotes.add(note);
            }

            // NOW WE CAN USE THE NOTES

            rememberedNotes = matchedNotes; // For the next frame, set our matched notes
        } catch (Exception e) {
            // TODO: handle exception
        }
    }
}
