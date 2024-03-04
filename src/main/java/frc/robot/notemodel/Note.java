// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.notemodel;

import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.Idle;

/** Add your docs here. */
public class Note {

    static int cameraX = 640;
    static int cameraY = 960;
    static double cameraFOV = 62.5;

    public int id;

    public double txp;
    public double typ;

    public Note (int id, double txp, double typ) {
        this.id = id;
        this.txp = txp;
        this.typ = typ;
    }

    public double screenXPos() {
        return txp / cameraX;
    }

    public double screenYPos() {
        return typ / cameraY;
    }

    // robot relative note angle
    public double noteXAngle () {
        return -((screenXPos() * cameraFOV) - (cameraFOV/2d));
    }

    public double noteYAngle () {
        return -((screenYPos() * cameraFOV) - (cameraFOV/2d));
    }

    // returns the closest note to this note
    public Note findClosestNote(List<Note> notes) {
        if (notes.isEmpty()) {
            return null;
        }
        Note closestNote = notes.get(0);
        for (Note note : notes) {
            if (note.distanceToNote(this) < closestNote.distanceToNote(this)) {
                closestNote = note;
            }
        }
        return closestNote;
    }
    
    // returns distance of note to other in screen space (0,0) (1,1)
    public double distanceToNote(Note other) {
        return Math
                .sqrt(Math.pow(other.screenXPos() - screenXPos(), 2) + Math.pow(other.screenYPos() - screenYPos(), 2));
    }
}
