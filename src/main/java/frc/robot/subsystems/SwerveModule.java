// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SwerveModule implements Sendable {
    /** Creates a new SwerveModule. */
    public String name;
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANcoder encoder;
    public final PIDController velocityPIDController = new PIDController(0.0, 0, 0);
    public final PIDController anglePIDController = new PIDController(0.1, 0, 0);
    public Translation2d location;

    public SwerveModule(String name,  int driveMotorPort, int angleMotorPort, int encoderPort, Translation2d locationFromCenter) {

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'initSendable'");
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    
}
