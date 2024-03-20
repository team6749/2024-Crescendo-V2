// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightsSubsystem extends SubsystemBase {
    /** Creates a new LightsSubsystem. */
    public LightsSubsystem() {
    }

    private static Relay r2 = new Relay(Constants.LightsConstants.lightsOne); // SPK3 +red -green
    private static Relay r1 = new Relay(Constants.LightsConstants.lightsOne); // SPK2 +blue -common

    public DriverStation.Alliance getAlliance() { // TO TEST
        Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red || ally.get() == Alliance.Blue) {
                return ally.get();
            }
        }
        return null;
    }

    public void setAllianceColors() {
        if (getAlliance() == Alliance.Red) {
            Red();
        }
        if (getAlliance() == Alliance.Blue) {
            Blue();
        }
    }

    public void Off() {// works
        r1.set(Relay.Value.kOn);
        r2.set(Relay.Value.kOn);
    }

    public void White() { // works
        r1.set(Relay.Value.kReverse);
        r2.set(Relay.Value.kOff);
    }

    public void Red() {// works
        r1.set(Relay.Value.kOn);
        r2.set(Relay.Value.kReverse);
    }

    public void Blue() {// work
        r1.set(Relay.Value.kReverse);
        r2.set(Relay.Value.kOn);
    }

    public void Green() {// works
        r1.set(Relay.Value.kOn);
        r2.set(Relay.Value.kForward);
    }

    public void Yellow() {// works
        r1.set(Relay.Value.kOn);
        r2.set(Relay.Value.kOff);
    }

    public void Cyan() {// works
        r1.set(Relay.Value.kReverse);
        r2.set(Relay.Value.kForward);
    }

    public void Magenta() {// works
        r1.set(Relay.Value.kReverse);
        r2.set(Relay.Value.kReverse);
    }

    @Override
    public void periodic() {}

    public Command setLightsCommand() {
        return Commands.run(
        () -> Green(),
        this);
    }
}
