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

    private static Relay r1 = new Relay(Constants.ElectronicsPorts.lightsOne); // SPK2 +blue -common
    private static Relay r2 = new Relay(Constants.ElectronicsPorts.lightsTwo); // SPK3 +red -green

    public DriverStation.Alliance getAlliance() { // TO TEST
        Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red || ally.get() == Alliance.Blue) {
                return ally.get();
            }
        }
        return null;
    }

    // public void setAllianceColors() {
    //     if (getAlliance() == Alliance.Red) {
    //         Red();
    //     }
    //     if (getAlliance() == Alliance.Blue) {
    //         Blue();
    //     }
    // }

    public void Off(){
        r1.set(Relay.Value.kOff);
        r2.set(Relay.Value.kOff);
    }
    public void Cyan(){
        r1.set(Relay.Value.kForward);
        r2.set(Relay.Value.kForward);
    }
    public void Blue(){
        r1.set(Relay.Value.kReverse);
        r2.set(Relay.Value.kReverse);
    }
    public void Test(Relay.Value val1, Relay.Value val2) {
        r1.set(val1);
        r2.set(val2);
    }

    @Override
    public void periodic() {}

    public Command setLightsCommand(Relay.Value value1, Relay.Value value2) {
        return Commands.run(
        () -> Test(value1, value2),
        this);
    }

    //RED/GREEN ON 2, BLUE/COMMON ON 3
    //ports 2, 3
    //off r1-on, r2-off
    //red r1-on, r2-on
    //off r1-on, r2-reverse
    //magenta r1-on, r2-forward

    //off r1-off, r2-off
    //yellow r1-off, r2-on
    //off r1-off, r2-reverse
    //white r1-off, r2-forward

    //off r1-reverse, r2-off
    //yellow r1-reverse, r2-on
    //off r1-reverse, r2-reverse
    //white r1-reverse, r2-forward

    //off r1-forward, r2-off
    //red r1-forward, r2-on
    //off r1-forward, r2-reverse
    //magenta r1-forward, r2-forward


    //ports 3, 2
    //yellow r1-on, r2-off
    //red r1-on, r2-on
    //yellow r1-on, r2-reverse
    //red r1-on, r2-forward

    //off r1-off, r2-off
    //off r1-off, r2-on
    //off r1-off, r2-reverse
    //off r1-off, r2-forward

    //off r1-reverse, r2-off
    //off r1-reverse, r2-on
    //off r1-reverse, r2-reverse
    //off r1-reverse, r2-forward

    //white r1-forward, r2-off
    //magenta r1-forward, r2-on
    //white r1-forward, r2-reverse
    //magenta r1-forward, r2-forward


    //RED/GREEN ON 2, BLUE/COMMON ON 3 WIRES ON SPIKES FOR LIGHTS WERE FLIPPED
    //ports 3, 2
    //yellow r1-on, r2-off
    //green r1-on, r2-on
    //yellow r1-on, r2-reverse
    //green r1-on, r2-forward

    //off r1-off, r2-off
    //off r1-off, r2-on
    //off r1-off, r2-reverse
    //off r1-off, r2-forward

    //white r1-reverse, r2-off
    //cyan r1-reverse, r2-on
    //white r1-reverse, r2-reverse
    //cyan r1-reverse, r2-forward

    //off r1-forward, r2-off
    //off r1-forward, r2-on
    //off r1-forward, r2-reverse
    //off r1-forward, r2-forward

    //ports 2, 3
    //off r1-on, r2-off
    //green r1-on, r2-on
    //cyan r1-on, r2-reverse
    //off r1-on, r2-forward

    //off r1-off, r2-off
    //yellow r1-off, r2-on
    //white r1-off, r2-reverse
    //off r1-off, r2-forward

    //off r1-reverse, r2-off
    //yellow r1-reverse, r2-on
    //white r1-reverse, r2-reverse
    //off r1-reverse, r2-forward

    //off r1-forward, r2-off
    //green r1-forward, r2-on
    //cyan r1-forward, r2-reverse
    //off r1-forward, r2-forward
}
