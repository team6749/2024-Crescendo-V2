// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveDrivebase;
import frc.robot.subsystems.SwerveModule;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kTopButtonBoard = 1;
        public static final int kBottomButtonBoard = 2;
    }

    public static class FieldConstants {
        public static Translation2d targetPosition2d = new Translation2d(0.35, 5.55); // in terms of x and y on the
                                                                                      // field
        public static Translation3d targetPosition3d = new Translation3d(targetPosition2d.getX(),
                targetPosition2d.getY(), 2.1082); // height of speaker opening meters
        public static Pose3d targetPosition3dWithAngle = new Pose3d(targetPosition3d,
                new Rotation3d(0.0, Units.degreesToRadians(-45), 0.0));
    }

    public static class ElectronicsPorts {
        public static final int frontLeftDrive = 1;
        public static final int frontLeftEncoder = 2;
        public static final int frontLeftAngle = 3;
        public static final int frontRightDrive = 4;
        public static final int frontRightEncoder = 5;
        public static final int frontRightAngle = 6;
        public static final int backRightDrive = 7;
        public static final int backRightEncoder = 8;
        public static final int backRightAngle = 9;
        public static final int backLeftDrive = 10;
        public static final int backLeftEncoder = 11;
        public static final int backLeftAngle = 12;

        public static final int indexerSpark = 15;
        public static final int indexerSwitch = 0;

        // public static final int intakeMotor = 18;
        // public static final int intakeSwitch = 1;

        public static final int intakePivot = 16;

        public static final int topShooterMotorPort = 13;
        public static final int bottomShooterMotorPort = 14;
    }

    public static class SwerveConstants {
        public static double chassisWidthMetersNoBumpers = .6858d;
        public static double chassisLengthMetersNoBumpers = .6858d;
        public static double swerveWheelDiameterMeters = .1d;
        public static double swerveGearRatio = 8.14d;
        public static double distFromCenterXMeters = .2525d;
        public static double distFromCenterYMeters = .2525d; // 68.5/2 - 9 in actual (close enuf)

        // same as 2024 v1 constants
        public static SwerveModule flModule = new SwerveModule(
                "Front Left",
                Constants.ElectronicsPorts.frontLeftDrive,
                Constants.ElectronicsPorts.frontLeftEncoder,
                Constants.ElectronicsPorts.frontLeftAngle,
                new Translation2d(distFromCenterXMeters, distFromCenterYMeters));

        public static SwerveModule frModule = new SwerveModule(
                "Front Right",
                Constants.ElectronicsPorts.frontRightDrive,
                Constants.ElectronicsPorts.frontRightEncoder,
                Constants.ElectronicsPorts.frontRightAngle,
                new Translation2d(distFromCenterXMeters, -distFromCenterYMeters));
        public static SwerveModule brModule = new SwerveModule(
                "Back Right",
                Constants.ElectronicsPorts.backRightDrive,
                Constants.ElectronicsPorts.backRightEncoder,
                Constants.ElectronicsPorts.backRightAngle,
                new Translation2d(-distFromCenterXMeters, -distFromCenterYMeters));
        public static SwerveModule blModule = new SwerveModule(
                "Back Left",
                Constants.ElectronicsPorts.backLeftDrive,
                Constants.ElectronicsPorts.backLeftEncoder,
                Constants.ElectronicsPorts.backLeftAngle,
                new Translation2d(-distFromCenterXMeters, distFromCenterYMeters));
        public static SwerveModule[] swerveModuleArray = { flModule, frModule, brModule, blModule };

    }

    public static class NoteDetectionConstants {
        public static double screenWidthPixels;
        public static double screenHeightPixels;
        
    }
    public static final int one = 1;
}
