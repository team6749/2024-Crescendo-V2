// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    }

    public static class MotorPorts {
        public static final int frontLeftDrive = 0;
        public static final int frontLeftAngle = 0;
        public static final int frontLeftEncoder = 0;
        public static final int frontRightDrive = 0;
        public static final int frontRightAngle = 0;
        public static final int frontRightEncoder = 0;
        public static final int backLeftDrive = 0;
        public static final int backLeftAngle = 0;
        public static final int backLeftEncoder = 0;
        public static final int backRightDrive = 0;
        public static final int backRightAngle = 0;
        public static final int backRightEncoder = 0;

    }

    public static class SwerveConstants {
		public static double swerveWheelDiameterMeters = .1d;
		public static double swerveGearRatio = 8.14d;
        
    }
    public static final int one = 1;
}
