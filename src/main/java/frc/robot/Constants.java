// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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

 //Initializes the constants class
public final class Constants {
    /**
     * Sub-class of Constants holding ID's of controllers (not motor controllers)
     */
    //seperate sub-class for any constants used by the driver, so in this case any controller device ports
    //the ports can be found in driver station under the fourth tab
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kTopButtonBoard = 1;
        public static final int kBottomButtonBoard = 2;
    }

    //another seperate sub-class which we input any constants on the field which could affect things like
    //our driving or vision
    public static class FieldConstants {
        public static Translation2d targetPosition2d = new Translation2d(0.35, 5.55); // in terms of x and y on the
                                                                                      // field
        public static Translation3d targetPosition3d = new Translation3d(targetPosition2d.getX(),
                targetPosition2d.getY(), 2.1082); // height of speaker opening meters
        public static Pose3d targetPosition3dWithAngle = new Pose3d(targetPosition3d,
                new Rotation3d(0.0, Units.degreesToRadians(-45), 0.0));
    }

    /**
     * Sub-class of Constants that holds all ID's for any electronics on the robot
     * that need assignment in the code
     */
    //this excludes electronics such as the RoboRIO, PowerDistributionHub, PneumaticsHub, VoltageRegulatorModules, and RadioPowerModule
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

        public static final int indexerFalcon = 15;

        public static final int intakeMotor = 18;
        public static final int intakeSwitch = 0;

        public static final int topShooterMotorPort = 13;
        public static final int bottomShooterMotorPort = 14;

        public static final int climberMotor = 20;
        public static final int climberSwitch = 3;

        //Also would put any pneumatic things here (we do not have on our 2024 robot, but put here for example)
        //There are examples of pneumatics code being put to use in the 2022 and 2023 mainseason robots
        // public static final int[] doubleSolenoid1 = {0,1};
        // public static final int[] doubleSolenoid2 = {2,3};
        // public static final int solenoid = 4;
    }

    public static class conversionConstants{
        public static double climberConversion = Math.PI * 0.0275 / 25;
    }

    /**
     * Sub-class of Constants that contains measurements for swerve drive
     * as well as initializes the swerve modules
     */
    public static class SwerveConstants {
        public static double chassisWidthMetersNoBumpers = .6858d;
        public static double chassisLengthMetersNoBumpers = .6858d;
        public static double chassisWidthMetersBumpers = .84;
        public static double chassisLengthMetersBumpers = .84;
        public static double swerveWheelDiameterMeters = .0978d;
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

    public static class JoystickConstants {
        public static final double deadZoneRange = 0.15;
        public static final double deadZoneRotation = 0.10;

        public static final double maxLinearSpeedms = 4.5;
        public static final double maxRotationalSpeedDegrees = 360;

        public static final double joystickLinearityAdjustment = 0.8;
    }
        

    //very necessary constants, must be in every constants class in order for the robot to run correctly
    public static final int one = 1;
}
