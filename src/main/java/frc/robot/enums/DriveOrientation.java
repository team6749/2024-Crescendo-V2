package frc.robot.enums;

/**
 * An ENUM to control the currently selected drive orientation
 * 
 * RobotOriented - drive scheme where the robot moves front relative to the
 * front of the robot regardless of rotation.
 * 
 * 
 * FieldOriented - Field oriented driving is a drive scheme for holonomic
 * drivetrains, where the driver moves the controls relative to their
 * perspective of
 * the field, and the robot moves in that direction regardless of where the
 * front of the robot is facing/rotated.
 * 
 * see
 * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
 * 
 */
public enum DriveOrientation {
    RobotOriented, FieldOriented
}

