// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.DriveOrientation;

public class SwerveDrivebase extends SubsystemBase {
    /** Creates a new SwerveDrivebase. */
    public SwerveModule[] modules;
    public SwerveDriveKinematics kinematics;
    public SwerveDriveOdometry odometry;
    public SwerveModuleState[] states;
    public SwerveDrivePoseEstimator poseEstimator;
    public SendableChooser<DriveOrientation> orientation = new SendableChooser<DriveOrientation>();

    public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    public static final Field2d field = new Field2d();

    /**
     * constructs a new swerve drivebase comprised of 2 or more modules (typically
     * four)
     * 
     * @param modules an array of modules in a drivebase.
     */
    public SwerveDrivebase(SwerveModule[] modules) {
        this.modules = modules;
        Translation2d[] translations = new Translation2d[modules.length];

        for (int i = 0; i < translations.length; i++) {
            translations[i] = modules[i].location;
        }

        for (SwerveModule swerveModule : modules) {
            SmartDashboard.putData(swerveModule.name, swerveModule);
        }

        kinematics = new SwerveDriveKinematics(translations);

        // measurement from just the wheels
        odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), getCurrentModulePositions());

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getRotation2d(), getCurrentModulePositions(),
                new Pose2d(0,0, Rotation2d.fromDegrees(180)));

        gyro.calibrate();
        orientation.setDefaultOption("Robot Oriented", DriveOrientation.RobotOriented);
        orientation.addOption("Field Oriented", DriveOrientation.FieldOriented);
        SmartDashboard.putData("Drive Mode", orientation);
        SmartDashboard.putData("field map", field);

        AutoBuilder.configureHolonomic(
                this::getPose2d, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSubsystemChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setSubsystemChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        odometry.update(getRotation2d(), getCurrentModulePositions());
        poseEstimator.update(getRotation2d(), getCurrentModulePositions());

        NetworkTable limelightNetworkTable = NetworkTableInstance.getDefault().getTable("limelight"); // https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api

        boolean limelightHasValidTargets = limelightNetworkTable.getEntry("tv").getDouble(0) == 1.0 ? true : false;
        NetworkTableEntry targetDistanceX = limelightNetworkTable.getEntry("tx"); // Horizontal Offset From Crosshair To
                                                                                  // Target (-27 degrees to 27 degrees)
        NetworkTableEntry targetDistanceY = limelightNetworkTable.getEntry("ty"); // Vertical Offset From Crosshair To
                                                                                  // Target (-20.5 degrees to 20.5
                                                                                  // degrees)
        NetworkTableEntry targetArea = limelightNetworkTable.getEntry("ta"); // Target Area (0% of image to 100% of
                                                                             // image)

        NetworkTableEntry botPose = limelightNetworkTable.getEntry("botpose_wpiblue"); // always blue relative coords

        double[] botPoseArray = botPose.getDoubleArray(new double[6]); // Translation(x,y,z), Rotation(roll, pitch,
                                                                       // yaw), full latency
        Pose2d estimatedPosition = new Pose2d(botPoseArray[0], botPoseArray[1],
                Rotation2d.fromDegrees(botPoseArray[5]).minus(Rotation2d.fromDegrees(90))); // TODO check that botPoseArray[5] is the correct est rotaton
                                                          // of the robot
        double currentTime = Timer.getFPGATimestamp() - (botPoseArray[6] / 1000.0);

        double primaryAprilTagID = limelightNetworkTable.getEntry("id").getDouble(0);

        if (limelightHasValidTargets) {
            poseEstimator.addVisionMeasurement(estimatedPosition, currentTime);
            // poseEstimator.setVisionMeasurementStdDevs(new MatBuilder(Nat.N3(),
            // Nat.N1()).fill(4, 4, 4)); // TODO NEEDED??
        }

        field.setRobotPose(poseEstimator.getEstimatedPosition());

    }

    /**
     * adds all values in the function to shuffleboard under the domain
     * "SwerveSubsystem"
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveSubsystem");
        SmartDashboard.putData("gyro", gyro);
        // builder.addStringProperty("Pose2d Odometry", () -> getPose2dOdometry().toString(), null);
        // builder.addStringProperty("Pose2d pose estimator", () -> getPose2dPoseEstimator().toString(), null);

        SmartDashboard.putNumber("Pose2d PE X", getPose2dPoseEstimator().getX());
        SmartDashboard.putNumber("Pose2d PE Y", getPose2dPoseEstimator().getY());
        SmartDashboard.putNumber("Pose2d Odometry X", getPose2dOdometry().getX());
        SmartDashboard.putNumber("Pose2d Odometry Y", getPose2dOdometry().getY());
    }

    /**
     * 
     * Converts chassisSpeeds for each module into SwerveModuleState. Calls
     * setSubsystemModuleStates with chassisSpeeds converted to SwerveModuleStates
     * 
     * @param chassisSpeeds the desired chassis speeds to set each module to
     */
    public void setSubsystemChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        setSubsystemModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * sets each module to the desired SwerveModuleState
     * 
     * @param desiredStates an array of desired module SwerveModuleStates.
     */
    public void setSubsystemModuleStates(SwerveModuleState[] desiredStates) {
        for (int i = 0; i < desiredStates.length; i++) {
            modules[i].setSwerveModuleState(desiredStates[i]);
        }
    }

    /**
     * 
     * @return an array of current module positions - used for updating the robot
     *         position on the field
     */
    public SwerveModulePosition[] getCurrentModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            modulePositions[i] = modules[i].getSwerveModulePosition();
        }
        return modulePositions;
    }

    /**
     * 
     * @return a Pose2d ( x, y ) of the robot position IN METERS
     */
    public Pose2d getPose2d() {
        return poseEstimator.getEstimatedPosition();
        // return odometry.getPoseMeters();
    }

    /**
     * 
     * @return a Pose2d ( x, y ) of the robot position IN METERS
     */
    public Pose2d getPose2dPoseEstimator() {
        return poseEstimator.getEstimatedPosition();
        // return odometry.getPoseMeters();
    }

    public Pose2d getPose2dOdometry() {
        // return poseEstimator.getEstimatedPosition();
        return odometry.getPoseMeters();
    }

    /**
     * 
     * @return a pose3d of the robots position on the field, and its rotation
     */
    public Pose3d getPose3d() {
        return new Pose3d(new Translation3d(getPose2d().getX(), getPose2d().getY(), 0),
                new Rotation3d(0, 0, getRotation2d().getRadians()));
    }

    /**
     * 
     * @return a Rotation2d of the robots current rotation
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    /**
     * 
     * @return a Rotation3d of the robots current rotation
     */
    public Rotation3d getRotation3d() {
        return new Rotation3d(); // TODO
        // return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    /**
     * converts the SwerveModuleStates of each module to
     * 
     * @return the current ChassisSpeeds of each module
     */
    public ChassisSpeeds getSubsystemChassisSpeeds() {
        SwerveModuleState[] currentModuleStates = new SwerveModuleState[this.modules.length];
        // gets current chassis state for each module
        for (int i = 0; i < this.modules.length; i++) {
            currentModuleStates[i] = modules[i].getSwerveModuleState();
        }
        return kinematics.toChassisSpeeds(currentModuleStates);
    }

    /**
     * resets the Position on the field of the Robot
     * used for the robot autonomous in order to reset the saved position on field
     * 
     * @param pose2d current position of the robot
     */
    public void resetOdometry(Pose2d pose2d) {
        odometry.resetPosition(
                getRotation2d(),
                getCurrentModulePositions(),
                pose2d);

        poseEstimator.resetPosition(
                getRotation2d(),
                getCurrentModulePositions(),
                pose2d);
    }

    /**
     * resets the Position on the field of the Robot
     * used for the robot autonomous in order to delete the last saved position on
     * field
     * 
     */
    public void resetOdometry() {
        resetOdometry(getPose2d());
    }

    /**
     * loops through each module and running stop(), a function to cut power to the
     * motors
     */
    public void stopModules() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].stop();
        }
    }

    /**
     * gets the driveMode from the SendableChooser in shuffleboard
     * 
     * @return the current select DriveMode, either RobotOriented or FieldOriented
     */
    public DriveOrientation getSelectedDriveMode() {
        return orientation.getSelected();
    }

    /**
     * sets each module to the NeutralModeValue - either Break or coast
     * Coast moves without locking the wheels, Break locks the wheels and resists
     * friction
     * 
     * @param neutralModeValue the NeutralMode to set the robot to
     */
    public void setModulesNeutralMode(NeutralModeValue neutralModeValue) {
        for (int i = 0; i < this.modules.length; i++) {
            modules[i].setModuleNeutralMode(neutralModeValue);
        }
    }
}