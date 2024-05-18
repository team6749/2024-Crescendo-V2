package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivebase;

public class autoBackward extends Command {
    /** Creates a new RotateSwerveOnPoint. */
    SwerveDrivebase swerveDrivebase;
    Pose2d finalPose;
    double toleranceMemers = 0.2;

    public autoBackward(SwerveDrivebase drivebase) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.swerveDrivebase = drivebase;
        addRequirements(swerveDrivebase);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        finalPose = swerveDrivebase.getPose2d().transformBy(new Transform2d(new Translation2d(7.32,0), new Rotation2d(swerveDrivebase.getPose2d().getRotation().getDegrees())));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveDrivebase.setSubsystemChassisSpeeds(new ChassisSpeeds(-3.5, 0, 0));
        // swerveDrivebase.setSubsystemChassisSpeeds(new ChassisSpeeds(0, 0, 1));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveDrivebase.setSubsystemChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    double getErrorLocation() {
        return swerveDrivebase.getPose2d().relativeTo(finalPose).getX();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(getErrorLocation()) < toleranceMemers;
    }
}



