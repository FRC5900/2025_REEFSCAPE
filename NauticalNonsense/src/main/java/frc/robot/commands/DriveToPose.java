// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
  /** Creates a new DriveToPose. */
  private static final double ANGLE_KP = 5.0;

  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double TRANSLATION_KP = 1.0;
  private static final double TRANSLATION_KD = 0.0;
  private static final double TRANSLATION_MAX_VELOCITY = 5.0;
  private static final double TRANSLATION_MAX_ACCELERATION = 10.0;

  ProfiledPIDController xController =
      new ProfiledPIDController(
          TRANSLATION_KP,
          0,
          TRANSLATION_KD,
          new TrapezoidProfile.Constraints(TRANSLATION_MAX_VELOCITY, TRANSLATION_MAX_VELOCITY));
  ProfiledPIDController yController =
      new ProfiledPIDController(
          TRANSLATION_KP,
          0,
          TRANSLATION_KD,
          new TrapezoidProfile.Constraints(TRANSLATION_MAX_VELOCITY, TRANSLATION_MAX_ACCELERATION));
  ProfiledPIDController angleController =
      new ProfiledPIDController(
          ANGLE_KP,
          0.0,
          ANGLE_KD,
          new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));

  DriveSubsystem drive;
  Pose2d targetPose;
  Supplier<Pose2d> poseSupplier;

  public DriveToPose(DriveSubsystem drive, Pose2d targetPose, Supplier<Pose2d> poseSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.targetPose = targetPose;
    this.poseSupplier = poseSupplier;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
    // Calculate angular speed
    double omega =
        angleController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            xController.calculate(currentPose.getX(), targetPose.getX()),
            yController.calculate(currentPose.getY(), targetPose.getY()),
            omega);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
