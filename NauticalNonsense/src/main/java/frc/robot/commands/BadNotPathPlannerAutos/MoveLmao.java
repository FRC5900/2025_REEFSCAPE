// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BadNotPathPlannerAutos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveLmao extends Command {
  /** Creates a new MoveLmao. */
  private final DriveSubsystem s_drive;

  private final double speed;
  private final Timer timer;

  public MoveLmao(DriveSubsystem drive, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_drive = drive;
    this.speed = speed;
    timer = new Timer();
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_drive.drive(speed, 0, 0, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_drive.drive(0, 0, 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() > 1) {
      return true;
    } else {
      return false;
    }
  }
}
