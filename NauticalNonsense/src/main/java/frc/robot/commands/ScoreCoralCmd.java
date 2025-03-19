// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreCoralCmd extends Command {
  /** Creates a new ScoreCoralCmd. */
  private CoralSubsystem s_coral;

  private Timer timer;
  private double speed;

  public ScoreCoralCmd(CoralSubsystem coral, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_coral = coral;
    this.speed = speed;
    addRequirements(coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_coral.intakeIntake(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_coral.intakeIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (s_coral.CoralDetected() == false) {
      timer.start();
      if (timer.get() > 1) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
}
