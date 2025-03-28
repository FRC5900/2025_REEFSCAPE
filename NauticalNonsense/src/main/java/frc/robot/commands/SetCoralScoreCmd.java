// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralPivotSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetCoralScoreCmd extends Command {
  /** Creates a new SetCoralScoreCmd. */
  private CoralPivotSubsystem s_coral;

  private double speed;

  public SetCoralScoreCmd(CoralPivotSubsystem coral, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_coral = coral;
    this.speed = speed;
    addRequirements(coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_coral.SetPivotScore(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_coral.intakePivot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (s_coral.PivotDegree() > CoralConstants.LevelPosition - 1
        || s_coral.PivotDegree() < CoralConstants.LevelPosition + 1) {
      return true;
    } else {
      return false;
    }
  }
}
