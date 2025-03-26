// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPivotSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotCoraltoPosition extends Command {
  /** Creates a new PivotCoral. */
  private final CoralPivotSubsystem s_coral;

  private final double speed;
  private final double pos;

  public PivotCoraltoPosition(CoralPivotSubsystem coral, double speed, double pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_coral = coral;
    this.speed = speed;
    this.pos = pos;
    addRequirements(coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (s_coral.PivotDegree() > pos) {
      s_coral.intakePivot(-speed);
    } else {
      s_coral.intakePivot(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_coral.intakePivot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (s_coral.PivotDegree() > pos - .5 && s_coral.PivotDegree() < pos + .5) {
      return true;
    } else {
      return false;
    }
  }
}
