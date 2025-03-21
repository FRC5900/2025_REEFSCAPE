// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotAlgaetoPosition extends Command {
  private AlgaeSubsystem s_algae;
  private double speed;
  private double pos;
  /** Creates a new PivotAlgae. */
  public PivotAlgaetoPosition(AlgaeSubsystem algae, double speed, double pos) {
    this.s_algae = algae;
    this.speed = speed;
    this.pos = pos;
    addRequirements(algae);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (s_algae.PivotAngle() > pos) {
      s_algae.PivotAlgae(-speed);
    } else {
      s_algae.PivotAlgae(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_algae.PivotAlgae(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (s_algae.PivotAngle() > pos - 1 && s_algae.PivotAngle() < pos + 1) {
      return true;
    } else {
      return false;
    }
  }
}
