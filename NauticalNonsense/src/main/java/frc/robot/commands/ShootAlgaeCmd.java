// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootAlgaeCmd extends Command {
  /** Creates a new ShootAlgaeCmd. */
  private AlgaeIntakeSubsystem s_algae;

  private double speed;

  public ShootAlgaeCmd(AlgaeIntakeSubsystem algae, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_algae = algae;
    this.speed = speed;
    addRequirements(algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_algae.IntakeAlgae(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_algae.IntakeAlgae(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (s_algae.AlgaeDetected() == false) {
      return true;
    } else {
      return false;
    }
  }
}
