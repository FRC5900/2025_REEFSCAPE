// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToPosition extends Command {
  /** Creates a new ElevatorToBottom. */
  private ElevatorSubsystem s_elevator;

  private double speed;
  private double position;

  public ElevatorToPosition(ElevatorSubsystem elevator, double speed, double position) {
    this.s_elevator = elevator;
    this.speed = speed;
    this.position = position;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (s_elevator.GetElevatorPos() > position) {
      s_elevator.MoveElevator(-speed);
    } else {
      s_elevator.MoveElevator(speed);
    }

    if (s_elevator.GetElevatorPos() > ElevatorConstants.kMaxHeight) {
      s_elevator.MoveElevator(-speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_elevator.MoveElevator(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (s_elevator.GetElevatorPos() < position + .3
        && s_elevator.GetElevatorPos() > position - .3) {
      return true;
    } else {
      return false;
    }
  }
}
