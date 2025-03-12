// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BadNotPathPlannerAutos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ElevatorToPosition;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleAuto extends ParallelCommandGroup {
  /** Creates a new SimpleAuto. */
  private final DriveSubsystem s_drive;

  private final CoralSubsystem s_coral;
  private final ElevatorSubsystem s_elevator;
  private final double speed;

  public SimpleAuto(
      DriveSubsystem drive, CoralSubsystem coral, ElevatorSubsystem elevator, double speed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.s_drive = drive;
    this.s_coral = coral;
    this.s_elevator = elevator;
    this.speed = speed;
    addCommands(
        new RunCommand(() -> s_drive.drive(speed, 0, 0, true, true), s_drive),
        new ElevatorToPosition(elevator, 0.5, ElevatorConstants.kHighScorePosition));
  }
}
