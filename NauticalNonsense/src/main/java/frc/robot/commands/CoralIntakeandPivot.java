// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.CoralPivotSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralIntakeandPivot extends ParallelCommandGroup {
  /** Creates a new AutoIntake. */
  public CoralIntakeandPivot(
      double Intakespeed,
      double Pivotspeed,
      double pivotposition,
      double elevatorposition,
      CoralSubsystem coral,
      CoralPivotSubsystem coralpiv,
      ElevatorSubsystem elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new PivotCoraltoPosition(coralpiv, -Pivotspeed, pivotposition),
        new IntakeCoralCmd(coral, Intakespeed));
    new ElevatorToPosition(elevator, 1, elevatorposition);
  }
}
