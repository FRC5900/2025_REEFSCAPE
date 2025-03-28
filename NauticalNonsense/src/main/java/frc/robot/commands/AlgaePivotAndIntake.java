// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaePivotAndIntake extends ParallelCommandGroup {
  /** Creates a new AlgaePivotAndIntake. */
  public AlgaePivotAndIntake(
      AlgaeIntakeSubsystem algaeint,
      AlgaeSubsystem algae,
      ElevatorSubsystem elevator,
      double speed,
      double pivotspeed,
      double pos,
      double elevatorpos) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new IntakeAlgae(algaeint, speed),
        new PivotAlgaetoPosition(algae, pivotspeed, pos),
        new ElevatorToPosition(elevator, 1, elevatorpos));
  }
}
