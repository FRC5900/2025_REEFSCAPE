// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FullCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaePivotAndIntake;
import frc.robot.commands.PivotAlgaetoPosition;
import frc.robot.subsystems.AlgaeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AglaeSequence extends SequentialCommandGroup {
  /** Creates a new AglaeSequence. */
  public AglaeSequence(AlgaeSubsystem algae, double speed, double intakepos, double holdpos) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AlgaePivotAndIntake(algae, speed, intakepos),
        new PivotAlgaetoPosition(algae, speed, holdpos));
  }
}
