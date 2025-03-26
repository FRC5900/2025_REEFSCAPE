// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FullCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PivotAlgaetoPosition;
import frc.robot.commands.ShootAlgaeCmd;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeShootSequence extends SequentialCommandGroup {
  /** Creates a new AlgaeShootSequence. */
  public AlgaeShootSequence(
      AlgaeSubsystem algae,
      AlgaeIntakeSubsystem alageint,
      double speed,
      double pivspeed,
      double pos) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ShootAlgaeCmd(alageint, speed), new PivotAlgaetoPosition(algae, pivspeed, pos));
  }
}
