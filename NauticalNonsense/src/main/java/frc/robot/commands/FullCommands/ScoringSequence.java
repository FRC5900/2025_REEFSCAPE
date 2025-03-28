// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FullCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ScoreCoralCmd;
import frc.robot.commands.SetCoralScoreCmd;
import frc.robot.subsystems.CoralPivotSubsystem;
import frc.robot.subsystems.CoralSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoringSequence extends SequentialCommandGroup {
  /** Creates a new ScoringSequence. */
  public ScoringSequence(CoralSubsystem coral, CoralPivotSubsystem coralpiv, double speed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetCoralScoreCmd(coralpiv, speed), new ScoreCoralCmd(coral, speed));
  }
}
