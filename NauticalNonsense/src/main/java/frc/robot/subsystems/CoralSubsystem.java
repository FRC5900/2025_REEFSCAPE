// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralIntakeSubsystem. */
  private final SparkMax intake_motor;

  private final SparkMax pivot_motor;

  public CoralSubsystem() {
    intake_motor = new SparkMax(CoralConstants.kIntakeMotorPort, MotorType.kBrushless);
    pivot_motor = new SparkMax(CoralConstants.kPivotMotorPort, MotorType.kBrushed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeIntake(double speed) {
    intake_motor.set(speed);
  }

  public void intakePivot(double speed) {
    pivot_motor.set(speed);
  }

  public double PivotDegree() {
    return 0; // Add Whatever Gyro.get() here
  }

  public void SetPivotIntake(double speed) {
    if (PivotDegree()
        < CoralConstants.IntakePosition - 1) { // Change < or > to whichever one is needed.
      intakePivot(speed);
    }
    if (PivotDegree()
        > CoralConstants.IntakePosition + 1) { // Change < or > to whichever one is needed.
      intakePivot(-speed);
    }
  }

  public void SetPivotScore(double speed) {
    if (PivotDegree()
        > CoralConstants.ScoringPosition + 1) { // Change < or > to whichever one is needed.
      intakePivot(-speed);
    } else if (PivotDegree()
        < CoralConstants.ScoringPosition - 1) { // Change < or > to whichever one is needed.
      intakePivot(speed);
    }
  }

  public void SetPivotIdle(double speed) {
    if (PivotDegree() > 1) {
      intakePivot(speed);
    } else if (PivotDegree() < -1) {
      intakePivot(-speed);
    }
  }

  public boolean CoralDetected() {
    return false; // Update with a sensor when added
  }
}
