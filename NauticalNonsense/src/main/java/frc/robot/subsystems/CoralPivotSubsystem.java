// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralPivotSubsystem extends SubsystemBase {
  /** Creates a new CoralPivotSubsystem. */
  private final SparkMax pivot_motor;

  private final DutyCycleEncoder encoder;

  public CoralPivotSubsystem() {
    pivot_motor = new SparkMax(CoralConstants.kPivotMotorPort, MotorType.kBrushless);
    encoder = new DutyCycleEncoder(CoralConstants.kPivotEncoderPort);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Degree", PivotDegree());
    // This method will be called once per scheduler run
  }

  public void intakePivot(double speed) {
    pivot_motor.set(speed);
  }

  public double PivotDegree() {
    return encoder.get() * CoralConstants.PivotEncoderMultFactor
        - CoralConstants.PivotOffset; // Add Whatever Gyro.get() here
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
}
