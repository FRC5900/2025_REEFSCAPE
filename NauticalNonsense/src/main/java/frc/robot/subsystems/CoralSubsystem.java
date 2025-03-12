// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralIntakeSubsystem. */
  private final SparkMax intake_motor;
  private final DutyCycleEncoder encoder;
  private final SparkMax pivot_motor;
  private final DigitalInput input1;
  private final DigitalInput input2;

  public CoralSubsystem() {
    intake_motor = new SparkMax(CoralConstants.kIntakeMotorPort, MotorType.kBrushless);
    pivot_motor = new SparkMax(CoralConstants.kPivotMotorPort, MotorType.kBrushless);
    encoder = new DutyCycleEncoder(CoralConstants.kPivotEncoderPort);
    input1 = new DigitalInput(CoralConstants.kDigitalInput1);
    input2 = new DigitalInput(CoralConstants.kDigitalInput2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Degree", PivotDegree());
    SmartDashboard.putBoolean("Coral Centered", CoralCentered());
    SmartDashboard.putBoolean("Coral Detected", CoralDetected());
  }

  public void intakeIntake(double speed) {
    intake_motor.set(speed);
  }

  public void intakePivot(double speed) {
    pivot_motor.set(speed);
  }

  public double PivotDegree() {
    return encoder.get() * CoralConstants.PivotEncoderMultFactor; // Add Whatever Gyro.get() here
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

  public boolean CoralCentered() {
    if (input1.get() == true && input2.get() == true){
      return true;
    }
    else{
      return false;
    }
  }

  public boolean CoralDetected(){
    if (input1.get() == true || input2.get() == true){
      return true;
    }
    else{
      return false;
    }
  }
}
