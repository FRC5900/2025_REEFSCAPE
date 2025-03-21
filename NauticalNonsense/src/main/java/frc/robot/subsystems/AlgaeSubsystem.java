// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeSubsystem. */
  private final SparkMax intakeMotor;

  private final DigitalInput homeswitch;
  private final SparkMax pivotMotor;
  private final RelativeEncoder pivotAngle;

  public AlgaeSubsystem() {
    intakeMotor = new SparkMax(AlgaeConstants.kIntakeMotorPort, MotorType.kBrushless);
    pivotMotor = new SparkMax(AlgaeConstants.kPivotMotorPort, MotorType.kBrushless);
    pivotAngle = pivotMotor.getEncoder();
    homeswitch = new DigitalInput(AlgaeConstants.kDigitalInput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Algae Pivot Angle", PivotAngle());
    SmartDashboard.putBoolean("Algae Detected", AlgaeDetected());
  }

  public void IntakeAlgae(double speed) {
    intakeMotor.set(speed);
  }

  public void ShootAlgae(double speed) {
    intakeMotor.set(-speed);
  }

  public void PivotAlgae(double speed) {
    pivotMotor.set(speed);
  }

  public double PivotAngle() {
    return pivotAngle.getPosition() * AlgaeConstants.kPivotDegreeMult;
  }

  public void ResetEncoder() {
    pivotAngle.setPosition(0);
  }

  public boolean AlgaeDetected() {
    if (homeswitch.get() == true) {
      return false;
    } else {
      return true;
    }
  }
}
