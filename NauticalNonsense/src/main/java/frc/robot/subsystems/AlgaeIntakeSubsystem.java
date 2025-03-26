// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
  private final SparkMax intakeMotor;

  private final DigitalInput homeswitch;

  public AlgaeIntakeSubsystem() {
    intakeMotor = new SparkMax(AlgaeConstants.kIntakeMotorPort, MotorType.kBrushless);
    homeswitch = new DigitalInput(AlgaeConstants.kDigitalInput);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Algae Detected", AlgaeDetected());
    // This method will be called once per scheduler run
  }

  public void IntakeAlgae(double speed) {
    intakeMotor.set(speed);
  }

  public void ShootAlgae(double speed) {
    intakeMotor.set(-speed);
  }

  public boolean AlgaeDetected() {
    if (homeswitch.get() == true) {
      return false;
    } else {
      return true;
    }
  }
}
