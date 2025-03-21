// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralIntakeSubsystem. */
  private final SparkMax intake_motor;

  private final DigitalInput input1;
  private final DigitalInput input2;

  public CoralSubsystem() {
    intake_motor = new SparkMax(CoralConstants.kIntakeMotorPort, MotorType.kBrushless);
    input1 = new DigitalInput(CoralConstants.kDigitalInput1);
    input2 = new DigitalInput(CoralConstants.kDigitalInput2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Sensor 1", input1.get());
    SmartDashboard.putBoolean("Sensor 2", input2.get());
  }

  public void intakeIntake(double speed) {
    intake_motor.set(speed);
  }

  public boolean CoralSensor1() {
    if (input1.get() == true) {
      return true;
    } else {
      return false;
    }
  }

  public boolean CoralSensor2() {
    if (input2.get() == true) {
      return true;
    } else {
      return false;
    }
  }
}
