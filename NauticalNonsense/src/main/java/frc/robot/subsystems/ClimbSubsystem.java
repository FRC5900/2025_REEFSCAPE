// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  private final SparkMax climb_motor;

  private final AbsoluteEncoder climb_encoder;

  public ClimbSubsystem() {
    climb_motor = new SparkMax(ClimbConstants.kClimbMotorPort, MotorType.kBrushless);
    climb_encoder = climb_motor.getAbsoluteEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb Encoder", getAbsoluteEncoder());
  }

  public double getAbsoluteEncoder() {
    return climb_encoder.getPosition();
  }

  public void Climb(double speed) {
    climb_motor.set(speed);
  }
}
