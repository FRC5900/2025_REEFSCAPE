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
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final SparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final DigitalInput m_homeswitch;

  public ElevatorSubsystem() {
    m_motor = new SparkMax(ElevatorConstants.kElevatorMotorPort, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
    m_homeswitch = new DigitalInput(ElevatorConstants.kDigitalInput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getHomeSwitch() == true) {
      ResetEncoder();
    }

    SmartDashboard.putNumber("Elevator Position", GetElevatorPos());
    SmartDashboard.putBoolean("Elevator Home Switch", getHomeSwitch());
  }

  public void MoveElevator(double speed) {
    if (getHomeSwitch() == false) {
      m_motor.set(speed);
    } else if (speed < 0) {
      m_motor.set(0);
    } else {
      m_motor.set(speed);
    }
  }

  public double GetElevatorPos() {
    return m_encoder.getPosition()
        * ElevatorConstants.kEncoderMultFactor; // ADD AN ENCODER HERE IN INCHES
  }

  public boolean getHomeSwitch() {
    return m_homeswitch.get();
  }

  public void ResetEncoder() {
    m_encoder.setPosition(0);
  }
}
