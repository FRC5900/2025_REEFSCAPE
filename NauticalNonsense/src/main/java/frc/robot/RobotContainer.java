// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.BadNotPathPlannerAutos.MoveLmao;
import frc.robot.commands.ElevatorToPosition;
import frc.robot.commands.FullCommands.IntakeSequence;
import frc.robot.commands.FullCommands.ScoringSequence;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.IntakeCoralCmd;
import frc.robot.commands.ScoreCoralCmd;
import frc.robot.commands.TurboCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralPivotSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Vision vision;

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final CoralSubsystem m_coral = new CoralSubsystem();
  private final CoralPivotSubsystem m_coralpiv = new CoralPivotSubsystem();
  private final AlgaeSubsystem m_algae = new AlgaeSubsystem();
  private final ClimbSubsystem m_climb = new ClimbSubsystem();
  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);
  // private final GenericHID m_operatorController = new GenericHID(0);
  private final XboxController m_operatorController = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand(
        "Reset Gyro", new InstantCommand(() -> m_robotDrive.zeroHeading()));
    NamedCommands.registerCommand("Intake", new IntakeCoralCmd(m_coral, 0.25));
    NamedCommands.registerCommand("Score", new ScoringSequence(m_coral, m_coralpiv, 0.5));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        vision =
            new Vision(
                m_robotDrive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1),
                new VisionIOPhotonVision(camera2Name, robotToCamera2),
                new VisionIOPhotonVision(camera3Name, robotToCamera3));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        vision =
            new Vision(
                m_robotDrive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, m_robotDrive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, m_robotDrive::getPose),
                new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, m_robotDrive::getPose),
                new VisionIOPhotonVisionSim(camera3Name, robotToCamera3, m_robotDrive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        vision =
            new Vision(m_robotDrive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Configure the button bindings
    configureButtonBindings();
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive( // Left and Right sticks (Driver)
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.08),
                    MathUtil.applyDeadband(m_driverController.getLeftX(), 0.08),
                    MathUtil.applyDeadband(m_driverController.getRightX(), 0.08),
                    true,
                    true),
            m_robotDrive));

    m_algae.setDefaultCommand(
        new RunCommand(() -> m_algae.PivotAlgae(m_operatorController.getLeftY() / 5), m_algae));

    m_coral.setDefaultCommand(
        new RunCommand(
            () -> m_coralpiv.intakePivot(m_operatorController.getRightX() / 4), m_coral));

    /*m_elevator.setDefaultCommand(
    new RunCommand(
        () -> m_elevator.MoveElevator(-m_operatorController.getRightY()), m_elevator));*/

    /*m_elevator.setDefaultCommand(
    new RunCommand(
        () -> m_elevator.MoveElevator(m_operatorController.getRightY()), m_elevator));*/
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Joystick m_robotDrive command
    new JoystickButton(m_driverController, 5) // Left Bumper, Reset Gyro
        .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading()));

    new JoystickButton(m_driverController, 6) // Right Bumper, Turbo Mode
        .onTrue(new TurboCommand(m_robotDrive, true))
        .onFalse(new TurboCommand(m_robotDrive, false));

    new JoystickButton(m_driverController, 2) // B, Climb
        .whileTrue(new RunCommand(() -> m_climb.Climb(0.5), m_climb))
        .whileFalse(new RunCommand(() -> m_climb.Climb(0), m_climb));

    new JoystickButton(m_driverController, 8) // Menu, Reset Elevator Encoder
        .whileTrue(new RunCommand(() -> m_elevator.ResetEncoder(), m_elevator));

    new JoystickButton(m_driverController, 7) // Back, Reset Algae Encoder
        .whileTrue(new RunCommand(() -> m_algae.ResetEncoder(), m_algae));

    new JoystickButton(m_driverController, 3) // X, Reverse Climb
        .whileTrue(new RunCommand(() -> m_climb.Climb(-0.5), m_climb))
        .whileFalse(new RunCommand(() -> m_climb.Climb(0), m_climb));

    /*new JoystickButton(m_operatorController, 4) // Y, Elevator Up
        .whileTrue(new RunCommand(() -> m_elevator.MoveElevator(1), m_elevator))
        .whileFalse(new RunCommand(() -> m_elevator.MoveElevator(0), m_elevator));

    new JoystickButton(m_operatorController, 1) // A, Elevator Down
        .whileTrue(new RunCommand(() -> m_elevator.MoveElevator(-1), m_elevator))
        .whileFalse(new RunCommand(() -> m_elevator.MoveElevator(0), m_elevator));*/

    new JoystickButton(m_operatorController, 1) // 0, Elevator to L1
        .whileTrue(new RunCommand(() -> m_elevator.MoveElevator(-1), m_elevator))
        .whileFalse(new RunCommand(() -> m_elevator.MoveElevator(0), m_elevator));

    new JoystickButton(m_operatorController, 2) // 1, Elevator to L2
        .whileTrue(new ElevatorToPosition(m_elevator, 1, ElevatorConstants.kLowScorePosition));

    new JoystickButton(m_operatorController, 3) // 4, Elevator to L3
        .whileTrue(new ElevatorToPosition(m_elevator, 1, ElevatorConstants.kMidScorePosition));

    new JoystickButton(m_operatorController, 4) // 7, Elevator to L4
        .whileTrue(new ElevatorToPosition(m_elevator, 1, ElevatorConstants.kHighScorePosition));

    new JoystickButton(m_operatorController, 6) // Enter, Score Coral
        .whileTrue(new ScoreCoralCmd(m_coral, 0.25));

    new JoystickButton(m_operatorController, 5) // +, Intake Coral
        .whileTrue(
            new IntakeSequence(
                0.25,
                0.1,
                CoralConstants.IntakePosition,
                CoralConstants.LevelPosition,
                m_coral,
                m_coralpiv));

    /*new JoystickButton(m_operatorController, 5) // +, Intake Coral
    .whileTrue(new IntakeCoralCmd(m_coral, 0.25));*/

    new JoystickButton(m_operatorController, 7) // 3, Intake Algae
        .whileTrue(new IntakeAlgae(m_algae, -1.0));

    new JoystickButton(m_operatorController, 8) // 6, Shoot Algae
        .whileTrue(new IntakeAlgae(m_algae, 1.0));

    /*new JoystickButton(m_operatorController, 9) // 2, Pivot Algae Down
        .whileTrue(new PivotAlgae(m_algae, -0.5));

    new JoystickButton(m_operatorController, 10) // 5, Pivot Algae Up
        .whileTrue(new PivotAlgae(m_algae, 0.5));*/

    // Auto aim command example
    /*@SuppressWarnings("resource")
    PIDController aimController = new PIDController(0.2, 0.0, 0.0);
    aimController.enableContinuousInput(-Math.PI, Math.PI);
    new JoystickButton(m_driverController, 2) // B, AutoAim
        .whileTrue(
            Commands.startRun(
                () -> {
                  aimController.reset();
                },
                () -> {
                  m_robotDrive.drive(
                      0.0,
                      0.0,
                      aimController.calculate(vision.getTargetX(0).getRadians()),
                      true,
                      true);
                },
                m_robotDrive));/* */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    return new MoveLmao(m_robotDrive, .75);
  }
}
