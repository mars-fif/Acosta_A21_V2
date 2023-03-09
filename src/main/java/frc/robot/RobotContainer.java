// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import java.util.List;

public class RobotContainer {
  // Subsystem Declaration
  private final Drivetrain m_robotDrive = new Drivetrain();

  // Driver Joystick
  Joystick m_leftStick = new Joystick(OIConstants.kDriverLeftPort);
  Joystick m_rightStick = new Joystick(OIConstants.kDriverRightPort);

  // Operator Controller
  XboxController m_opController = new XboxController(OIConstants.kOperatorPort);

  public RobotContainer() {
    configureBindings();
    m_robotDrive.setDefaultCommand(
      new RunCommand(()->m_robotDrive.tankDrive(m_leftStick.getY(), m_rightStick.getY()), m_robotDrive));
  }

  private void configureBindings() {
    new JoystickButton(m_leftStick, 1)
    .onTrue(new InstantCommand(()->m_robotDrive.setMaxOutput(0.5)))
    .onFalse(new InstantCommand(()->m_robotDrive.setMaxOutput(1.0)));

    new JoystickButton(m_rightStick, 1)
    .onTrue(new InstantCommand(()->m_robotDrive.setMaxOutput(0.5)))
    .onFalse(new InstantCommand(()->m_robotDrive.setMaxOutput(1.0)));
  }

  public Command getAutonomousCommand() {
    // Voltage constraint to limit acceleration
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        DriveConstants.ksVolts, 
        DriveConstants.kvVoltSecondsPerMeter, 
        DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, 
        AutoConstants.kAutoMaxVolt);

    // Create trajectory config
    TrajectoryConfig config = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquard)
      .setKinematics(DriveConstants.kDriveKinematics)
      .addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(new Translation2d(1,1), new Translation2d(2, -1)), new Pose2d(3,0, new Rotation2d(0)), config);

    RamseteCommand ramseteCommand = 
    new RamseteCommand(
      exampleTrajectory, m_robotDrive::getPose, 
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter), 
      DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds, 
      new PIDController(DriveConstants.kPDriveVel, 0, 0), 
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
       m_robotDrive::tankDriveVolts,
       m_robotDrive);

    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    return ramseteCommand.andThen(()-> m_robotDrive.tankDriveVolts(0, 0));
  }
}
