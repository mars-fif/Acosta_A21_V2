// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmCommands.SetArmHome;
import frc.robot.commands.ArmCommands.SetArmHorizontalFront;
import frc.robot.commands.ArmCommands.SetArmVertical;
import frc.robot.commands.ArmCommands.SetArmConeHigh;
import frc.robot.commands.DriveCommands.Drive;
import frc.robot.commands.WristCommands.wristUp;
import frc.robot.commands.WristCommands.wristDown;
import frc.robot.commands.ClawCommands.closeClaw;
import frc.robot.commands.ClawCommands.openClaw;


public class RobotContainer {
  // Subsystem Declaration
  private final Drivetrain drivetrain;
  private final Autonomous autonomous;
  private final Arm arm;
  private final Wrist sub_Wrist;

  // Driver Joystick
  Joystick m_leftStick = new Joystick(OIConstants.kDriverLeftPort);
  Joystick m_rightStick = new Joystick(OIConstants.kDriverRightPort);

  // Operator Controller
  XboxController m_opController = new XboxController(OIConstants.kOperatorPort);

  public RobotContainer() {
    drivetrain = Drivetrain.getInstance();
    autonomous = Autonomous.getInstance();
    arm = Arm.getInstance();
    sub_Wrist = new Wrist();

    configureBindings();
    drivetrain.setDefaultCommand(new Drive(m_leftStick, m_rightStick));
  }

  private void configureBindings() {
    new JoystickButton(m_leftStick, 1)
    .onTrue(new InstantCommand(()->drivetrain.setMaxOutput(0.5)))
    .onFalse(new InstantCommand(()->drivetrain.setMaxOutput(1.0)));

    new JoystickButton(m_rightStick, 1)
    .onTrue(new InstantCommand(()->drivetrain.setMaxOutput(0.5)))
    .onFalse(new InstantCommand(()->drivetrain.setMaxOutput(1.0)));
  
    new JoystickButton(m_opController, 1)
    .whileTrue(new SetArmConeHigh())
    .onFalse(new SetArmHome());

    new JoystickButton(m_opController, 2)
    .whileTrue(new SetArmVertical())
    .onFalse(new SetArmHome());

    new JoystickButton(m_opController, 3)
    .whileTrue(new wristUp(sub_Wrist, m_opController));

    new JoystickButton(m_opController, 4)
    .whileTrue(new wristDown(sub_Wrist, m_opController));

    new JoystickButton(m_opController, 5) // X = 3, B = 2
    .whileTrue (new openClaw(sub_Wrist, m_opController));

    new JoystickButton(m_opController, 6)
    .whileTrue (new closeClaw(sub_Wrist, m_opController));

  }

  public Command getAutonomousCommand() {

    drivetrain.resetPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)), new Rotation2d(0.0));
    return autonomous.returnAutonomousCommand();
  }
}

    // Voltage constraint to limit acceleration
  //   var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
  //     new SimpleMotorFeedforward(
  //       DriveConstants.ksVolts, 
  //       DriveConstants.kvVoltSecondsPerMeter, 
  //       DriveConstants.kaVoltSecondsSquaredPerMeter),
  //       DriveConstants.kDriveKinematics, 
  //       AutoConstants.kAutoMaxVolt);

  //   // Create trajectory config
  //   TrajectoryConfig config = new TrajectoryConfig(
  //     AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquard)
  //     .setKinematics(DriveConstants.kDriveKinematics)
  //     .addConstraint(autoVoltageConstraint);

  //   Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
  //     new Pose2d(0, 0, new Rotation2d(0)),
  //     List.of(new Translation2d(1, -0.5), new Translation2d(2, -0.5)), new Pose2d(3,0, new Rotation2d(0)), config);

  //   Trajectory exampleTrajectory2 = TrajectoryGenerator.generateTrajectory(
  //     new Pose2d(3, 0, new Rotation2d(180)), 
  //     List.of(new Translation2d(2,0), new Translation2d(1,0)), new Pose2d(0,0, new Rotation2d(0)), config);

  //   RamseteCommand ramseteCommand = 
  //   new RamseteCommand(
  //     exampleTrajectory, drivetrain::getPose, 
  //     new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
  //     new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter), 
  //     DriveConstants.kDriveKinematics, drivetrain::getWheelSpeeds, 
  //     new PIDController(DriveConstants.kPDriveVel, 0, 0), 
  //     new PIDController(DriveConstants.kPDriveVel, 0, 0),
  //      drivetrain::tankDriveVolts,
  //      drivetrain);

  //  RamseteCommand ramseteCommand2 = 
  //   new RamseteCommand(
  //     exampleTrajectory2, drivetrain::getPose, 
  //     new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
  //     new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter), 
  //     DriveConstants.kDriveKinematics, drivetrain::getWheelSpeeds, 
  //     new PIDController(DriveConstants.kPDriveVel, 0, 0), 
  //     new PIDController(DriveConstants.kPDriveVel, 0, 0),
  //      drivetrain::tankDriveVolts,
  //      drivetrain);
      

  //   drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

  //   return ramseteCommand
  //   .andThen(()-> ramseteCommand2
  //   .andThen(()->drivetrain.tankDriveVolts(0, 0)));
  // }

