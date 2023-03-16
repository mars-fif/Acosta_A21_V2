// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmCommands.SetArmHome;
import frc.robot.commands.ArmCommands.TeleopCommands.T_SetArmBackPick;
import frc.robot.commands.ArmCommands.TeleopCommands.T_SetArmConeHigh;
import frc.robot.commands.ArmCommands.TeleopCommands.T_SetArmConeMid;
import frc.robot.commands.DriveCommands.Drive;
import frc.robot.commands.WristCommands.wristUp;
import frc.robot.commands.WristCommands.TeleopCommands.T_SetWristBackPick;
import frc.robot.commands.WristCommands.TeleopCommands.T_SetWristHighCone;
import frc.robot.commands.WristCommands.TeleopCommands.T_SetWristMidCone;
import frc.robot.commands.WristCommands.SetWristPickUp_Front;
import frc.robot.commands.WristCommands.SetWristHome;
import frc.robot.commands.WristCommands.wristDown;
import frc.robot.commands.ClawCommands.closeClaw;
import frc.robot.commands.ClawCommands.openClaw;


public class RobotContainer {
  // Subsystem Declaration
  private final Drivetrain drivetrain;
  private final Autonomous autonomous;
  private final Arm arm;
  private final Wrist wrist;
  private final Pneumatics pneumatics;

  // Driver Joystick
  Joystick m_leftStick = new Joystick(OIConstants.kDriverLeftPort);
  Joystick m_rightStick = new Joystick(OIConstants.kDriverRightPort);

  // Operator Controller
  XboxController m_opController = new XboxController(OIConstants.kOperatorPort);

  public RobotContainer() {
    drivetrain = Drivetrain.getInstance();
    autonomous = Autonomous.getInstance();
    arm = Arm.getInstance();
    wrist = Wrist.getInstance();
    pneumatics = Pneumatics.getInstance();

    drivetrain.setDefaultCommand(new Drive(m_leftStick, m_rightStick));
    arm.setDefaultCommand(new SetArmHome());
    wrist.setDefaultCommand(new SetWristHome());
    pneumatics.register();

    configureBindings();
    
  }

  private void configureBindings() {
    new JoystickButton(m_leftStick, 1)
    .onTrue(new InstantCommand(()->drivetrain.setMaxOutput(0.5)))
    .onFalse(new InstantCommand(()->drivetrain.setMaxOutput(1.0)));

    new JoystickButton(m_rightStick, 1)
    .onTrue(new InstantCommand(()->drivetrain.setMaxOutput(0.5)))
    .onFalse(new InstantCommand(()->drivetrain.setMaxOutput(1.0)));
  
    new JoystickButton(m_opController, 1)
    .whileTrue(Commands.parallel(new T_SetArmConeHigh(), new T_SetWristHighCone()));

    new JoystickButton(m_opController, 2)
    .whileTrue(new SetWristPickUp_Front());

    new JoystickButton(m_opController, 3)
    .whileTrue(new wristUp());

    new JoystickButton(m_opController, 4)
    .whileTrue(new wristDown());

    new JoystickButton(m_opController, 5) // X = 3, B = 2
    .whileTrue (new openClaw());

    new JoystickButton(m_opController, 6)
    .whileTrue (new closeClaw());

    new JoystickButton(m_opController, 9)
    .whileTrue(Commands.parallel(new T_SetArmBackPick(), new T_SetWristBackPick()));

    new JoystickButton(m_opController, 10)
    .whileTrue(Commands.parallel(new T_SetArmConeMid(), new T_SetWristMidCone()));

  }

  public Command getAutonomousCommand() {

    drivetrain.resetPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)), new Rotation2d(0.0));
    return autonomous.returnAutonomousCommand();
  }
}
