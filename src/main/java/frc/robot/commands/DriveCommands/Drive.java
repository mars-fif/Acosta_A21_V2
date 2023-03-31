package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;

public class Drive extends CommandBase {
    private Drivetrain drivetrain;
    Joystick leftJoystick, rightJoystick;

    public Drive(Joystick leftStick, Joystick rightStick){
        drivetrain = Drivetrain.getInstance();
        
        leftJoystick = leftStick;
        rightJoystick = rightStick;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        drivetrain.tankDrive(-leftJoystick.getY(), -rightJoystick.getY());
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
