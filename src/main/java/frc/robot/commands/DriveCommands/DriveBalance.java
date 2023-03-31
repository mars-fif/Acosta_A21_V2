package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;

public class DriveBalance extends CommandBase{
    private Drivetrain drivetrain;
    Joystick leftJoystick, rightJoystick;

    public DriveBalance(Joystick leftStick, Joystick rightStick){
        drivetrain = Drivetrain.getInstance();
        leftJoystick = leftStick;
        rightJoystick = rightStick;
        
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        drivetrain.setDriveWBalance(-leftJoystick.getY(),-rightJoystick.getY());
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

