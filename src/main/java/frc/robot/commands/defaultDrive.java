package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;

public class defaultDrive extends CommandBase {
    private final Drivetrain sub_Drivetrain;
    Joystick arcadeJoy;

    public defaultDrive(Drivetrain driveTrain, Joystick joystick){
        sub_Drivetrain = driveTrain;
        arcadeJoy = joystick;
        addRequirements(sub_Drivetrain);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
