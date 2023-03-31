package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Balance extends CommandBase{
    private Drivetrain drivetrain;

    public Balance(){
        drivetrain = Drivetrain.getInstance();
        
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        drivetrain.setDriveBalance();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

