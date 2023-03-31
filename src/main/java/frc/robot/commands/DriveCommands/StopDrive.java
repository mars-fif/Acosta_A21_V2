package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class StopDrive extends CommandBase{
    private Drivetrain drivetrain;

    public StopDrive(){
        drivetrain = Drivetrain.getInstance();
    }

    @Override
    public void initialize(){
        drivetrain.tankDriveVolts(0, 0);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
