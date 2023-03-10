package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class StopDrive extends CommandBase{
    private Drivetrain drivetrain;

    public StopDrive(){
        drivetrain = Drivetrain.getInstance();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        drivetrain.tankDriveVolts(0, 0);
    }
}
