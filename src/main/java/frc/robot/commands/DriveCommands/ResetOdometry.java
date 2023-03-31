package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ResetOdometry extends CommandBase{
    private Drivetrain drivetrain;
    private Pose2d newPose;
    public ResetOdometry(Pose2d newPose){
        drivetrain = Drivetrain.getInstance();
        this.newPose = newPose;
    }

    @Override
    public void initialize(){
        drivetrain.resetOdometry(newPose);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
