package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveOut extends SequentialCommandGroup{
    public DriveOut(Pose2d initalPose, RamseteCommand part1){
        addCommands(
            new ResetOdometry(initalPose),
            part1,
            new StopDrive()
        );
    }
}
