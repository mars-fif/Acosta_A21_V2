package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands.ResetOdometry;
import frc.robot.commands.DriveCommands.StopDrive;

public class TwoCone extends SequentialCommandGroup{
    public TwoCone(Pose2d initalPose, RamseteCommand part1, RamseteCommand part2){
        addCommands(
            new ResetOdometry(initalPose),
            part1,
            part2,
            new StopDrive()
        );
    }
}
