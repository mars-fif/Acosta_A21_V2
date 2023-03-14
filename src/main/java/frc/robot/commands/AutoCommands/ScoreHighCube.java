package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.SetArmHome;
import frc.robot.commands.ArmCommands.SetArmVertical;
import frc.robot.commands.DriveCommands.StopDrive;
import frc.robot.commands.DriveCommands.ResetOdometry;

public class ScoreHighCube extends SequentialCommandGroup{
    public ScoreHighCube(Pose2d initalPose, RamseteCommand part1){
        addCommands(
            new ResetOdometry(initalPose),
            new SetArmVertical(),
            new SetArmHome(),
            part1,
            new StopDrive()
        );
    }
}
