package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.SetArmHome;
import frc.robot.commands.ArmCommands.SetArmVertical;

public class ScoreHighCube extends SequentialCommandGroup{
    public ScoreHighCube(Pose2d initalPose, RamseteCommand part1){
        addCommands(
            new SetArmVertical(),
            new SetArmHome(),
            part1
        );
    }
}
