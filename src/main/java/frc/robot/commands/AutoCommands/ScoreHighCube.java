package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ArmCommands.AutoCommands.A_SetArmConeHigh;
import frc.robot.commands.ClawCommands.openClaw;
import frc.robot.commands.DriveCommands.StopDrive;
import frc.robot.commands.WristCommands.AutoCommands.A_SetWristHighCone;
import frc.robot.commands.DriveCommands.ResetOdometry;

public class ScoreHighCube extends SequentialCommandGroup{
    public ScoreHighCube(Pose2d initalPose, RamseteCommand part1){
        addCommands(
            new ResetOdometry(initalPose),
            new ParallelCommandGroup(new A_SetArmConeHigh(), new A_SetWristHighCone()),
            new openClaw(),
            part1,
            new StopDrive()
        );
    }
}
