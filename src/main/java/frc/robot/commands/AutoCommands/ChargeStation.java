package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands.Balance;
import frc.robot.commands.DriveCommands.DriveBalance;
import frc.robot.commands.DriveCommands.ResetOdometry;
import frc.robot.commands.DriveCommands.StopDrive;

public class ChargeStation extends SequentialCommandGroup{
    public ChargeStation(Pose2d initalPose, RamseteCommand part1){
        addCommands(
            new ResetOdometry(initalPose),
            part1,
            new Balance(),
            new StopDrive()
        );
    }
}
