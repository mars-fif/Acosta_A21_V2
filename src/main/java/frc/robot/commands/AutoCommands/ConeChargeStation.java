package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawCommands.chompOUTtime;
import frc.robot.commands.DriveCommands.Balance;
import frc.robot.commands.DriveCommands.ResetOdometry;
import frc.robot.commands.DriveCommands.StopDrive;
import frc.robot.commands.ArmCommands.AutoCommands.A_SetArmHome;
import frc.robot.commands.WristCommands.SetWristHome;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.ArmCommands.AutoCommands.A_SetArmConeHigh;
import frc.robot.commands.WristCommands.AutoCommands.A_SetWristHighCone;

public class ConeChargeStation extends SequentialCommandGroup{
    public ConeChargeStation(Pose2d initalPose, RamseteCommand part1){
        addCommands(
            new ResetOdometry(initalPose),
            new ParallelCommandGroup(new A_SetArmConeHigh(), new A_SetWristHighCone()),
            new chompOUTtime(),
            new ParallelRaceGroup(new A_SetArmHome(), new SetWristHome()),
            part1,
            new Balance(),
            new StopDrive()
        );
    }
}
