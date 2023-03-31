package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ResetArmEncoder extends CommandBase{
    private Arm arm;

    public ResetArmEncoder(){
        arm = Arm.getInstance();
    }

    @Override
    public void initialize(){
        arm.resetEncoder();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    
}
