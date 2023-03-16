package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class StopArm extends CommandBase{
    private Arm arm;

    public StopArm (){
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize(){
        arm.setArmSpeed(0);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
