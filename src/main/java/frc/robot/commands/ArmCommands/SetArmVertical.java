package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmVertical extends CommandBase{
    private Arm arm;

    public SetArmVertical(){
        arm = Arm.getInstance();
    }

    @Override 
    public void initialize(){
        
    }

    @Override
    public void execute(){
        arm.setSetpoint(180);
        arm.setArmToPos();
    }

    @Override
    public void end(boolean interrupted){
        arm.setArmSpeed(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
