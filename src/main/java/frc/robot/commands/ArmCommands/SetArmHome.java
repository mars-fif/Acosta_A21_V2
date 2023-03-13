package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmHome extends CommandBase{
    private Arm arm;

    public SetArmHome(){
        arm = Arm.getInstance();
    }

    @Override 
    public void initialize(){
        
    }

    @Override
    public void execute(){
        arm.setArmToPos(5);
    }

    @Override
    public void end(boolean interrupted){
        arm.setArmSpeed(0);
    }

    @Override
    public boolean isFinished(){
        return arm.getArmInPos();
    }
}
