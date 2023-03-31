package frc.robot.commands.ArmCommands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class A_SetArmHome extends CommandBase{
    private Arm arm;

    public A_SetArmHome(){
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override 
    public void initialize(){
    }

    @Override
    public void execute(){
        arm.setSetpoint(10);
        arm.setArmToPos();
        
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
