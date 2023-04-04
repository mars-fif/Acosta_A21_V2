package frc.robot.commands.ArmCommands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class T_SetArmConeHigh extends CommandBase{
    private Arm arm;

    public T_SetArmConeHigh(){
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override 
    public void initialize(){
        
    }

    @Override
    public void execute(){
        arm.setSetpoint(165);
        arm.setArmToPos();
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
