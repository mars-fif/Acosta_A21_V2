package frc.robot.commands.WristCommands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class A_SetWristHighCone extends CommandBase{
    private Wrist wrist;

    public A_SetWristHighCone(){
        wrist = Wrist.getInstance();
        addRequirements(wrist);
    }

    @Override 
    public void initialize(){
        
    }

    @Override
    public void execute(){
        //wrist.setToPos(135); <--Old command 
        wrist.setToPos(125);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        if (wrist.getWristInPos()){
             return true;
        }
        return false;
    }
}
