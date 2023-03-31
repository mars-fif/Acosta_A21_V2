package frc.robot.commands.WristCommands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class T_SetWristBackPick extends CommandBase{
    private Wrist wrist;

    public T_SetWristBackPick(){
        wrist = Wrist.getInstance();
        addRequirements(wrist);
    }

    @Override 
    public void initialize(){
        
    }

    @Override
    public void execute(){
        wrist.setToPos(125);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
