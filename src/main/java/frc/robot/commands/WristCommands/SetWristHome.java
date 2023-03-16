package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class SetWristHome extends CommandBase{
    private Wrist wrist;

    public SetWristHome(){
        wrist = Wrist.getInstance();
        addRequirements(wrist);
    }

    @Override 
    public void initialize(){
        
    }

    @Override
    public void execute(){
        wrist.setToPos(0);
    }

    @Override
    public void end(boolean interrupted){
        //wrist.setwristSpeed(0);
    }

    @Override
    public boolean isFinished(){
        if (wrist.getWristInPos()){
             return true;
        }
        return false;
    }
}
