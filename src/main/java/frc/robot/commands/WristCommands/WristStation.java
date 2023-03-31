package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class WristStation extends CommandBase{
    private Wrist wrist;

    public WristStation(){
        wrist = Wrist.getInstance();
        addRequirements(wrist);
    }

    @Override 
    public void initialize(){
        
    }

    @Override
    public void execute(){
        wrist.setToPos(65);
    }

    @Override
    public void end(boolean interrupted){
        //wrist.setwristSpeed(0);
    }

    @Override
    public boolean isFinished(){
        // if (wrist.getwristInPos()){
        //     return true;
        // }
        return false;
    }
}