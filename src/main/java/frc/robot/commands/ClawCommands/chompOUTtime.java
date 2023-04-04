package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Claw;

public class chompOUTtime extends CommandBase{
    
    private final Claw claw;
    private static Timer timer;
    private static double startTime;
    
    public chompOUTtime(){
        claw = Claw.getInstance();
        
    }

    @Override
    public void initialize(){
        startTime = timer.getFPGATimestamp();
    
    }

    @Override
    public void execute(){
        claw.setSpeed(-1);
    }

    @Override
    public void end(boolean interrupted){
        claw.setSpeed(0);
    }

    @Override
    public boolean isFinished(){
        if(timer.getFPGATimestamp() - startTime > 1){
            return true;
        }
        return false;
        
    }
}