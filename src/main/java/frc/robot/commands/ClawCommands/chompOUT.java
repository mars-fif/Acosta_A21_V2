package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Claw;

public class chompOUT extends CommandBase{
    
    private final Claw claw;

    
    public chompOUT(){
        claw = Claw.getInstance();
        
    }

    @Override
    public void initialize(){
    
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
        return false;
        
    }
}