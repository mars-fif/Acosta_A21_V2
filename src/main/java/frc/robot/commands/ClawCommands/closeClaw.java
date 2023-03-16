package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Wrist;

public class closeClaw extends CommandBase{
    
    private final Wrist wrist;
    
    public closeClaw(){
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize(){
    
    }

    @Override
    public void execute(){
        wrist.noidnoidREVERSE();
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}