package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Wrist;

public class wristUp extends CommandBase{
    
    private final Wrist wrist;
    
    public wristUp(){
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        wrist.setSpeed(0.5); //Remember to also check the sparkmax
    }

    @Override
    public void end(boolean interrupted){
        wrist.stop();
    }
}