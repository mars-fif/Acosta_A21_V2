package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj.XboxController;

public class wristUp extends CommandBase{
    
    private final Wrist sub_wrist;
    XboxController _xbox;
    
    public wristUp(Wrist wrist, XboxController xbox){
        sub_wrist = wrist;
        _xbox = xbox;
        addRequirements(sub_wrist);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        sub_wrist.setSpeed(0.5); //Remember to also check the sparkmax
    }

    @Override
    public void end(boolean interrupted){
        sub_wrist.stop();
    }
}