package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj.XboxController;

public class closeClaw extends CommandBase{
    
    private final Wrist sub_wrist;
    XboxController _xbox;
    
    public closeClaw(Wrist wrist, XboxController xbox){
        sub_wrist = wrist;
        _xbox = xbox;
        addRequirements(sub_wrist);
    }

    @Override
    public void initialize(){
    
    }

    @Override
    public void execute(){
        sub_wrist.noidnoidREVERSE();
    }

    @Override
    public void end(boolean interrupted){
        sub_wrist.noidnoidOFF();
    }
}