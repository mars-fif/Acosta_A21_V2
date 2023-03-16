package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class Pneumatics extends SubsystemBase{
    private static Pneumatics pneumatics;

    private final Compressor m_pneumaticsHub;

    public Pneumatics(){
        m_pneumaticsHub = new Compressor(PneumaticsModuleType.REVPH);
        m_pneumaticsHub.enableAnalog(PneumaticsConstants.kMinPressure, PneumaticsConstants.kMaxPressure);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Compressor On", getCompressorEnabled());
        SmartDashboard.putNumber("Pressure - HIGH", getAnalogPressure());
    }

    public static Pneumatics getInstance(){
        if(pneumatics == null){
            pneumatics = new Pneumatics();
        }
        return pneumatics;
    }

    public boolean getCompressorEnabled(){
        return m_pneumaticsHub.isEnabled();
    }

    public double getAnalogPressure(){
        return m_pneumaticsHub.getPressure();
    }
}
