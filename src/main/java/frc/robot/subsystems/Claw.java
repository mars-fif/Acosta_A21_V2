package frc.robot.subsystems;

import frc.robot.Constants.ClawConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase{
    private static Claw claw;
    private final CANSparkMax m_clawMotor;


    public Claw(){
        m_clawMotor = new CANSparkMax(ClawConstants.kClawSparkMAXID, MotorType.kBrushless);

        config();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Claw Temp", getMotorTemperature());
    }

    public static Claw getInstance(){
        if(claw == null){
            claw = new Claw();
        }
        return claw;
    }

    public void config(){
        m_clawMotor.setSmartCurrentLimit(ClawConstants.kClawCurrentLimit);
        m_clawMotor.setIdleMode(IdleMode.kCoast);
        
    }

    public void setSpeed(double speed){
        m_clawMotor.set(speed);
    }

    public void stop(){
        m_clawMotor.set(0);
    }

    public double getMotorTemperature(){
        return m_clawMotor.getMotorTemperature();
    }
}