package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;;

public class Arm extends SubsystemBase{
    private static Arm arm;
    private final CANSparkMax m_armMotorFront, m_armMotorRear;
    private final MotorControllerGroup m_armMotors;
    private final Encoder m_armEncoder;
    private final PIDController m_armPID;


    public Arm(){
        m_armPID = new PIDController(ArmConstants.kArmP, ArmConstants.kArmI, ArmConstants.kArmD);
        m_armPID.setTolerance(5,10);
        m_armMotorFront = new CANSparkMax(ArmConstants.kArmFrontSparkMAXID, MotorType.kBrushless);
        m_armMotorRear = new CANSparkMax(ArmConstants.kArmRearSparkMAXID, MotorType.kBrushless);
        m_armMotorFront.setSmartCurrentLimit(ArmConstants.kArmCurrentLimit);
        m_armMotorRear.setSmartCurrentLimit(ArmConstants.kArmCurrentLimit);
        m_armMotorFront.setIdleMode(IdleMode.kBrake);
        m_armMotorRear.setIdleMode(IdleMode.kBrake);
        m_armMotors = new MotorControllerGroup(m_armMotorFront, m_armMotorRear);

        m_armEncoder = new Encoder(ArmConstants.kArmEncoderADIO, ArmConstants.kArmEncoderBDIO, ArmConstants.kArmEncoderReversed, EncodingType.k4X);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Angle", getEncoderDeg());
    }
    
    public static Arm getInstance(){
        if(arm == null){
            arm = new Arm();
        }
        return arm;
    }

    public void resetEncoder(){
        m_armEncoder.reset();
    }

    public void setArmVolts(double armVolts){
        m_armMotors.setVoltage(armVolts);
    }

    public void setArmSpeed(double speed){
        m_armMotors.set(speed);
    }

    public double getEncoderRawValue(){
        return m_armEncoder.getRaw();
    }

    public double getEncoderDeg(){
        return m_armEncoder.getRaw()/ArmConstants.kEncoderCPRtoDeg;
    }

    public void setArmToPos(double angle){
        setArmSpeed(MathUtil.clamp(m_armPID.calculate(getEncoderDeg(),angle),-0.5,0.5));
    }

    public boolean getArmInPos(){
        return m_armPID.atSetpoint();
    }
}
