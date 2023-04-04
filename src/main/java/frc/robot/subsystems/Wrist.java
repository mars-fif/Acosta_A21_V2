package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase{
    private static Wrist wrist;
    private final CANSparkMax m_wristMotor;

    DoubleSolenoid wrist2;
    private final PIDController m_wristPID;
    private final RelativeEncoder m_wristEncoder;
    //private final Encoder m_wristEncoder;


    public Wrist(){
        //m_wristEncoder = new Encoder(6, 7, false, EncodingType.k4X);
        m_wristPID = new PIDController(WristConstants.kWristP, WristConstants.kWristI, WristConstants.kWristD);
        m_wristMotor = new CANSparkMax(WristConstants.kWristSparkMAXID, MotorType.kBrushless);
        //m_wristEncoder = m_wristMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 2048);
        m_wristEncoder = m_wristMotor.getEncoder();
        
        wrist2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 15, 14);


        config();
        resetEncoder();

    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Wrist Raw", getEncoderRaw());
        SmartDashboard.putNumber("Wrist Conversion Factor", getEncoderConversionFactor());
        SmartDashboard.putNumber("Wrist CPR", getEncoderCPR());
        SmartDashboard.putNumber("Wrist Angle", getEncoderDeg());
    }

    public static Wrist getInstance(){
        if(wrist == null){
            wrist = new Wrist();
        }
        return wrist;
    }

    public void config(){
        m_wristMotor.setSmartCurrentLimit(ArmConstants.kArmCurrentLimit);
        m_wristMotor.setIdleMode(IdleMode.kBrake);
        
    }

    public void noidnoidON(){
        wrist2.set(Value.kForward);
    }

    public void noidnoidREVERSE(){
        wrist2.set(Value.kReverse);
    }

    public void noidnoidOFF(){
        wrist2.set(Value.kOff);
    }
    
    public void setSpeed(double speed){
        m_wristMotor.set(speed);
    }

    public void stop(){
        m_wristMotor.set(0);
    }

    public double getEncoderConversionFactor(){
        return m_wristEncoder.getPositionConversionFactor();
    }

    public int getEncoderCPR(){
        return m_wristEncoder.getCountsPerRevolution();
    }

    // public void setEncoderConversionFactor(){
    //     m_wristEncoder.setPositionConversionFactor(WristConstants.kEncoderCPRtoDeg);
    // }

    public double getEncoderRaw(){
        return m_wristEncoder.getPosition();
    }

    public double getEncoderDeg(){
        return m_wristEncoder.getPosition()*WristConstants.kEncoderCPRtoDeg;
    }

    public void resetEncoder(){
        m_wristEncoder.setPosition(0.0);
    }



    // public double getEncoderValue(){
    //     //return m_wristEncoder.getPosition();
    //     return m_wristEncoder.getRaw();
    // }

    // public double getEncoderDeg(){
    //     return m_wristEncoder.getRaw()/22.5444;
    // }

    public void setToPos(double angle){
        setSpeed(MathUtil.clamp(m_wristPID.calculate(getEncoderDeg(), angle),-0.3,0.5));
    }

    public boolean getWristInPos(){
        return m_wristPID.atSetpoint();
    }

}