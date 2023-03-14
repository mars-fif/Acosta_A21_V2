package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase{

    private final CANSparkMax brushedWrist;
    private final Compressor PHcompressor = new Compressor(1, PneumaticsModuleType.REVPH);
    DoubleSolenoid wrist;
    private final PIDController wristController;
    //private final RelativeEncoder wristEncoder;
    private final Encoder wristEncoder;


    public Wrist(){
        wristEncoder = new Encoder(6, 7, false, EncodingType.k4X);
        wristController = new PIDController(0.025, 0, 0);
        brushedWrist = new CANSparkMax(11, MotorType.kBrushless);
        //wristEncoder = brushedWrist.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 2048);
        wrist = new DoubleSolenoid(PneumaticsModuleType.REVPH, 15, 14);
        PHcompressor.enableAnalog(60, 120);
        PHcompressor.isEnabled();

        config();

    }

    public void config(){
        brushedWrist.setSmartCurrentLimit(ArmConstants.kArmCurrentLimit);
        brushedWrist.setIdleMode(IdleMode.kBrake);
    }

    public void noidnoidON(){
        wrist.set(Value.kForward);
    }

    public void noidnoidREVERSE(){
        wrist.set(Value.kReverse);
    }

    public void noidnoidOFF(){
        wrist.set(Value.kOff);
    }
    
    public void setSpeed(double speed){
        brushedWrist.set(speed);
    }

    public void stop(){
        brushedWrist.set(0);
    }

    public double getEncoderValue(){
        //return wristEncoder.getPosition();
        return wristEncoder.getRaw();
    }

    public double getEncoderDeg(){
        return wristEncoder.getRaw()/22.5444;
    }

    public void setToPos(double angle){
        setSpeed(wristController.calculate(getEncoderDeg(), angle));
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Wrist Raw", getEncoderValue());
        SmartDashboard.putNumber("Wrist Angle", getEncoderDeg());
        SmartDashboard.putNumber("Compressor Levels", PHcompressor.getPressure());
    }
}