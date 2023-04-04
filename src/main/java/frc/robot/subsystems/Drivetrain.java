package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
    private static Drivetrain drivetrain;

    private final CANSparkMax m_LFSparkMax = new CANSparkMax(DriveConstants.kLFSparkCANID, MotorType.kBrushless);
    private final CANSparkMax m_LRSparkMax = new CANSparkMax(DriveConstants.kLRSparkCANID, MotorType.kBrushless);
    private final CANSparkMax m_RFSparkMax = new CANSparkMax(DriveConstants.kRFSparkCANID, MotorType.kBrushless);
    private final CANSparkMax m_RRSparkMax = new CANSparkMax(DriveConstants.kRRSparkCANID, MotorType.kBrushless);


    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_LFSparkMax, m_LRSparkMax);
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_RFSparkMax, m_RRSparkMax);

    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    private final Encoder m_leftEncoder = new Encoder(DriveConstants.kLEncoderADIO, DriveConstants.kLEncoderBDIO,DriveConstants.kLEncoderReversed);
    private final Encoder m_rightEncoder = new Encoder(DriveConstants.kREncoderADIO, DriveConstants.kREncoderBDIO,DriveConstants.kREncoderReversed);

    private final ADIS16470_IMU m_imu = new ADIS16470_IMU();

    //private final Gyro m_Gyro = new 

    private final DifferentialDriveOdometry m_Odometry; 

    private final PIDController m_DrivePID;

    public Drivetrain(){
        m_DrivePID = new PIDController(0.025, 0, 0);
        CameraServer.startAutomaticCapture();
        m_LFSparkMax.setSmartCurrentLimit(DriveConstants.kCurrentLimit);
        m_LRSparkMax.setSmartCurrentLimit(DriveConstants.kCurrentLimit);
        m_RFSparkMax.setSmartCurrentLimit(DriveConstants.kCurrentLimit);
        m_RRSparkMax.setSmartCurrentLimit(DriveConstants.kCurrentLimit);

        m_leftMotors.setInverted(DriveConstants.kLInvertMotor);
        m_rightMotors.setInverted(DriveConstants.kRInvertMotor);

        m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

        resetEncoders();
        
        //Settng up odometry object; gets angle from gryo, distances from encoders 
        m_Odometry = new DifferentialDriveOdometry(getRotation(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

        //Pose2d is the starting position..

    }

    @Override
    public void periodic(){
        m_Odometry.update(getRotation(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

        SmartDashboard.putNumber("IMU Angle", m_imu.getAngle());
        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putNumber("Pitch", getPitch());
        SmartDashboard.putNumber("Left Encoder", getLeftEncoder().getDistance());
        SmartDashboard.putNumber("Right Encoder", getRighEncoder().getDistance());
        SmartDashboard.putNumber("Left Front Motor Temp", getMotorLFTemperature());
        SmartDashboard.putNumber("Left Rear Motor Temp", getMotorLRTemperature());
        SmartDashboard.putNumber("Right Front Motor Temp", getMotorRFTemperature());
        SmartDashboard.putNumber("Right Rear Motor Temp", getMotorLRTemperature());
    }

    public static Drivetrain getInstance(){
        if(drivetrain == null){
            drivetrain = new Drivetrain();
        }
        return drivetrain;
    }

    public void resetEncoders(){
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    //Currently estimated pose of the robot
    public Pose2d getPose(){
        return m_Odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    }

    public void resetOdometry(Pose2d pose){
        resetEncoders();
        m_Odometry.resetPosition(getRotation(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
    }

    public void arcadeDrive(double fwd, double rot){
        m_drive.arcadeDrive(-fwd, rot);
    }

    public void tankDrive(double left, double right){
        m_drive.tankDrive(left, right);
    }

    public double getHeading(){
        return (Math.floorMod((long) m_imu.getAngle(), (long) 360));
    }

    public Rotation2d getRotation(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public void tankDriveVolts(double leftVolts, double rightVolts){
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(rightVolts);
        m_drive.feed();
    }

    public double getAverageEncoderDistance(){
        return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
    }

    public Encoder getLeftEncoder(){
        return m_leftEncoder;
    }

    public Encoder getRighEncoder(){
        return m_rightEncoder;
    }

    public void setMaxOutput(double maxOutput){
        m_drive.setMaxOutput(maxOutput);
    }

    public void zeroHeading(){
        m_imu.reset();
    }

    public double getPitch(){
        return m_imu.getYComplementaryAngle();
    }

    public double getTurnRate(){
        return -m_imu.getRate();
    }

    public double getMotorLFTemperature(){
         return m_LFSparkMax.getMotorTemperature();
    }

    public double getMotorLRTemperature(){
        return m_LRSparkMax.getMotorTemperature();
    }

    public double getMotorRFTemperature(){
        return m_RFSparkMax.getMotorTemperature();
    }

    public double getMotorRRTemperature(){
        return m_RRSparkMax.getMotorTemperature();
    }   

    public void resetPose(Pose2d estimatedPostion, Rotation2d gyroAngle){
        resetEncoders();
        m_Odometry.resetPosition(getRotation(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), estimatedPostion);
    }

    public void setDriveBalance(){
        arcadeDrive(MathUtil.clamp(m_DrivePID.calculate(getPitch(),-2.4), -0.4, 0.4), 0);
        //arcadeDrive(0.3, 0);
    }

    public void setDriveWBalance(double left, double right){
        tankDrive(left+m_DrivePID.calculate(getPitch(),-2.4), right+m_DrivePID.calculate(getPitch(),-2.4));
    }
}