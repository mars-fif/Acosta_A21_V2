package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {

    public static final class DriveConstants{
        // Drivetrain Physical Parameters
        public static final double kWheelRadiusMeters = 0.0762; // Meters
        public static final double kGearboxRatio = 10.75;
        public static final double kTrackwidthMeters = 0.508;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        // Spark Max CAN IDs
        public static final int kLFSparkCANID = 15;
        public static final int kLRSparkCANID = 16;
        public static final int kRFSparkCANID = 21;
        public static final int kRRSparkCANID = 22;

        // Drivetrain Encoder Information
        public static final int kEncoderCPR = 2048;
        public static final int kLEncoderADIO = 2;
        public static final int kLEncoderBDIO = 3;
        public static final int kREncoderADIO = 0;
        public static final int kREncoderBDIO = 1;
        public static final boolean kLEncoderReversed = false;
        public static final boolean kREncoderReversed = true;
        public static final double kEncoderDistancePerPulse = (2*Math.PI*kWheelRadiusMeters)/(double)kEncoderCPR; //Confirm this value

        // Robot Characterization
        public static final double ksVolts = 0.16536;
        public static final double kvVoltSecondsPerMeter = 2.2165;
        public static final double kaVoltSecondsSquaredPerMeter = 0.76152;
        public static final double kPDriveVel = 0.013045;

        // Drivetrain Motor Configuration
        public static final int kCurrentLimit = 40;
        public static final boolean kLInvertMotor = true;
        public static final boolean kRInvertMotor = true;

  
    }

    public static final class ArmConstants{
        public static final int kArmCurrentLimit = 40;

        public static final double kArmP = 0.05;
        public static final double kArmI = 0.0;
        public static final double kArmD = 0.0;

        public static final int kArmFrontSparkMAXID = 10;
        public static final int kArmRearSparkMAXID = 19;

        public static final int kArmEncoderADIO = 4;
        public static final int kArmEncoderBDIO = 5;
        public static final boolean kArmEncoderReversed = true;
        public static final double kEncoderCPRtoDeg = 81.093;
    }

    public static final class WristConstants{
        public static final int kWristCurrentLimit = 40;

        public static final int kWristSparkMAXID = 11;

        public static final double kWristP = 0.010;
        public static final double kWristI = 0.0;
        public static final double kWristD = 0.0;

        public static final double kEncoderCPRtoDeg = 360.0/75.0;
    }

    public static final class ClawConstants{
        public static final int kClawSparkMAXID = 12;
        public static final int kClawCurrentLimit = 20;
    }

    public static final class PneumaticsConstants{
        public static final double kMinPressure = 60.0;
        public static final double kMaxPressure = 120.0;
    }


    public static final class OIConstants{
        public static final int kDriverLeftPort = 0;
        public static final int kDriverRightPort = 1;

        public static final int kOperatorPort = 2;
    }

    public static final class AutoConstants{
        public static final double kAutoMaxVolt = 10.0;
        public static final double kMaxSpeedMetersPerSecond = 3.0;
        public static final double kMaxAccelerationMetersPerSecondSquard = 1;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}