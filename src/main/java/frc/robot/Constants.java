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
        public static final int kLFSparkCANID = 21;
        public static final int kLRSparkCANID = 19;
        public static final int kRFSparkCANID = 10;
        public static final int kRRSparkCANID = 22;

        // Drivetrain Encoder Information
        public static final int kEncoderCPR = 2048;
        public static final int kLEncoderADIO = 0;
        public static final int kLEncoderBDIO = 1;
        public static final int kREncoderADIO = 2;
        public static final int kREncoderBDIO = 3;
        public static final boolean kLEncoderReversed = false;
        public static final boolean kREncoderReversed = true;
        public static final double kEncoderDistancePerPulse = (2*Math.PI*kWheelRadiusMeters)/(double)kEncoderCPR; //Confirm this value

        // Robot Characterization
        public static final double ksVolts = 0.0;
        public static final double kvVoltSecondsPerMeter = 0.0;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0;
        public static final double kPDriveVel = 0.0;

        // Drivetrain Motor Configuration
        public static final int kCurrentLimit = 40;
        public static final boolean kLInvertMotor = false;
        public static final boolean kRInvertMotor = true;

  
    }

    public static final class OIConstants{
        public static final int kDriverLeftPort = 0;
        public static final int kDriverRightPort = 1;

        public static final int kOperatorPort = 2;
    }

    public static final class AutoConstants{
        public static final double kAutoMaxVolt = 10.0;
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquard = 1;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}