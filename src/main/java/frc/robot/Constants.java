package frc.robot;

public final class Constants {

    public static final class DriveConstants{
        // Spark Max CAN IDs
        public static final int kLFSparkCANID = 21;
        public static final int kLRSparkCANID = 19;
        public static final int kRFSparkCANID = 10;
        public static final int kRRSparkCANID = 22;

        // Drivetrain Encoder Information
        public static final int kLEncoderADIO = 0;
        public static final int kLEncoderBDIO = 1;
        public static final int kREncoderADIO = 2;
        public static final int kREncoderBDIO = 3;
        public static final boolean kLEncoderReversed = false;
        public static final boolean kREncoderReversed = true;
        public static final double kEncoderDistancePerPulse = 0.00919921875; //Confirm this value

        // Drivetrain Motor Configuration
        public static final int kCurrentLimit = 40;
        public static final boolean kLInvertMotor = false;
        public static final boolean kRInvertMotor = true;

        // Drivetrain Physical Parameters
        public static final double kWheelRadius = 3.0;
        public static final double kGearboxRatio = 10.75;

        
    }
}