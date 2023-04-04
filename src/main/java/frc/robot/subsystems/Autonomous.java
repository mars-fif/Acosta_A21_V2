package frc.robot.subsystems;

import com.pathplanner.lib.PathPlanner;

import java.util.Enumeration;
import java.util.Hashtable;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoCommands.DriveOut;
import frc.robot.commands.AutoCommands.ScoreHighCube;
import frc.robot.commands.AutoCommands.TwoCone;
import frc.robot.commands.AutoCommands.ChargeStation;
import frc.robot.commands.AutoCommands.ConeChargeStation;

public class Autonomous extends SubsystemBase{
    private final Drivetrain drivetrain;
    private static Autonomous autonomous;

    private SendableChooser<Command> autoRoutineChooser;
    private Hashtable<String,Command> autoRoutines;

    private Trajectory Mobility, ReturnLeft, ChargeStation;

    public Autonomous(){
        autoRoutines = new Hashtable<String,Command>();
        autoRoutineChooser = new SendableChooser<Command>();

        drivetrain = Drivetrain.getInstance();

        defineAutoPaths();
        setupAutoRoutines();
        setupAutoSelector();
    }

    public static Autonomous getInstance(){
        if(autonomous == null){
            autonomous = new Autonomous();
        }

        return autonomous;
    }

    public void setupAutoSelector(){
        Enumeration<String> e = autoRoutines.keys();

        while(e.hasMoreElements()){
            String autoRoutineName = e.nextElement();
            autoRoutineChooser.addOption(autoRoutineName, autoRoutines.get(autoRoutineName));
        };

        SmartDashboard.putData("Auto Routines", autoRoutineChooser);
    }

    public void setupAutoRoutines(){
        autoRoutines.put("Mobility", new DriveOut(Mobility.getInitialPose(), createCommandFromTrajectory(Mobility)));
        autoRoutines.put("ScoreHighCone", new ScoreHighCube(Mobility.getInitialPose(), createCommandFromTrajectory(Mobility)));
        autoRoutines.put("ChargeStation", new ChargeStation(ChargeStation.getInitialPose(), createCommandFromTrajectory(ChargeStation)));
        autoRoutines.put("Cone Charge Station", new ConeChargeStation(ChargeStation.getInitialPose(), createCommandFromTrajectory(ChargeStation)));
        //autoRoutines.put("TwoCone", new TwoCone(Mobility.getInitialPose(), createCommandFromTrajectory(Mobility), createCommandFromTrajectory(ReturnLeft)));
    }

    public Command returnAutonomousCommand(){
        return autoRoutineChooser.getSelected();
    }

    private void defineAutoPaths(){
        Mobility = PathPlanner.loadPath("Forward Out", AutoConstants.kMaxSpeedMetersPerSecond,AutoConstants.kMaxAccelerationMetersPerSecondSquard);
        ReturnLeft = PathPlanner.loadPath("TwoCone", AutoConstants.kMaxSpeedMetersPerSecond,AutoConstants.kMaxAccelerationMetersPerSecondSquard);
        ChargeStation = PathPlanner.loadPath("Charge Station", 1.5*AutoConstants.kMaxSpeedMetersPerSecond,1.5*AutoConstants.kMaxAccelerationMetersPerSecondSquard);
    }

    public RamseteCommand createCommandFromTrajectory(Trajectory trajectory){
        var RamseteController = new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta);

        RamseteCommand autoCommand = 
        new RamseteCommand(
            trajectory, drivetrain::getPose, 
            RamseteController,
            new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter), 
            DriveConstants.kDriveKinematics, drivetrain::getWheelSpeeds, 
            new PIDController(DriveConstants.kPDriveVel, 0, 0), 
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
             drivetrain::tankDriveVolts,
             drivetrain);

        return autoCommand;
    }
    
    public Trajectory getTransformedTrajectory(Trajectory t){
        Pose2d newOrigin = t.getInitialPose();
        Trajectory transformed = t.relativeTo(newOrigin);
        return transformed;
    }
}
