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

public class Autonomous extends SubsystemBase{
    private final Drivetrain drivetrain;
    private static Autonomous autonomous;

    private SendableChooser<Command> autoRoutineChooser;
    private Hashtable<String,Command> autoRoutines;

    private Trajectory exampleTrajectory, exampleTrajectory2, fowardOutLong;

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
        autoRoutines.put("Forward Out", new DriveOut(exampleTrajectory.getInitialPose(), createCommandFromTrajectory(exampleTrajectory)));
        autoRoutines.put("Forward Left", new DriveOut(exampleTrajectory2.getInitialPose(), createCommandFromTrajectory(exampleTrajectory2)));
        autoRoutines.put("Forward Out Long", new DriveOut(fowardOutLong.getInitialPose(), createCommandFromTrajectory(fowardOutLong)));
        autoRoutines.put("Score High Cube", new ScoreHighCube(exampleTrajectory.getInitialPose(), createCommandFromTrajectory(exampleTrajectory)));
    }

    public Command returnAutonomousCommand(){
        return autoRoutineChooser.getSelected();
    }

    private void defineAutoPaths(){
        exampleTrajectory = PathPlanner.loadPath("Forward Out", AutoConstants.kMaxSpeedMetersPerSecond,AutoConstants.kMaxAccelerationMetersPerSecondSquard);
        exampleTrajectory2= PathPlanner.loadPath("Forward Left", AutoConstants.kMaxSpeedMetersPerSecond,AutoConstants.kMaxAccelerationMetersPerSecondSquard);
        fowardOutLong = PathPlanner.loadPath("Forward Out Long",  AutoConstants.kMaxSpeedMetersPerSecond,AutoConstants.kMaxAccelerationMetersPerSecondSquard);
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
