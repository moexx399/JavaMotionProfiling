// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Scanner;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_Drivetrain = new Drivetrain();

  //private final DriveTrajectory m_autoCommand = new DriveTrajectory(m_Drivetrain);
  

  private final XboxController m_controller = new XboxController(0);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Assign default commands
    m_Drivetrain.setDefaultCommand(new DriveWithJoysticks(m_Drivetrain, m_controller));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    //Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    /// Pathwaever ////////////////////////////////
    Trajectory trajectory = new Trajectory();
    try {
      File trajectoryJSON = new File(
        Filesystem.getDeployDirectory().toPath().resolve(
          "paths/output/Autos/"+SmartDashboard.getString("Auto Selector", "Do Nothing"))
          .toString()
      );
      Scanner myReader = new Scanner(trajectoryJSON);
      while (myReader.hasNextLine()) {
        String data = myReader.nextLine();
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/output/" + data.substring(0, data.length() - 5)+".wpilib.json");
        Trajectory temp_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        trajectory = trajectory.concatenate(temp_trajectory);
        }
        myReader.close();
        Field2d m_fieldSim = (Field2d) SmartDashboard.getData("Field");
        m_fieldSim.getObject("traj").setTrajectory(trajectory);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + SmartDashboard.getString("Auto Selector", "Do Nothing"), ex.getStackTrace());
    }
    /////////////////////////////////////

    /// Pathplanner ////////////////////
    // Trajectory trajectory = new Trajectory();
    // try {
    //   String m_auto = SmartDashboard.getString("Auto Selector", "Do Nothing");
    //   trajectory = PathPlanner.loadPath(m_auto, 3, 2);
    //   Field2d m_fieldSim = (Field2d) SmartDashboard.getData("Field");
    //   m_fieldSim.getObject("traj").setTrajectory(trajectory);
    // } catch (Exception ex) {
    //   DriverStation.reportError("Unable to open trajectory: ", ex.getStackTrace());
    //   String m_auto = "Do Nothing";
    //   trajectory = PathPlanner.loadPath(m_auto, 3, 2);
    //   Field2d m_fieldSim = (Field2d) SmartDashboard.getData("Field");
    //   m_fieldSim.getObject("traj").setTrajectory(trajectory);
    // }
    /////////////////////////////////////

    

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        m_Drivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_Drivetrain::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, DriveConstants.kIDriveVel, DriveConstants.kDDriveVel),
        new PIDController(DriveConstants.kPDriveVel, DriveConstants.kIDriveVel, DriveConstants.kDDriveVel),
        // RamseteCommand passes volts to the callback
        m_Drivetrain::tankDriveVolts,
        m_Drivetrain
    );

    // Reset odometry to the starting pose of the trajectory.
    m_Drivetrain.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_Drivetrain.tankDriveVolts(0, 0));
  }
}
