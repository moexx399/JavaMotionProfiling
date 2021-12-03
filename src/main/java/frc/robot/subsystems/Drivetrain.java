// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.numbers.N2;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

   // 3 meters per second.
   public static final double kMaxSpeed = 3.0;
   // 1/2 rotation per second.
   public static final double kMaxAngularSpeed = Math.PI;
 
   private static final double kTrackWidth = 0.381 * 2;
   private static final double kWheelRadius = 0.0508;
   private static final int kEncoderResolution = -4096;
 
   private final PWMVictorSPX m_leftLeader = new PWMVictorSPX(1);
   private final PWMVictorSPX m_leftFollower = new PWMVictorSPX(2);
   private final PWMVictorSPX m_rightLeader = new PWMVictorSPX(3);
   private final PWMVictorSPX m_rightFollower = new PWMVictorSPX(4);
 
   private final MotorControllerGroup m_leftGroup =
       new MotorControllerGroup(m_leftLeader, m_leftFollower);
   private final MotorControllerGroup m_rightGroup =
       new MotorControllerGroup(m_rightLeader, m_rightFollower);

   // The robot's drive
   private final DifferentialDrive m_drive = new DifferentialDrive(m_leftGroup, m_rightGroup);
 
   //private final Encoder m_leftEncoder = new Encoder(0, 1);
   //private final Encoder m_rightEncoder = new Encoder(2, 3);

   private final Encoder m_leftEncoder =
      new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1],
                  DriveConstants.kLeftEncoderReversed);

   // The right-side drive encoder
   private final Encoder m_rightEncoder =
      new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1],
                  DriveConstants.kRightEncoderReversed);
                  
 
   private final PIDController m_leftPIDController = new PIDController(8.5, 0, 0);
   private final PIDController m_rightPIDController = new PIDController(8.5, 0, 0);
 
   private final AnalogGyro m_gyro = new AnalogGyro(0);
 
   private final DifferentialDriveKinematics m_kinematics =
       new DifferentialDriveKinematics(kTrackWidth);
   private final DifferentialDriveOdometry m_odometry =
       new DifferentialDriveOdometry(m_gyro.getRotation2d());
 
   // Gains are for example purposes only - must be determined for your own
   // robot!
   private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
 
   // Simulation classes help us simulate our robot
   private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
   private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
   private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
   private final Field2d m_fieldSim = new Field2d();
   private final LinearSystem<N2, N2, N2> m_drivetrainSystem =
       LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
   private final DifferentialDrivetrainSim m_drivetrainSimulator =
       new DifferentialDrivetrainSim(
           m_drivetrainSystem, DCMotor.getCIM(2), 8, kTrackWidth, kWheelRadius, null);

           
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    //m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_rightGroup.setInverted(true);
    SmartDashboard.putData("Field", m_fieldSim);
  }

  @Override
  public void periodic() {
    updateOdometry();
    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putNumber("Encoder left Rate", m_leftEncoder.getRate());
    SmartDashboard.putNumber("Encoder right Rate", m_rightEncoder.getRate());
  }

  @Override
    public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(
      m_leftLeader.get() * RobotController.getInputVoltage(),
      -m_rightLeader.get() * RobotController.getInputVoltage());
  m_drivetrainSimulator.update(0.02);

  m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
  m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
  m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
  m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
  m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
      var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
      var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
      double leftOutput =
          m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
      double rightOutput =
          m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
  
      m_leftGroup.setVoltage(leftOutput + leftFeedforward);
      m_rightGroup.setVoltage(rightOutput + rightFeedforward);
    }
  
    public void drive(double xSpeed, double rot) {
      setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
    }
  
    public void updateOdometry() {
      m_odometry.update(
          m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    }
  
    public void resetOdometry(Pose2d pose) {
      m_leftEncoder.reset();
      m_rightEncoder.reset();
      m_drivetrainSimulator.setPose(pose);
      m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }
  
    public Pose2d getPose() {
      return m_odometry.getPoseMeters();
    }

    public double getVelocity(){
      return m_leftEncoder.getRate();
    }


    /**
     * Added from the Trajectory walk Through
     */

     /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    //m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    SmartDashboard.putNumber("leftVolts", leftVolts);
    SmartDashboard.putNumber("rightVolts", rightVolts);
    m_leftGroup.setVoltage(leftVolts);
    m_rightGroup.setVoltage(rightVolts);
    m_drive.feed();
  }


}
