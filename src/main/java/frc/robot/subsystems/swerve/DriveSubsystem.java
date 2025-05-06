// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Configs;
import frc.robot.subsystems.vision.CameraSystem;
import frc.robot.Ports;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.*;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

public class DriveSubsystem extends SubsystemBase {
  public enum DriveState {
    NORMAL(1),
    SLOW(0.2);

    public double driveSpeed;

    private DriveState(double driveSpeed) {
      this.driveSpeed = driveSpeed;
    }
  }
  private DriveState driveState = DriveState.NORMAL;

  CameraSystem system = CameraSystem.getInstance();
  // Create MAXSwerveModules
  private static DriveSubsystem instance;
  private final MAXSwerveModule frontLeft = new MAXSwerveModule(
      Ports.frontLeftDrive,
      Ports.frontLeftSteer,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule frontRight = new MAXSwerveModule(
     Ports.frontRightDrive,
      Ports.frontRightSteer,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule rearLeft = new MAXSwerveModule(
      Ports.backLeftDrive,
      Ports.backLeftSteer,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule rearRight = new MAXSwerveModule(
      Ports.backRightDrive,
      Ports.backRightSteer,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  public final AHRS gyro = new AHRS(SPI.Port.kMXP);
  // private final ADIS16470_IMU gyro = new ADIS16470_IMU();
  public static SwerveDrivePoseEstimator poseEstimator;

  private ProfiledPIDController headingController;
  private PPHolonomicDriveController config;
  private RobotConfig Rconfig;


   private Field2d field = new Field2d();
  private Field2d visionField = new Field2d();

  // Odometry class for tracking robot pose
  // SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
  //     DriveConstants.kDriveKinematics,
  //     Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
  //     new SwerveModulePosition[] {
  //         m_frontLeft.getPosition(),
  //         m_frontRight.getPosition(),
  //         m_rearLeft.getPosition(),
  //         m_rearRight.getPosition()
  //     });
  

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() 
  {
    // Usage reporting for MAXSwerve template
    
    // gyro.setAngleAdjustment(90);
    // gyro.zeroYaw();
    //ANSH added
    poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(-gyro.getAngle()),
        getPositions(), new Pose2d());

    config = new PPHolonomicDriveController(new PIDConstants(0, 0, 0),
      new PIDConstants(2, 0.0, 0.0));

      //15,1.2 and 1.1

   
   
    try{
      Rconfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }


     // Auto builder basically sets up the Autonomous file and in the resets
     AutoBuilder.configure(this::getPose, this::resetOdometry, this::getSpeeds, this::setAutoSpeeds, config, Rconfig,
     shouldFlipPath(), this);
    //AutoBuilder.configure(null, null, null, null, config, Rconfig, null, null);

    SmartDashboard.putData("Orig Field", field);
    SmartDashboard.putData("Vision Field", visionField);

    PathPlannerLogging.setLogActivePathCallback(
        poses -> field.getObject("pathplanner path poses").setPoses(poses));
    PathPlannerLogging.setLogTargetPoseCallback(
        targ -> field.getObject("pathplanner targ pose").setPose(targ));
    PathPlannerLogging.setLogCurrentPoseCallback(
        curr -> field.getObject("pathplanner curr pose").setPose(curr));
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    system.isBlueSide = !shouldFlipPath().getAsBoolean();
  }
  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        frontLeft.getState(),
        frontRight.getState(),
        rearLeft.getState(),
        rearRight.getState()
    };
  }
  public void 
  setAutoSpeeds(ChassisSpeeds input) {
    var speeds = ChassisSpeeds.discretize(input, 0.02);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    frontLeft.setAutoSpeeds(swerveModuleStates[0]);
    frontRight.setAutoSpeeds(swerveModuleStates[1]);
    rearLeft.setAutoSpeeds(swerveModuleStates[2]);
    rearRight.setAutoSpeeds(swerveModuleStates[3]);
  }
  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        rearLeft.getPosition(),
        rearRight.getPosition()
    };
  }
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    // m_odometry.update(
    //     Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
    //     new SwerveModulePosition[] {
    //         m_frontLeft.getPosition(),
    //         m_frontRight.getPosition(),
    //         m_rearLeft.getPosition(),
    //         m_rearRight.getPosition()
    //     });
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), Rotation2d.fromDegrees(getHeading()), getPositions());
    field.setRobotPose(getPose());
    if(system.hasTargets())
    {
      system.AddVisionMeasurements(poseEstimator, visionField);
    }
    SmartDashboard.putNumber("FrontLeft", frontLeft.getEncoder().getPosition());
    SmartDashboard.putNumber("FrontRight", frontRight.getEncoder().getPosition());
    SmartDashboard.putNumber("RearLeft", rearLeft.getEncoder().getPosition());
    SmartDashboard.putNumber("RearRight", rearRight.getEncoder().getPosition());
  }
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    //return m_odometry.getPoseMeters();
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    // m_odometry.resetPosition(
    //     Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
    //     new SwerveModulePosition[] {
    //         m_frontLeft.getPosition(),
    //         m_frontRight.getPosition(),
    //         m_rearLeft.getPosition(),
    //         m_rearRight.getPosition()
    //     },
    //     pose);
    poseEstimator.resetPosition(Rotation2d.fromDegrees(-gyro.getAngle()), getPositions(), pose);
  }
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(-gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public double inputDeadband(double input){
    return Math.abs(input)>0.25? input : 0;
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }
  public BooleanSupplier shouldFlipPath() {
    return new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }
    };
  }
  public void printTranslationalVelocities() {
    SmartDashboard.putNumber("Front Left TranslationalVelo", frontLeft.getTranslationalVelocity());
    SmartDashboard.putNumber("Front Right TranslationalVelo", frontRight.getTranslationalVelocity());
    SmartDashboard.putNumber("Back Left TranslationalVelo", rearLeft.getTranslationalVelocity());
    SmartDashboard.putNumber("Back Right TranslationalVelo", rearRight.getTranslationalVelocity());

  }
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle()).getDegrees();
  }
  public double getWorkingHeading(){
    if(Rotation2d.fromDegrees(-gyro.getAngle()).getDegrees() >= 0)
      return (Rotation2d.fromDegrees(-gyro.getAngle()).getDegrees()) % 360;
    else
    return 360 - Math.abs(Rotation2d.fromDegrees(-gyro.getAngle()).getDegrees() % 360);
  }
  public void setDriveState(DriveState state) {
    driveState = state;
  }
  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  public static DriveSubsystem getInstance(){
    if (instance == null){
      instance = new DriveSubsystem();
    }
    return instance;
  }
}