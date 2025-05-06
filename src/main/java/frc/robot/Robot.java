package frc.robot;

import java.util.ArrayList;

import org.ietf.jgss.Oid;
import org.littletonrobotics.junction.LoggedRobot;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.DomainExpansion.AutoAllignL;
import frc.robot.Commands.DomainExpansion.AutoAllignR;

import frc.robot.Commands.AlignToCoral;


//import frc.robot.subsystems.claw.Wrist.WristState;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.swerve.MAXSwerveModule;
import frc.robot.subsystems.swerve.DriveSubsystem.DriveState;
import frc.robot.subsystems.vision.CameraSystem;
import frc.robot.subsystems.vision.CameraSystem.PoleSide;

public class Robot extends LoggedRobot {
  // all the initialing for the systems
  public double speedMod;
  public double clawZeroPower;
  private DriveSubsystem drivebase;
 
  private CameraSystem camSystem;
 
  private AutoAllignR autoAllignR;
  private AutoAllignL autoAllignL;
  private AlignToCoral alignToCoral;
  // private LED litty;
  // private CameraSystem camSystem;
  private String mode = "coral";
  public double rumbleTimer;
  public double switchTimer;
  public double transTimer;
  public RobotConfig Rconfig;
  private static XboxController driver;
  private static XboxController operator;
  // initialization of the auton chooser in the dashboard
  private Command m_autoSelected;
  Double targetRange = null;
  Double targetAngle = null;
  double invert = 1;
  
  public PIDController xController;
  public PIDController yController;
  public PIDController thetaController;

  private boolean usingAlign;
  private boolean atPoleR;
  // that is a chooser for the autons utilizing the sendableChooser which allows
  // us to choose the auton commands
  private SendableChooser<Command> m_chooser = new SendableChooser<>();
  

  @Override
  public void robotInit() {
    clawZeroPower = .075;
    speedMod = 1;
    drivebase = DriveSubsystem.getInstance();
    // litty = LED.getInstance();
   
    camSystem = CameraSystem.getInstance(); 
    camSystem.AddCamera(new PhotonCamera("ClimbCam"), new Transform3d(
      new Translation3d(-0.30043, -0.26457, 0.31945), new Rotation3d(0.0, 0.0, Math.toRadians(-55.56095))), 
      true);
    camSystem.AddCamera(new PhotonCamera("SwerveCam"), new Transform3d(
      new Translation3d(0.28831, -0.2421, 0.29561), new Rotation3d(0.0, Math.toRadians(2.5), Math.toRadians(-105))), 
      true);
      // camSystem.AddCamera(new PhotonCamera("MiddleCam"), new Transform3d(
      //   new Translation3d(0.00833, -0.22138, 0.14534), new Rotation3d(0.0, 0.0, Math.toRadians(-90))), 
      //   true);
    
    
    autoAllignR = new AutoAllignR();
    autoAllignL = new AutoAllignL();

    
    //alignToCoral = new AlignToCoral();
    // x: .0095 y: .95 theta: .008

    xController = new PIDController(.0095, 0, 0);
    yController = new PIDController(2.15 , 0, 0);
    thetaController = new PIDController(0.015, 0, 0);

    thetaController.enableContinuousInput(0, 360);

    xController.setTolerance(.005);
    yController.setTolerance(.005);
    thetaController.setTolerance(.01);

    atPoleR = false;
    // camSystem = CameraSystem.getInstance();
    // camSystem.AddCamera(new PhotonCamera("Cam1"), new Transform3d(
    // new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0))
    // , true);

    // camSystem.AddCamera(new PhotonCamera("Cam2"), new Transform3d(
    // new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0)),
    // true);

    driver = new XboxController(0);
    operator = new XboxController(1);
    
   

    NamedCommands.registerCommand("AutoAllignR", autoAllignR);
    NamedCommands.registerCommand("AutoAllignL", autoAllignL);
    NamedCommands.registerCommand("CoralAllignL", new AlignToCoral(PoleSide.LEFT));
    NamedCommands.registerCommand("CoralAlignR", new AlignToCoral(PoleSide.RIGHT));
    m_chooser.addOption("middlePath", new PathPlannerAuto("middlePath"));
    m_chooser.addOption("1_C_1_P1C align", new PathPlannerAuto("1_C_1_P1C align"));
    m_chooser.addOption("testReal", new PathPlannerAuto("testReal"));
    m_chooser.addOption("test2", new PathPlannerAuto("test2"));
    m_chooser.addOption("testcoral", new PathPlannerAuto("testcoral"));
    m_chooser.addOption("3_C_2_P2C align", new PathPlannerAuto("3_C_2_P2C align"));
    // SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putData(m_chooser);
    
  }

  @Override
  public void robotPeriodic() {
    


    SmartDashboard.putString("intakemode", mode);


    

    SmartDashboard.putNumber("Last Tag Seen", camSystem.lastTag);
    SmartDashboard.putNumber("Desired Degree", 
    CameraSystem.aprilTagFieldLayout.getTagPose(18).get().getRotation().toRotation2d().getDegrees());
    SmartDashboard.putNumber("Currenr Degree", DriveSubsystem.poseEstimator.getEstimatedPosition().getRotation().getDegrees());

    if(camSystem.getTargetRange(1, camSystem.lastTag) != null && camSystem.getYawForTag(1, camSystem.lastTag)!= null)
    {
      SmartDashboard.putNumber("SwerveCam dist", camSystem.getTargetRange(1, camSystem.lastTag));
      SmartDashboard.putNumber("SwerveCam yaw", camSystem.getYawForTag(1, camSystem.lastTag));
      SmartDashboard.putNumber("SwerveCam Pitch", camSystem.getPitch(1, camSystem.lastTag));
    }
    if(camSystem.getTargetRange(0, camSystem.lastTag) != null && camSystem.getYawForTag(0, camSystem.lastTag)!= null)
    {
      SmartDashboard.putNumber("ClimbCam dist", camSystem.getTargetRange(0, camSystem.lastTag));
      SmartDashboard.putNumber("ClimbCam yaw", camSystem.getYawForTag(0, camSystem.lastTag));
      SmartDashboard.putNumber("CLimbCam Pitch", camSystem.getPitch(0, camSystem.lastTag));
    }
    SmartDashboard.putNumber("heading", drivebase.getWorkingHeading());
    if(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag) != null
    && camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag) != null)
    {
      SmartDashboard.putNumber("xController", xController.calculate(
        camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag).doubleValue()));
      SmartDashboard.putNumber("yController", yController.calculate(
        camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue()));
      SmartDashboard.putNumber("thetaController", thetaController.calculate(drivebase.getWorkingHeading()));
    }
    
    SmartDashboard.putString("Focus Cam", (camSystem.focusCamIndex == 0)  
    ? "climb" : "swerve");
  
    
   
   

 

    CommandScheduler.getInstance().run();
    drivebase.periodic();



    // putting all of the info from the subsystems into the dashvoard so we can test
    // things
    
    SmartDashboard.putNumber("Gyro Angle:", (drivebase.getHeading() + 90) % 360);
    SmartDashboard.putNumber("X-coordinate", drivebase.getPose().getX());
    SmartDashboard.putNumber("Y-coordinate", drivebase.getPose().getY());

  }

  @Override
  public void autonomousInit() {
    robotInit();
    
    // getting the value we chose from the dashboard and putting it into motion in
    // the auton
   
    m_autoSelected = m_chooser.getSelected();

     //drivebase.resetOdometry(PathPlannerAuto.getStartingPoseFromAutoFile(m_chooser.getSelected().getName()));

    // schedules the command so it actually begins moving
    if (m_autoSelected != null) {
      m_autoSelected.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // updating the intake for the autointake command
    // using cameras to calculate the robot position instead of odometry.
    // we use a mix of odometry + camera positions to calculate the robot position
    // Pose2d cameraPosition = camSystem.calculateRobotPosition();
    // Pose2d pose = drivebase.updateOdometry(cameraPosition);

    // SmartDashboard.putNumber("Auto X", drivebase.getPose().getX());
    // SmartDashboard.putNumber("Auto Y", drivebase.getPose().getY());
    // SmartDashboard.putNumber("Odometry X", pose.getX());
    // SmartDashboard.putNumber("Odometry Y", pose.getY());
    drivebase.periodic();
    camSystem.updateLatestResult(false);
  }

  @Override
  public void teleopInit() {
    robotInit();
    // as soon as we begin teleop we desable the auton selection
    // litty.setBlue();
    if (m_autoSelected != null) {
      m_autoSelected.cancel();
    }

    Translation2d testxy = new Translation2d(16.57 - 14.7, 5.54);
    Rotation2d testRot = new Rotation2d(0);
    Pose2d test = new Pose2d(testxy, testRot);
    // drivebase.resetOdometry(test);
  }

  @Override
  public void teleopPeriodic() {
    boolean atRight = false;
    double multFactor = 1;
    usingAlign = false;
   
    
    double ySpeed = drivebase.inputDeadband(-driver.getLeftX()) * speedMod;
    double xSpeed = drivebase.inputDeadband(-driver.getLeftY()) * speedMod;
    double rot = drivebase.inputDeadband(-driver.getRightX()) * speedMod;
    if(driver.getRightTriggerAxis() >= .5){
      speedMod = .35;
    }else if(driver.getLeftTriggerAxis() >= .5){
      speedMod = .1;
    }else{
      speedMod = 1;
    }
    camSystem.updateLatestResult(driver.getBButton() || driver.getXButton());
   
    if (driver.getPOV() == 0) {
      drivebase.zeroHeading();
    }
    

    if(driver.getBButton()){
      usingAlign = true;
      camSystem.poleSide = PoleSide.RIGHT;
      thetaController.setSetpoint(0);
      //was .11
      yController.setSetpoint(0.06);
      xController.setSetpoint(-1.9);
      camSystem.focusCamIndex = 1;
      if (camSystem.hasDesiredTarget(0, camSystem.lastTag) && camSystem.hasDesiredTarget(1, camSystem.lastTag)) 
      // driver align right so left camera
      {
        updateThetaControllerSetpoint(camSystem.lastTag);
        // if(yController.calculate(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue()) < .55
        // && camSystem.focusCamIndex == 0){
        //   xController.setSetpoint(-4.5);
        // }

        if(camSystem.focusCamIndex == 0 && camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag) != null
        && yController.calculate(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue()) < .20)
        { 
          multFactor = 1;
          xController.setSetpoint(-7.1);
          xSpeed = xController.calculate(camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag));
          ySpeed = yController.calculate(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue());
        }
        else{
          xSpeed = xController.calculate(camSystem.getYawForTag(1, camSystem.lastTag));
          ySpeed = yController.calculate(camSystem.getTargetRange(1, camSystem.lastTag).doubleValue());
          multFactor = 2.4;
        }
        multFactor = .7;
        rot = thetaController.calculate(drivebase.getWorkingHeading());
        // drivebase.drive(xSpeed1,
        //   multFactor * ySpeed1, 
        //   thetaController.calculate(drivebase.getWorkingHeading()),
        //   //0, 
        //   false);
      }

      else if((camSystem.hasDesiredTarget(camSystem.focusCamIndex, camSystem.lastTag))
      || (camSystem.focusCamIndex == 0 && camSystem.hasDesiredTarget(1, camSystem.lastTag))){
        updateThetaControllerSetpoint(camSystem.lastTag);

        if(camSystem.focusCamIndex == 0 && camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag) != null
        && yController.calculate(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue()) < .20)
        { 
          multFactor = 1;
          xController.setSetpoint(-7.1);
          xSpeed = xController.calculate(camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag));
          ySpeed = yController.calculate(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue());
        }
        else{
          xSpeed = xController.calculate(camSystem.getYawForTag(1, camSystem.lastTag));
          ySpeed = yController.calculate(camSystem.getTargetRange(1, camSystem.lastTag).doubleValue());
          multFactor = 2.4;
        }
        multFactor = .7;
        rot = thetaController.calculate(drivebase.getWorkingHeading());
        // drivebase.drive(xSpeed1,
        //   multFactor * ySpeed1, 
        //   thetaController.calculate(drivebase.getWorkingHeading()),
        //   //0, 
        //   false);
        }
    }
    
    
    if(driver.getXButton()){
      usingAlign = true;
      camSystem.poleSide = PoleSide.LEFT;
      //thetaController.setSetpoint(0);
      //was .11
      //yController.setSetpoint(0.07);
      xController.setSetpoint(-18.8);
      camSystem.focusCamIndex = 0;
      if(camSystem.focusCamIndex == 0 && camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag) == null){
        xSpeed = .1;
        ySpeed = 0;
        rot = 0;
      }
      else if(camSystem.focusCamIndex == 0 && camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag) != null){
        xSpeed =  xController.calculate(camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag));
        ySpeed = 0;
        rot = 0;
      }
    
    drivebase.drive(xSpeed, multFactor * ySpeed, rot, !usingAlign);
    }
}
  
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
  private void updateThetaControllerSetpoint(int targetID) {
    switch (targetID) {
      case 6, 19 -> thetaController.setSetpoint(30);
      case 7, 18 -> thetaController.setSetpoint(90);
      case 8, 17 -> thetaController.setSetpoint(150);
      case 9, 22 -> thetaController.setSetpoint(210);
      case 10, 21 -> thetaController.setSetpoint(270);
      case 11, 20 -> thetaController.setSetpoint(330);
    }

    PPHolonomicDriveController.clearXFeedbackOverride();
    PPHolonomicDriveController.clearYFeedbackOverride();
    PPHolonomicDriveController.clearRotationFeedbackOverride();
  }
  private void updateXControllerSetpoint(){
    if(camSystem.focusCamIndex == 0 && camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag) != null){
      xController.setSetpoint(-5.8 + (-15 * .13)/ camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag));
    }
    else{
      xController.setSetpoint(-20.8);
    }
    
  }
}
