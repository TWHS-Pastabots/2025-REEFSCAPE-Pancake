// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.CameraSystem;
import frc.robot.subsystems.vision.CameraSystem.PoleSide;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToCoral extends Command {
  /** Creates a new AlignToCoral. */
  private CameraSystem camSystem;
  private DriveSubsystem driveBase;

  private PoleSide side;
  private boolean ended;
  public PIDController xController;
  public PIDController yController;
  public PIDController thetaController;
  private double heading;
  public AlignToCoral(PoleSide sideNew) {
    camSystem = CameraSystem.getInstance();
    driveBase = DriveSubsystem.getInstance();
    this.side = sideNew;
    xController = new PIDController(.0095, 0, 0);
    yController = new PIDController(2.15 , 0, 0);
    thetaController = new PIDController(0.015, 0, 0);
    //heading = -driveBase.gyro.getAngle() + 90;
    //heading = driveBase.getHeading() % 180;
    //xController.setSetpoint(0.11);//temporary number
    thetaController.setSetpoint(0);
    if(side == PoleSide.LEFT){
      yController.setSetpoint(0.185);
      xController.setSetpoint(14);
      //xController.setSetpoint(15);
      //xController.setSetpoint(-4.5);
      camSystem.focusCamIndex = 0;
    }
    else {
      yController.setSetpoint(0.06);
      xController.setSetpoint(-1.7);
      camSystem.focusCamIndex = 1;
    }
    
    thetaController.enableContinuousInput(0, 360);

    xController.setTolerance(.01);
    yController.setTolerance(.01);
    thetaController.setTolerance(.1);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(side == PoleSide.LEFT){
      yController.setSetpoint(0.13);
      xController.setSetpoint(14);
      //xController.setSetpoint(15);
      //xController.setSetpoint(-4.5);
      camSystem.focusCamIndex = 0;
    }
    else {
      yController.setSetpoint(0.06);
      xController.setSetpoint(-1.7);
      camSystem.focusCamIndex = 1;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (camSystem.hasDesiredTarget(0, camSystem.lastTag) && camSystem.hasDesiredTarget(1, camSystem.lastTag)
    && camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag) != null 
    && camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag) != null) // driver align right so left camera
    {
      updateThetaControllerSetpoint(camSystem.lastTag);
      // if(yController.calculate(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue()) < .55
      // && camSystem.focusCamIndex == 0){
      //   xController.setSetpoint(-4.5);
      // }
      double xSpeed = 0;
      double ySpeed = 0;
      double multFactor = 1;
      if(camSystem.focusCamIndex == 0 && camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag) != null
      && yController.calculate(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue()) < .20)
      { 
        xController.setSetpoint(-20.8);
        xSpeed = xController.calculate(camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag));
        ySpeed = yController.calculate(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue());
        multFactor = 0.8;
      }
      else{
        xSpeed = xController.calculate(camSystem.getYawForTag(1, camSystem.lastTag));
        ySpeed = yController.calculate(camSystem.getTargetRange(1, camSystem.lastTag).doubleValue());
        multFactor = 1.4;
      }
      // xSpeed = xController.calculate(camSystem.getYawForTag(1, camSystem.lastTag));
      // ySpeed = yController.calculate(camSystem.getTargetRange(1, camSystem.lastTag).doubleValue());
      driveBase.drive(xSpeed,
        multFactor * ySpeed, 
        thetaController.calculate(driveBase.getWorkingHeading()),
        //0, 
        false);
      // driveBase.drive(
      //   xController.calculate(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue()),
      //   camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue() < 0.2
      //     ? yController.calculate(camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag))
      //     : 0,
      //   thetaController.calculate(driveBase.getWorkingHeading()),
      //   false);
      // driveBase.drive(xController.calculate(camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag)),
      //   (xController.calculate(camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag)) < .2)
      //   ? yController.calculate(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue())
      //   : 0,
      //   //0, 
      //   thetaController.calculate(driveBase.getWorkingHeading()), 
      //   false);
        // driveBase.drive(xController.calculate(camSystem.getYawForTag(1, camSystem.lastTag)),
        // 0,
        // 0, false);
        //Timer.delay(.012);
    }
    else if((camSystem.hasDesiredTarget(camSystem.focusCamIndex, camSystem.lastTag) 
    && camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag) != null 
    && camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag) != null)
    || (camSystem.focusCamIndex == 0 && camSystem.hasDesiredTarget(1, camSystem.lastTag) 
    && camSystem.getTargetRange(1, camSystem.lastTag) != null 
    && camSystem.getYawForTag(1, camSystem.lastTag) != null))
    {
      updateThetaControllerSetpoint(camSystem.lastTag);
      // if(yController.calculate(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue()) < .55
      // && camSystem.focusCamIndex == 0){
      //   xController.setSetpoint(-4.5);
      // }

      double xSpeed = 0;
      double ySpeed = 0;
      double multFactor = 1;
      if(camSystem.focusCamIndex == 0 && camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag) != null
      && yController.calculate(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue()) < .20)
      { 
        // was -18.6
        xController.setSetpoint(-20.8);
        xSpeed = xController.calculate(camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag));
        ySpeed = yController.calculate(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue());
        multFactor = .8;
      }
      else{
        xSpeed = xController.calculate(camSystem.getYawForTag(1, camSystem.lastTag));
        ySpeed = yController.calculate(camSystem.getTargetRange(1, camSystem.lastTag).doubleValue());
        multFactor = 1.4;
      }
      // xSpeed = xController.calculate(camSystem.getYawForTag(1, camSystem.lastTag));
      // ySpeed = yController.calculate(camSystem.getTargetRange(1, camSystem.lastTag).doubleValue());
      driveBase.drive(xSpeed,
        multFactor * ySpeed, 
        thetaController.calculate(driveBase.getWorkingHeading()),
        //0, 
        false);
      // driveBase.drive(
      //   xController.calculate(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue()),
      //   camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue() < 0.2
      //     ? yController.calculate(camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag))
      //     : 0,
      //   thetaController.calculate(driveBase.getWorkingHeading()),
      //   false);
      // driveBase.drive( xController.calculate(camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag)),
      //   (xController.calculate(camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag)) < .2)
      //   ? yController.calculate(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue())
      //   : 0,
      //   //0, 
      //   thetaController.calculate(driveBase.getWorkingHeading()), 
      //   false);
      // driveBase.drive(Math.abs(yController.calculate(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue())) <
      //   (camSystem.focusCamIndex == 0 ? .9 : .2) 
      //   ? xController.calculate(camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag))
      //   : 0,
      //   yController.calculate(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue()), 
      //   thetaController.calculate(driveBase.getWorkingHeading()),
      //   //0, 
      //   false);
      // driveBase.drive(xController.calculate(camSystem.getYawForTag(1, camSystem.lastTag)),
      // 0,
      // 0, false);
        //Timer.delay(.012); 
    }
    else{
      driveBase.drive(0, 0, 0, true);
    }
    
    if(xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint()){
      ended = true;
    }
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
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.drive(0, 0, 0, interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ended;
  }
}