// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DomainExpansion;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.CameraSystem;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAllignR extends Command {
  /** Creates a new AutoAllignR. */
  private DriveSubsystem drivebase;
  private CameraSystem camSystem;
  private double rot;
  private double xSpeed;
  private double ySpeed;
  private boolean ended = false;
  private SwerveDrivePoseEstimator swerveEst ;
  private double timer;
  Pose2d desPose = null;
  
  public AutoAllignR() {
    // Use addRequirements() here to declare subsystem dependencies.
    camSystem = CameraSystem.getInstance();
    drivebase = DriveSubsystem.getInstance();
    timer = Timer.getFPGATimestamp();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveEst = camSystem.swerveEst;
    
    //   if(camSystem.lastTag != 0){
    //     desPose = camSystem.dictionary.get(camSystem.lastTag).get(1);
    // }
    // else{
    //     desPose = curPose;
    // }
    // if(desPose.getX() - curPose.getX() <= .07 && desPose.getY() - curPose.getY() <= .07 && Timer.getFPGATimestamp() > timer + 1){
    //   ended = true;
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Pose2d curPose = swerveEst.getEstimatedPosition();
    camSystem.updateLatestResult(false);
    Double yaw = camSystem.getYawForTag(0, camSystem.lastTag);
      if(yaw != null){
        rot = -yaw * .002 * Constants.DriveConstants.kMaxAngularSpeed;
      }
      ArrayList<Double> speeds = camSystem.getPoseToTravel(1);
      if(!camSystem.side.getSelected()){
        xSpeed = -MathUtil.clamp(speeds.get(0), -.4, .4);
        ySpeed = -MathUtil.clamp(speeds.get(1), -.4, .4);
        if(!camSystem.hasTargets()){
        xSpeed = -MathUtil.clamp(speeds.get(0), -.1, .1);
        ySpeed = -MathUtil.clamp(speeds.get(1), -.1, .1);
      }
      }else{
        xSpeed = MathUtil.clamp(speeds.get(0), -.4, .4);
        ySpeed = MathUtil.clamp(speeds.get(1), -.4, .4);
        if(!camSystem.hasTargets()){
        xSpeed = MathUtil.clamp(speeds.get(0), -.1, .1);
        ySpeed = MathUtil.clamp(speeds.get(1), -.1, .1);
      }
    }
      if(Math.abs(speeds.get(0)) < .5 && Math.abs(speeds.get(1)) < .5){
        rot = camSystem.getPerpendicularYaw() * .0014 * Constants.DriveConstants.kMaxAngularSpeed;
      }
    drivebase.drive(xSpeed, ySpeed, rot, true);
    if(Math.abs(xSpeed) < .05 && Math.abs(ySpeed) < .05 && Math.abs(rot) < .05){
      ended = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ended;
  }
}
