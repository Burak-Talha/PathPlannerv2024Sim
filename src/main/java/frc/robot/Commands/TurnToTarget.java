// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToTarget extends Command {

  private DriveSubsystem driveSubsystem;
  private Alliance currentAlly;
  private Pose2d targetPose;
  private Pose2d currentPose;

  /** Creates a new TurnToTarget. */
  public TurnToTarget(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentAlly = Alliance.Blue;
    targetPose = FieldConstants.BLUE_SUB_WOOFER;
    /*if(currentAlly.get() == Alliance.Blue){
      targetPose = FieldConstants.BLUE_SUB_WOOFER;
    }else{
      targetPose = FieldConstants.RED_SUB_WOOFER;
    }*/
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = driveSubsystem.getPose();
   // System.out.println("Current X:"+currentPose.getX()+"Y:"+currentPose.getY());
    double x = targetPose.getX() - currentPose.getX();
    double y = targetPose.getY() - currentPose.getY();
    double targetSetpoint = Math.toDegrees(Math.atan2(y, x));
    System.out.print("Target Setpoint:"+targetSetpoint);
    driveSubsystem.turnXdegrees(targetSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
