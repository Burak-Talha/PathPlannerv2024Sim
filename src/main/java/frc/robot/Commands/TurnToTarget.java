// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.lang.reflect.Field;
import java.sql.Driver;
import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ControlBoard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToTarget extends Command {

  public DriveSubsystem driveSubsystem;
  private Optional<Alliance> currentAlly;
  private Pose2d targetPose;
  private Pose2d currentPose;
  private double currentAngle;

  private final PIDController turnPidController = new PIDController(AutoConstants.kPTurnController, AutoConstants.KITurnController, AutoConstants.kDTurnController);
  private XboxController xboxController;

  /** Creates a new TurnToTarget. */
  public TurnToTarget(DriveSubsystem driveSubsystem, XboxController xboxController) {
    this.driveSubsystem = driveSubsystem;
    this.xboxController = xboxController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentAlly = DriverStation.getAlliance();

    try{
    if(currentAlly.get() == Alliance.Red){
    targetPose = FieldConstants.RED_SUB_WOOFER;
    }else if(currentAlly.get() == Alliance.Blue){
      targetPose = FieldConstants.BLUE_SUB_WOOFER;
      //driveSubsystem.;
    }
  }catch(Exception exception){exception.printStackTrace();}

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = driveSubsystem.getPose();
   // System.out.println("Current X:"+currentPose.getX()+"Y:"+currentPose.getY());
    double x = targetPose.getX() - currentPose.getX();
    double y = targetPose.getY() - currentPose.getY();
    double targetSetpoint = Math.toDegrees(Math.atan2(y, x));

    try{
      if(DriverStation.getAlliance().get() == Alliance.Blue){
      // Ensure the angle is in the range [0, 360)
      targetSetpoint = (targetSetpoint + 360) % 360;
      }
          System.out.print("Target Setpoint: " + targetSetpoint);
        if(DriverStation.getAlliance().get() == Alliance.Blue){
          currentAngle = driveSubsystem.getAngleCalculationForBlue();
        }
        else{
          currentAngle = driveSubsystem.getPose().getRotation().getDegrees();
        }
      }
    catch(Exception exception){
      exception.printStackTrace();
    }
      //getPose().getRotation().getDegrees()

    // Pass the target setpoint to the turn method in degrees
    driveSubsystem.arcadeDrive((ControlBoard.getXvelocityDrive())*0.4, turnPidController.calculate(currentAngle, targetSetpoint));

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
