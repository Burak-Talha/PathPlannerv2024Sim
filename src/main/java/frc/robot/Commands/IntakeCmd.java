// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class IntakeCmd extends Command {

  DriveSubsystem driveSubsystem;
  XboxController controller;

  /** Creates a new IntakeCmd. */
  public IntakeCmd(DriveSubsystem driveSubsystem, XboxController controller) {
    this.driveSubsystem = driveSubsystem;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.arcadeDrive((controller.getRawAxis(3)-controller.getRawAxis(2))*0.4, -controller.getLeftX());
    // TO-DO
    // 1-) Adjust the intake angle
    // 2-) Run intake
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
