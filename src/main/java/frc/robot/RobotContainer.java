// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Commands.TurnToTarget;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot periodic
 * methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  double targetX;
  double targetY;

  // The driver's controller
  XboxController m_driverController =
      new XboxController(Constants.OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser("New Auto");
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        // If you are using the keyboard as a joystick, it is recommended that you go
        // to the following link to read about editing the keyboard settings.
        // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/simulation-gui.html#using-the-keyboard-as-a-joystick
        new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    -m_driverController.getLeftY(), -m_driverController.getRightX()),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
        .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));

    new JoystickButton(m_driverController, Button.kA.value).whileTrue(new TurnToTarget(m_robotDrive).repeatedly());
  }

  public DriveSubsystem getRobotDrive() {
    return m_robotDrive;
  }

  /** Zeros the outputs of all subsystems. */
  public void zeroAllOutputs() {
    m_robotDrive.tankDriveVolts(0, 0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Reset odometry to starting pose of trajectory.
    //m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    //m_robotDrive.followPathCommand("Example Path");
    PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
    //autoChooser.getSelected().andThen(AutoBuilder.pathfindThenFollowPath(path, new PathConstraints(3, 3, 0, 0)));
    //autoChooser.getSelected(); autoChooser.getSelected().andThen(AutoBuilder.pathfindThenFollowPath(path, new PathConstraints(3, 3, 0, 0)));
    // AutoBuilder.pathfindToPose(new Pose2d(7, 6, new Rotation2d(Math.toRadians(0))), new PathConstraints(3, 3, 0, 0)).andThen(AutoBuilder.pathfindToPose(new Pose2d(1, 4, new Rotation2d(Math.toRadians(0))), new PathConstraints(3, 3, 0, 0))).andThen(AutoBuilder.pathfindToPose(new Pose2d(9, 1, new Rotation2d(Math.toRadians(0))), new PathConstraints(3, 3, 0, 0)));
    //autoChooser.getSelected().andThen(AutoBuilder.pathfindThenFollowPath(path, new PathConstraints(3, 3, 0, 0)));
    return autoChooser.getSelected();
  }
}