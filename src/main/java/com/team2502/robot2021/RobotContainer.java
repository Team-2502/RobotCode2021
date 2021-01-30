/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team2502.robot2021;

import com.team2502.robot2021.command.*;
import com.team2502.robot2021.subsystem.*;
import com.team2502.robot2021.Constants.OI;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the Robot periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  protected final ControlPanelWheelSubsystem CONTROL_PANEL = new ControlPanelWheelSubsystem();
  protected final DrivetrainSubsystem DRIVE_TRAIN = new DrivetrainSubsystem();
  protected final ClimberSubsystem CLIMBER = new ClimberSubsystem();
  protected final IntakeSubsystem INTAKE = new IntakeSubsystem();
  protected final HopperSubsystem HOPPER = new HopperSubsystem();
  protected final VisionSubsystem VISION = new VisionSubsystem();
  protected final ShooterSubsystem SHOOTER = new ShooterSubsystem();

  private static final Joystick JOYSTICK_DRIVE_RIGHT = new Joystick(Constants.OI.JOYSTICK_DRIVE_RIGHT);
  private static final Joystick JOYSTICK_DRIVE_LEFT = new Joystick(Constants.OI.JOYSTICK_DRIVE_LEFT);
  private static final Joystick JOYSTICK_OPERATOR = new Joystick(Constants.OI.JOYSTICK_OPERATOR);


    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
      configureButtonBindings();

      DRIVE_TRAIN.setDefaultCommand(new DriveCommand(DRIVE_TRAIN, JOYSTICK_DRIVE_LEFT, JOYSTICK_DRIVE_RIGHT));

      AutoSwitcher.putToSmartDashboard();
      CameraServer.getInstance().startAutomaticCapture();
  }

  private void configureButtonBindings() {
    JoystickButton RunControlPanelButton = new JoystickButton(JOYSTICK_OPERATOR, Constants.OI.BUTTON_CONTROL_PANEL);
    RunControlPanelButton.whileHeld(new RunControlPanelWheelCommand(CONTROL_PANEL, Constants.Robot.MotorSpeeds.CONTROL_PANEL));

    JoystickButton ActuateControlPanel = new JoystickButton(JOYSTICK_OPERATOR, Constants.OI.BUTTON_ACTUATE_CONTROL_PANEL);
    ActuateControlPanel.whenPressed(new ToggleControlPanelWheelSolenoidCommand(CONTROL_PANEL));

    JoystickButton RunIntakeButton = new JoystickButton(JOYSTICK_OPERATOR,Constants.OI.BUTTON_RUN_INTAKE);
    JoystickButton RunIntakeBackwardsButton = new JoystickButton(JOYSTICK_OPERATOR,Constants.OI.BUTTON_RUN_INTAKE_BACKWARDS);

    RunIntakeButton.whileHeld(new RunIntakeCommand(INTAKE, HOPPER, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE));
    RunIntakeBackwardsButton.whileHeld(new RunIntakeCommand(INTAKE, HOPPER, Constants.Robot.MotorSpeeds.INTAKE_SPEED_BACKWARDS, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_BACKWARDS, 0));

    JoystickButton ShiftButton = new JoystickButton(JOYSTICK_DRIVE_RIGHT, Constants.OI.BUTTON_SHIFT);
    ShiftButton.whenPressed(new ToggleDrivetrainGearCommand(DRIVE_TRAIN));

    JoystickButton VisionButton = new JoystickButton(JOYSTICK_DRIVE_LEFT, OI.BUTTON_VISION_ALIGN);
    VisionButton.whileHeld(new DriveWithVisionAlignCommand(VISION, DRIVE_TRAIN, JOYSTICK_DRIVE_LEFT, JOYSTICK_DRIVE_RIGHT, Constants.Robot.Vision.STANDARD_DISTANCE));

    JoystickButton VisionButtonYellow = new JoystickButton(JOYSTICK_DRIVE_RIGHT, OI.BUTTON_YELLOW_ZONE);
    VisionButtonYellow.whileHeld(new DriveWithVisionAlignCommand(VISION, DRIVE_TRAIN, JOYSTICK_DRIVE_LEFT, JOYSTICK_DRIVE_RIGHT, Constants.Robot.Vision.YELLOW_ZONE_DISTANCE));

    JoystickButton VisionButtonBlue = new JoystickButton(JOYSTICK_DRIVE_RIGHT, OI.BUTTON_BLUE_ZONE);
    VisionButtonBlue.whileHeld(new DriveWithVisionAlignCommand(VISION, DRIVE_TRAIN, JOYSTICK_DRIVE_LEFT, JOYSTICK_DRIVE_RIGHT, Constants.Robot.Vision.BLUE_ZONE_DISTANCE));

    JoystickButton VisionButtonRed = new JoystickButton(JOYSTICK_DRIVE_RIGHT, OI.BUTTON_RED_ZONE);
    VisionButtonRed.whileHeld(new DriveWithVisionAlignCommand(VISION, DRIVE_TRAIN, JOYSTICK_DRIVE_LEFT, JOYSTICK_DRIVE_RIGHT, Constants.Robot.Vision.RED_ZONE_DISTANCE));


    JoystickButton HopperContinuousButton = new JoystickButton(JOYSTICK_OPERATOR, Constants.OI.BUTTON_HOPPER_CONTINUOUS);
    HopperContinuousButton.whileHeld(new RunHopperCommand(HOPPER, SHOOTER, Constants.Robot.MotorSpeeds.HOPPER_LEFT_BELT,
            Constants.Robot.MotorSpeeds.HOPPER_RIGHT_BELT, Constants.Robot.MotorSpeeds.HOPPER_EXIT_WHEEL, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT, true));

    JoystickButton HopperContinuousButtonReverse = new JoystickButton(JOYSTICK_OPERATOR, Constants.OI.BUTTON_HOPPER_CONTINUOUS_REVERSE);
    HopperContinuousButtonReverse.whileHeld(new RunHopperCommand(HOPPER, SHOOTER, Constants.Robot.MotorSpeeds.HOPPER_LEFT_BELT_REVERSE,
            Constants.Robot.MotorSpeeds.HOPPER_RIGHT_BELT_REVERSE, Constants.Robot.MotorSpeeds.HOPPER_EXIT_WHEEL_REVERSE, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_REVERSE, false));

    JoystickButton RunShooterCloseButton = new JoystickButton(JOYSTICK_OPERATOR, Constants.OI.BUTTON_RUN_SHOOTER_INIT_LINE);
    RunShooterCloseButton.toggleWhenPressed(new RunShooterCommand(SHOOTER, VISION, 3733D));

    JoystickButton RunShooterTrenchButton = new JoystickButton(JOYSTICK_OPERATOR, Constants.OI.BUTTON_RUN_SHOOTER_TRENCH);
    RunShooterTrenchButton.toggleWhenPressed(new RunShooterCommand(SHOOTER,VISION , Constants.LookupTables.DIST_TO_RPM_TABLE.get(25D)));

    JoystickButton RunClimberForwardsButton = new JoystickButton(JOYSTICK_OPERATOR, Constants.OI.BUTTON_CLIMBER);
    RunClimberForwardsButton.whileHeld(new RunClimberCommand(CLIMBER, Constants.Robot.MotorSpeeds.CLIMBER_FORWARD));

    JoystickButton RunClimberReverseButton = new JoystickButton(JOYSTICK_OPERATOR, Constants.OI.BUTTON_CLIMBER_REVERSE);
    RunClimberReverseButton.whileHeld(new RunClimberCommand(CLIMBER, Constants.Robot.MotorSpeeds.CLIMBER_BACKWARD));

    JoystickButton ActuateCLimberButton = new JoystickButton(JOYSTICK_OPERATOR, OI.BUTTON_CLIMBER_ACTUATE);
    ActuateCLimberButton.whenPressed(new ToggleClimberLockCommand(CLIMBER));

    JoystickButton RunSqueezeBackwards = new JoystickButton(JOYSTICK_OPERATOR, OI.BUTTON_BOTTOM_ROLLER_BACKWARDS);
    RunSqueezeBackwards.whileHeld(new RunIntakeCommand(INTAKE, HOPPER, 0, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_BACKWARDS, 0));
  }

  public Command getAutonomousRoutine() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(Constants.Robot.Auto.ksVolts,
                            Constants.Robot.Auto.kvVoltSecondsPerMeter,
                            Constants.Robot.Auto.kaVoltSecondsSquaredPerMeter),
                    Constants.Robot.Auto.kDriveKinematics,
                    10);

    // Create config for trajectory
    TrajectoryConfig config =
            new TrajectoryConfig(Constants.Robot.Auto.kMaxSpeedMetersPerSecond,
                    Constants.Robot.Auto.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.Robot.Auto.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                    new Translation2d(1, 1),
                    new Translation2d(2, -1)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
            exampleTrajectory,
            DRIVE_TRAIN::getPose,
            new RamseteController(Constants.Robot.Auto.kRamseteB, Constants.Robot.Auto.kRamseteZeta),
            new SimpleMotorFeedforward(Constants.Robot.Auto.ksVolts,
                    Constants.Robot.Auto.kvVoltSecondsPerMeter,
                    Constants.Robot.Auto.kaVoltSecondsSquaredPerMeter),
            Constants.Robot.Auto.kDriveKinematics,
            DRIVE_TRAIN::getWheelSpeeds,
            new PIDController(Constants.Robot.Auto.kPDriveVel, 0, 0),
            new PIDController(Constants.Robot.Auto.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            DRIVE_TRAIN::tankDriveVolts,
            DRIVE_TRAIN
    );

    // Reset odometry to the starting pose of the trajectory.
    DRIVE_TRAIN.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> DRIVE_TRAIN.tankDriveVolts(0, 0));
//      return AutoSwitcher.getAutoInstance().getInstance(
//              DRIVE_TRAIN,
//              INTAKE,
//              HOPPER,
//              VISION,
//              SHOOTER
//      );
  }
}