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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
      CameraServer.getInstance().startAutomaticCapture("HopperCamera", 0);
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
      return AutoSwitcher.getAutoInstance().getInstance(
              DRIVE_TRAIN,
              INTAKE,
              HOPPER,
              VISION,
              SHOOTER
      );
  }
}