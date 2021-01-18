package com.team2502.robot2021.command.autonomous.groups;

import com.team2502.robot2021.Constants;
import com.team2502.robot2021.command.*;
import com.team2502.robot2021.command.autonomous.ingredients.*;
import com.team2502.robot2021.subsystem.*;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonomousCommandGroupFactory {
    private static SequentialCommandGroup SpoolUpShooterAndShootRaceGroup(ShooterSubsystem shooter, HopperSubsystem hopper, double speed) {
        ParallelRaceGroup spoolUpShooter = new ParallelRaceGroup(
                new ShootAtRPMCommand(shooter, speed),
                new WaitCommand(2)
        );

        ParallelRaceGroup runHopperAndShootBalls = new ParallelRaceGroup(
                new ShootAtRPMCommand(shooter, speed),
                new RunHopperCommand(hopper, shooter, Constants.Robot.MotorSpeeds.HOPPER_LEFT_BELT,
                        Constants.Robot.MotorSpeeds.HOPPER_RIGHT_BELT, Constants.Robot.MotorSpeeds.HOPPER_EXIT_WHEEL,
                        Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT, false),
                new WaitCommand(3)
        );

        return new SequentialCommandGroup(
                spoolUpShooter,
                runHopperAndShootBalls
        );
    }

    private static SequentialCommandGroup VoltageDriveRaceGroup(DrivetrainSubsystem drivetrain, double leftSpeed, double rightSpeed, double timeout) {
        ParallelRaceGroup drive = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, leftSpeed, rightSpeed),
                new WaitCommand(timeout)
        );

        return new SequentialCommandGroup(drive);
    }

    public static SequentialCommandGroup LEFT_START_RENDEZVOUS_4_BALL(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem v, ShooterSubsystem shooter) {

        ParallelRaceGroup driveBackFromInitLine = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, -0.8, -0.8),
                new WaitCommand(0.1)
        );

        ParallelRaceGroup stopRobot = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 0, 0),
                new WaitCommand(0.5)
        );

        ParallelRaceGroup turnRobot = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain, 165),
                new WaitCommand(2)
        );

        ParallelRaceGroup intakeBall = new ParallelRaceGroup(
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new VoltageDriveCommand(drivetrain, 0.7, 0.7),
                new WaitCommand(1.5)
        );

        ParallelRaceGroup stopRobotAgain = new ParallelRaceGroup(
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new VoltageDriveCommand(drivetrain, 0, 0),
                new ShootAtRPMCommand(shooter, Constants.LookupTables.DIST_TO_RPM_TABLE.get(25D)),
                new WaitCommand(1.5)
        );

        ParallelRaceGroup runHopperAndShootBallsAgain = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 0, 0),
                new ShootAtRPMCommand(shooter, Constants.LookupTables.DIST_TO_RPM_TABLE.get(25D)),
                new RunHopperCommand(hopper, shooter, Constants.Robot.MotorSpeeds.HOPPER_LEFT_BELT,
                        Constants.Robot.MotorSpeeds.HOPPER_RIGHT_BELT, Constants.Robot.MotorSpeeds.HOPPER_EXIT_WHEEL,
                        Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT, false),
                new WaitCommand(2.5)
        );

        return new SequentialCommandGroup(
                SpoolUpShooterAndShootRaceGroup(shooter, hopper, Constants.LookupTables.DIST_TO_RPM_TABLE.get(10D)),
                driveBackFromInitLine,
                stopRobot,
                turnRobot,
                intakeBall,
                stopRobotAgain,
                runHopperAndShootBallsAgain
        );
    }

    public static SequentialCommandGroup TRENCH_6_BALL_AUTO(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem vision, ShooterSubsystem shooter) {

        ParallelRaceGroup spoolUpShooter = new ParallelRaceGroup(
                new ShootAtRPMCommand(shooter, Constants.LookupTables.DIST_TO_RPM_TABLE.get(10D)),
                new TurnToAngleCommand(drivetrain, 22),
                new WaitCommand(2)
        );

        ParallelRaceGroup runHopperAndShootBalls = new ParallelRaceGroup(
                new ShootAtRPMCommand(shooter, Constants.LookupTables.DIST_TO_RPM_TABLE.get(10D)),
                new RunHopperCommand(hopper, shooter, Constants.Robot.MotorSpeeds.HOPPER_LEFT_BELT,
                        Constants.Robot.MotorSpeeds.HOPPER_RIGHT_BELT, Constants.Robot.MotorSpeeds.HOPPER_EXIT_WHEEL,
                        Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT, false),
                new WaitCommand(2)
        );

        ParallelRaceGroup TurnToBalls = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain, 176),
                new WaitCommand(1)
        );

        ParallelRaceGroup driveThroughTrench = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.6, 180),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(3.5)
        );

        ParallelRaceGroup stopAndIntake = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 0, 0),
                new ShootAtRPMCommand(shooter, Constants.LookupTables.DIST_TO_RPM_TABLE.get(30D)),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(1.5)
        );

        ParallelRaceGroup TurnToTarget = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain, 22),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(1.5)
        );

        ParallelRaceGroup VisionAlignment = new ParallelRaceGroup(
                new VoltageDriveWhileVisionAligningCommand(vision, drivetrain, 0.4),
                new WaitCommand(1)
        );
        ParallelCommandGroup runHopperAndShootBallsAligning = new ParallelCommandGroup(
                new RunHopperCommand(hopper, shooter, Constants.Robot.MotorSpeeds.HOPPER_LEFT_BELT,
                        Constants.Robot.MotorSpeeds.HOPPER_RIGHT_BELT, Constants.Robot.MotorSpeeds.HOPPER_EXIT_WHEEL,
                        Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT, false),
                new WaitCommand(2)
        );


        return new SequentialCommandGroup(
                spoolUpShooter,
                runHopperAndShootBalls,
                TurnToBalls,
                driveThroughTrench,
                stopAndIntake,
                new RunShooterCommand(shooter, vision, Constants.LookupTables.DIST_TO_RPM_TABLE.get(25D)),
                TurnToTarget,
                VisionAlignment,
                runHopperAndShootBallsAligning
        );
    }

    public static SequentialCommandGroup SIMPLE_SHOOT_3_BACKWARDS(DrivetrainSubsystem drivetrain, IntakeSubsystem i, HopperSubsystem hopper, VisionSubsystem v, ShooterSubsystem shooter) {

        return new SequentialCommandGroup(
                SpoolUpShooterAndShootRaceGroup(shooter, hopper, Constants.LookupTables.DIST_TO_RPM_TABLE.get(10D)),
                VoltageDriveRaceGroup(drivetrain, -0.8, -0.8, 0.55)
        );
    }

    public static SequentialCommandGroup SIMPLE_SHOOT_3_FORWARDS(DrivetrainSubsystem drivetrain, IntakeSubsystem i, HopperSubsystem hopper, VisionSubsystem v, ShooterSubsystem shooter) {

        return new SequentialCommandGroup(
                SpoolUpShooterAndShootRaceGroup(shooter, hopper, Constants.LookupTables.DIST_TO_RPM_TABLE.get(10D)),
                VoltageDriveRaceGroup(drivetrain, 0.8, 0.8, 0.35)
        );
    }
}
