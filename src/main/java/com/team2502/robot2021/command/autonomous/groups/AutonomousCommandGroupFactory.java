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

    public static SequentialCommandGroup BARREL_RACING(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem vision, ShooterSubsystem shooter) {
        ParallelRaceGroup ForwardStartZone = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1.0, 0),
                new WaitCommand(1.0)
        );
        ParallelRaceGroup TurnLeft = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 1.0, 0.6),
                new WaitCommand(3.0)
        );
        ParallelRaceGroup Straight = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1.0, 0),
                new WaitCommand(1.0)
        );
        ParallelRaceGroup TurnRight = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 0.6, 1.0),
                new WaitCommand(0.5)
        );
        ParallelRaceGroup Straight2 = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 1.0, 1.0),
                new WaitCommand(0.25)
        );
        ParallelRaceGroup MoveLeft = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 0.6, 1.0),
                new WaitCommand(3.0)
        );
        ParallelRaceGroup BackToStartZone = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1.0, 0),
                new WaitCommand(3.0)
        );

        return new SequentialCommandGroup(
                ForwardStartZone,
                TurnLeft,
                Straight,
                TurnRight,
                Straight2,
                MoveLeft
                //BackToStartZone
        );
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

    public static SequentialCommandGroup PATH_A_PICKUP_ALL(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem v, ShooterSubsystem shooter) {
        ParallelRaceGroup intakeWhileMoving = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.6, 0),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.5)
        );

        ParallelRaceGroup stopAndIntake = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0, 0),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(1.5)
        );

        ParallelRaceGroup c3BallPickup = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.6, 0),
                new WaitCommand(0.3)
        );

        ParallelRaceGroup d5BallPickupP1 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.6, 0),
                new WaitCommand(0.1)
        );

        ParallelRaceGroup turnToE6 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain, 45),
                new WaitCommand(0.5)
        );


        ParallelRaceGroup e6BallPickup = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.6, 0),
                new WaitCommand(0.2)
        );

        ParallelRaceGroup turnToA6 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain, 240),
                new WaitCommand(1.5)
        );

        ParallelRaceGroup a6BallPickup = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.6, 0),
                new WaitCommand(1)
        );

        ParallelRaceGroup turnToB7 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain, 30),
                new WaitCommand(1)
        );

        ParallelRaceGroup b7BallPickup = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.6, 0),
                new WaitCommand(0.3)
        );

        ParallelRaceGroup c9BallPickupP1 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.6, 0),
                new WaitCommand(0.4)
        );

        ParallelRaceGroup turnToC9 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain, 330),
                new WaitCommand(1)
        );

        ParallelRaceGroup c9BallPickupP2 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.6, 0),
                new WaitCommand(1)
        );

        ParallelRaceGroup driveToEnd = new ParallelRaceGroup(
          new DriveStraightCommand(drivetrain, 0.6, 0),
          new WaitCommand(1)
        );


        return new SequentialCommandGroup(
                c3BallPickup,
                intakeWhileMoving,
                d5BallPickupP1,
                intakeWhileMoving,
                turnToE6,
                e6BallPickup,
                stopAndIntake,
                turnToA6,
                a6BallPickup,
                stopAndIntake,
                turnToB7,
                b7BallPickup,
                intakeWhileMoving,
                c9BallPickupP1,
                turnToC9,
                intakeWhileMoving,
                driveToEnd
                );



    }
    public static SequentialCommandGroup Galactic_B_Backup(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem v, ShooterSubsystem shooter){

        ParallelRaceGroup straightToB3Pickup = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain,0.6,0),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.6)
        );

        ParallelRaceGroup turnToD5 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain,30),
                new WaitCommand(1)
        );

        ParallelRaceGroup straightToD5Pickup = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain,0.6,30),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.6)
        );

        ParallelRaceGroup turnToD6 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain,0),
                new WaitCommand(1)
        );

        ParallelRaceGroup straightToD6Pickup = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain,0.7,0),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.4)
        );

        ParallelRaceGroup turnToB7 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain,290),
                new WaitCommand(1)
        );

        ParallelRaceGroup strightToB7Pickup = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.6,290),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.6)
        );

        ParallelRaceGroup turnToB8 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain,0),
                new WaitCommand(1)
        );

        ParallelRaceGroup straightToB8Pickup = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain,0.7,0),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.4)
        );

        ParallelRaceGroup turnToD10 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain,70),
                new WaitCommand(1)
        );

        ParallelRaceGroup straightToD10 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain,0.6,70),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.6)
        );

        ParallelRaceGroup goToEnd = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain,0.6,55),
                new WaitCommand(0.5)
        );

        return new SequentialCommandGroup(
                straightToB3Pickup,
                turnToD5,
                straightToD5Pickup,
                turnToD6,
                straightToD6Pickup,
                turnToB7,
                strightToB7Pickup,
                turnToB8,
                straightToB8Pickup,
                turnToD10,
                straightToD10,
                goToEnd

        );
    }

    public static SequentialCommandGroup GalacticSearchRedA(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem v, ShooterSubsystem shooter) {
        ParallelRaceGroup DriveStraightC3 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1, 0),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.5)
        );
        ParallelRaceGroup MoveToD5 = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 1, 0.8),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.7)
        );
        ParallelRaceGroup DriveStraightA6 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1, 45),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(1)
        );
        ParallelRaceGroup MoveHome = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1, 0),
                new WaitCommand(1.5)
        );

        return new SequentialCommandGroup(
                DriveStraightC3,
                MoveToD5,
                DriveStraightA6,
                MoveHome
        );
    }
}