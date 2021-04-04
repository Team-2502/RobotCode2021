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

    public static SequentialCommandGroup SLALOM_PATH(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem v, ShooterSubsystem shooter) {
        ParallelRaceGroup TurnToFirstStraightaway = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 0.7, 1.0),
                new WaitCommand(0.90)
        );

        ParallelRaceGroup TurnToFirstStraightaway2 = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 1.0, 0.60),
                new WaitCommand(0.40)
        );

        ParallelRaceGroup StraightToEndTurn = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1.0, 0),
                new WaitCommand(1.2)
        );

        ParallelRaceGroup EndTurn1 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.85, -70),
                new WaitCommand(0.7)
        );
        ParallelRaceGroup EndTurn2 = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 0.65, 1.0),
                new WaitCommand(2.65)
        );
        ParallelRaceGroup EndTurn2b = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.85, -90),
                new WaitCommand(0.30)
        );
        ParallelRaceGroup EndTurn3b = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 1.0, 0.1),
                new WaitCommand(0.45)
        );

        ParallelRaceGroup StraightBack = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1.0, -179),
                new WaitCommand(1.4)
        );

        ParallelRaceGroup FinalTurn1 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1.0, 110),
                new WaitCommand(1.5)
        );

        ParallelRaceGroup FinalTurn2 = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 0.8, 1.0),
                new WaitCommand(1.0)
        );

        return new SequentialCommandGroup(
                TurnToFirstStraightaway,
                TurnToFirstStraightaway2,
                StraightToEndTurn,
                EndTurn1,
                EndTurn2,
                EndTurn2b,
                EndTurn3b,
                StraightBack,
                FinalTurn1
        );
    }

    public static SequentialCommandGroup BARREL_RACING_LOW(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem vision, ShooterSubsystem shooter) {
        // 12.43 Chris
        ParallelRaceGroup ForwardFromStartZone = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1.0, 0),
                new WaitCommand(1.25)
        );
        ParallelRaceGroup TurnAroundPoint1 = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 1.0, 0.5),
                new WaitCommand(1.55)
        );
        ParallelRaceGroup StraightToSecondPoint = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1.0, 0),
                new WaitCommand(0.90)
        );
        ParallelRaceGroup TurnLeftAroundPoint2 = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 0.6, 1.0),
                new WaitCommand(2.75)
        );
        ParallelRaceGroup StraightToThirdPoint = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1.0, -45), // -50
                new WaitCommand(0.75)
        );
        ParallelRaceGroup TurnAroundThirdPoint = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 0.6, 1.0),
                new WaitCommand(2)
        );
        ParallelRaceGroup StraightBackToStartZone = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1.0, 180),
                new WaitCommand(1.5)
        );

        return new SequentialCommandGroup(
                ForwardFromStartZone,
                TurnAroundPoint1,
                StraightToSecondPoint,
                TurnLeftAroundPoint2,
                StraightToThirdPoint,
                TurnAroundThirdPoint,
                new ToggleDrivetrainGearCommand(drivetrain),
                StraightBackToStartZone
        );
    }
    
    public static SequentialCommandGroup BOUNCE_PATH(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem v, ShooterSubsystem shooter) {

            ParallelRaceGroup startTurn = new ParallelRaceGroup(
                    new VoltageDriveCommand(drivetrain, 0.65, 1),
                    new WaitCommand(1.0)
            );

            ParallelRaceGroup driveStraightBack = new ParallelRaceGroup(
                    new DriveStraightCommand(drivetrain, -0.8, 103),
                    new WaitCommand(1.5)
            );

            ParallelRaceGroup turnAroundBack1 = new ParallelRaceGroup(
                    new VoltageDriveCommand(drivetrain, -1, -0.6),
                    new WaitCommand(1.0)
            );

            ParallelRaceGroup backToPoint2 = new ParallelRaceGroup(
                    new DriveStraightCommand(drivetrain, -1, -90),
                    new WaitCommand(1.05)
            );

            ParallelRaceGroup straightAfter2 = new ParallelRaceGroup(
                    new DriveStraightCommand(drivetrain, 1, -90),
                    new WaitCommand(1.25)
            );

            ParallelRaceGroup turnTo3 = new ParallelRaceGroup(
                    new VoltageDriveCommand(drivetrain, 0.7, 1),
                    new WaitCommand(1.3)
            );

            ParallelRaceGroup straightTo3 = new ParallelRaceGroup(
                    new DriveStraightCommand(drivetrain, 1, 90),
                    new WaitCommand(1.1)
            );

            ParallelRaceGroup turnToFinish = new ParallelRaceGroup(
                    new VoltageDriveCommand(drivetrain, -0.9, -0.6),
                    new WaitCommand(1.5)
            );

            return new SequentialCommandGroup(
                    startTurn,
                    driveStraightBack,
                    turnAroundBack1,
                    backToPoint2,
                    straightAfter2,
                    turnTo3,
                    straightTo3,
                    turnToFinish
            );
    }

    public static SequentialCommandGroup BARREL_RACING_HIGH(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem vision, ShooterSubsystem shooter) {
        // 12.43 Chris
        ParallelRaceGroup ForwardFromStartZone = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1.0, 8),
                new WaitCommand(1.0)
        );
        ParallelRaceGroup TurnAroundPoint1 = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 1.0, 0.5),
                new WaitCommand(2.0)
        );
        ParallelRaceGroup StraightToSecondPoint = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1.0, 0),
                new WaitCommand(0.60)
        );
        ParallelRaceGroup TurnLeftAroundPoint2 = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 0.6, 1.0),
                new WaitCommand(2.75)
        );
        ParallelRaceGroup StraightToThirdPoint = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1.0, -45), // -50
                new WaitCommand(0.3)
        );
        ParallelRaceGroup TurnAroundThirdPoint = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 0.6, 1.0),
                new WaitCommand(2)
        );
        ParallelRaceGroup StraightBackToStartZone = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1.0, 180),
                new WaitCommand(1.5)
        );

        return new SequentialCommandGroup(
                new ToggleDrivetrainGearCommand(drivetrain),
                ForwardFromStartZone,
                TurnAroundPoint1,
                StraightToSecondPoint
//                new ToggleDrivetrainGearCommand(drivetrain),
//                TurnLeftAroundPoint2,
//                new ToggleDrivetrainGearCommand(drivetrain),
//                StraightToThirdPoint,
//                new ToggleDrivetrainGearCommand(drivetrain),
//                TurnAroundThirdPoint,
//                new ToggleDrivetrainGearCommand(drivetrain),
//                StraightBackToStartZone
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
                new WaitCommand(3)
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
                new RunShooterCommand(shooter, vision, Constants.LookupTables.DIST_TO_RPM_TABLE.get(30D)),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(1.5)
        );

        ParallelRaceGroup TurnToTarget = new ParallelRaceGroup(
                new RunShooterCommand(shooter, vision, Constants.LookupTables.DIST_TO_RPM_TABLE.get(30D)),
                new TurnToAngleCommand(drivetrain, 11.5),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(2.0)
        );

        ParallelCommandGroup runHopperAndShootBallsAligning = new ParallelCommandGroup(
                new RunShooterCommand(shooter, vision, Constants.LookupTables.DIST_TO_RPM_TABLE.get(30D)),
                new VoltageDriveWhileVisionAligningCommand(vision, drivetrain, 0.0),
                new RunHopperCommand(hopper, shooter, Constants.Robot.MotorSpeeds.HOPPER_LEFT_BELT,
                        Constants.Robot.MotorSpeeds.HOPPER_RIGHT_BELT, Constants.Robot.MotorSpeeds.HOPPER_EXIT_WHEEL,
                        Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT, false),
                new WaitCommand(2.0)
        );


        return new SequentialCommandGroup(
                spoolUpShooter,
                runHopperAndShootBalls,
                TurnToBalls,
                driveThroughTrench,
                stopAndIntake,
                TurnToTarget,
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

    public static SequentialCommandGroup Galactic_B_Backup(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem v, ShooterSubsystem shooter) {

        ParallelRaceGroup straightToB3Pickup = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.6, 0),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.6)
        );

        ParallelRaceGroup turnToD5 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain, 30),
                new WaitCommand(1)
        );

        ParallelRaceGroup straightToD5Pickup = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.6, 30),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.6)
        );

        ParallelRaceGroup turnToD6 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain, 0),
                new WaitCommand(1)
        );

        ParallelRaceGroup straightToD6Pickup = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.7, 0),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.4)
        );

        ParallelRaceGroup turnToB7 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain, 290),
                new WaitCommand(1)
        );

        ParallelRaceGroup strightToB7Pickup = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.6, 290),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.6)
        );

        ParallelRaceGroup turnToB8 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain, 0),
                new WaitCommand(1)
        );

        ParallelRaceGroup straightToB8Pickup = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.7, 0),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.4)
        );

        ParallelRaceGroup turnToD10 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain, 70),
                new WaitCommand(1)
        );

        ParallelRaceGroup straightToD10 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.6, 70),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.6)
        );

        ParallelRaceGroup goToEnd = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.6, 55),
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

    public static SequentialCommandGroup GalacticSearchRedB(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem v, ShooterSubsystem shooter) {
        ParallelRaceGroup toB3 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.8, 27),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(1)
        );

        ParallelRaceGroup toD5 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1, -40),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(1.0)
        );

        ParallelRaceGroup TurnToB7 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain, 43),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.8)
        );

        ParallelRaceGroup toB7 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1, 43),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.8)
        );

        ParallelRaceGroup turn2 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1, 0),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(2.0)
        );
        return new SequentialCommandGroup(
                toB3,
                toD5,
                TurnToB7,
                toB7,
                turn2
        );
    }

    // 12.45 sibley
    public static SequentialCommandGroup GalacticSearchRedA(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem v, ShooterSubsystem shooter) {
        ParallelRaceGroup DriveStraightC3 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.8, 0),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.6)
        );
        ParallelRaceGroup MoveToD5 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1, -20),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.7)
        );

        ParallelRaceGroup TurnToA6 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain, 67),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.8)
        );

        ParallelRaceGroup DriveStraightA6 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 0.9, 67),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(1.0) //1.8
        );
        ParallelRaceGroup MoveHome = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1.0, 0),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(2.0)
        );

        return new SequentialCommandGroup(
                DriveStraightC3,
                MoveToD5,
                TurnToA6,
                DriveStraightA6,
                MoveHome
        );
    }

    public static SequentialCommandGroup GalacticSearchBlueB(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem v, ShooterSubsystem shooter) {
        ParallelRaceGroup DriveToD6 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1, -10),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(1.5)
        );
        ParallelRaceGroup MoveToB8 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1, 30),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.9)
        );
        ParallelRaceGroup MoveToD10 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1, -30),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(0.9)
        );

        return new SequentialCommandGroup(
                DriveToD6,
                MoveToB8,
                MoveToD10

        );
    }

    public static SequentialCommandGroup GalacticSearchBlueA(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem v, ShooterSubsystem shooter) {
        ParallelRaceGroup driveStraight1 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1, -15),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(2)
        );

        ParallelRaceGroup driveStraight2 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1, 85),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(1.2)
        );

        ParallelRaceGroup driveStraight3 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1, -20),
                new RunIntakeCommand(intake, hopper, Constants.Robot.MotorSpeeds.INTAKE_SPEED_FORWARD, Constants.Robot.MotorSpeeds.INTAKE_SQUEEZE_SPEED_FORWARDS, Constants.Robot.MotorSpeeds.HOPPER_BOTTOM_BELT_INTAKE),
                new WaitCommand(1.2)
        );

        return new SequentialCommandGroup(
                driveStraight1,
                driveStraight2,
                driveStraight3
        );
    }

    public static SequentialCommandGroup BouncePath(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem v, ShooterSubsystem shooter) {

        ParallelRaceGroup startTurn = new ParallelRaceGroup(
            new VoltageDriveCommand(drivetrain, 0.75, 1),
            new WaitCommand(1.5)
        );

        ParallelRaceGroup initialDriveStraight = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1),
                new WaitCommand(1)
        );

        ParallelRaceGroup turnBounce1 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain, 260),
                new WaitCommand(2)
        );

        ParallelRaceGroup firstBounceDrive = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 1, 0.8),
                new WaitCommand(2.5)
        );

        ParallelRaceGroup firstBounce1 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain, 260),
                new WaitCommand(2)
        );

        ParallelRaceGroup firstBouncePart2 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, -1, 180),
                new WaitCommand(1)
        );

        ParallelRaceGroup firstBouncePart3 = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 1, 0.65),
                new WaitCommand(1)
        );

        ParallelRaceGroup firstBouncePart4 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, -1, 180),
                new WaitCommand(2.5)
        );

        ParallelRaceGroup turnBounce2 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain, 70),
                new WaitCommand(2)
        );

        ParallelRaceGroup secondBouncePart1 = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 0.8, 1),
                new WaitCommand(2)
        );

        ParallelRaceGroup secondBouncePart2 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1, 0),
                new WaitCommand(1)
        );

        ParallelRaceGroup secondBouncePart3 = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 0.8, 1),
                new WaitCommand(1)
        );

        ParallelRaceGroup secondBouncePart4 = new ParallelRaceGroup(
                new DriveStraightCommand(drivetrain, 1, 280),
                new WaitCommand(2.5)
        );

        ParallelRaceGroup turnBounce3 = new ParallelRaceGroup(
                new TurnToAngleCommand(drivetrain, 260),
                new WaitCommand(2)
        );

        ParallelRaceGroup thirdBouncePart1 = new ParallelRaceGroup(
                new VoltageDriveCommand(drivetrain, 0.75, 1),
                new WaitCommand(2)
        );

        return new SequentialCommandGroup(
                startTurn,
                initialDriveStraight,
                turnBounce1,
                firstBounceDrive,
                firstBounce1,
                firstBouncePart2,
                firstBouncePart3,
                firstBouncePart4,
                turnBounce2,
                secondBouncePart1,
                secondBouncePart2,
                secondBouncePart3,
                secondBouncePart4,
                turnBounce3,
                thirdBouncePart1
        );
    }

}
