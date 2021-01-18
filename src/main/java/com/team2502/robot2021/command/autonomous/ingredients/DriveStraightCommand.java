package com.team2502.robot2021.command.autonomous.ingredients;

import com.team2502.robot2021.Constants;
import com.team2502.robot2021.subsystem.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveStraightCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;

    private final double speed;
    private double targetAngle;
    private final double kp = Constants.Robot.Auto.DRIVE_STRAIGHT_KP;

    private final PIDController pidControllerError;

    private final boolean absoluteMode;

    public DriveStraightCommand(DrivetrainSubsystem drivetrain, double speed) {
        this.drivetrain = drivetrain;
        this.speed = speed;
        this.pidControllerError = new PIDController(kp, 0, 0);
        absoluteMode = false;
        addRequirements(drivetrain);
    }

    public DriveStraightCommand(DrivetrainSubsystem drivetrain, double speed, double targetAngle) {
        this.drivetrain = drivetrain;
        this.speed = speed;
        this.pidControllerError = new PIDController(kp, 0, 0);
        this.targetAngle = targetAngle;
        absoluteMode = true;
        addRequirements(drivetrain);
    }



    @Override
    public void initialize()
    {
        if(!absoluteMode) {
            this.targetAngle = drivetrain.getHeading();
        }

        pidControllerError.setSetpoint(0); // we want 0 error
    }

    @Override
    public void execute()
    {
        double currentAngle = drivetrain.getHeading();

        double error = angleDiff(targetAngle, currentAngle);

        // Angular velocity is the change in error and also the change in absolute angle because taking the derivative eliminates constants
        // and the initial angle is a constant
        // Learn calculus for more information
        double desiredWheelDifferential = pidControllerError.calculate(error);

        drivetrain.getDrive().tankDrive(speed + desiredWheelDifferential, speed - desiredWheelDifferential);
    }

    /**
     * Difference between 2 angles, accounting for -180 = 180
     *
     * @param targetAngle angle you want to be at
     * @param currentAngle angle you are actually at
     * @return the number of degrees you must change current angle by to reach target angle.
     */
    private double angleDiff(double targetAngle, double currentAngle) {
        double diff = targetAngle - currentAngle;

        if(diff > 180) {
            diff -= 360;
        } else if (diff < -180) {
            diff += 360;
        }

        return diff;
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
