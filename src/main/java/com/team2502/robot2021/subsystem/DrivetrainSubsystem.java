package com.team2502.robot2021.subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.team2502.robot2021.Constants;
import com.team2502.robot2021.Constants.RobotMap.Motors;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

    private final Solenoid drivetrainSolenoid;

    private final AHRS navX;

    private final DifferentialDrive drive;

    public DrivetrainSubsystem() {
        WPI_TalonFX drivetrainBackLeft = new WPI_TalonFX(Motors.DRIVE_BACK_LEFT);
        WPI_TalonFX drivetrainFrontLeft = new WPI_TalonFX(Motors.DRIVE_FRONT_LEFT);
        WPI_TalonFX drivetrainBackRight = new WPI_TalonFX(Motors.DRIVE_BACK_RIGHT);
        WPI_TalonFX drivetrainFrontRight = new WPI_TalonFX(Motors.DRIVE_FRONT_RIGHT);

        drivetrainBackLeft.follow(drivetrainFrontLeft);
        drivetrainBackRight.follow(drivetrainFrontRight);

        drivetrainFrontRight.setInverted(TalonFXInvertType.CounterClockwise);
        drivetrainFrontLeft.setInverted(TalonFXInvertType.CounterClockwise);

        drivetrainFrontRight.setNeutralMode(NeutralMode.Coast);
        drivetrainFrontLeft.setNeutralMode(NeutralMode.Coast);

        navX = new AHRS(SPI.Port.kMXP);
        resetHeading();

        drive = new DifferentialDrive(drivetrainFrontLeft, drivetrainFrontRight);

        drivetrainSolenoid = new Solenoid(Constants.RobotMap.Solenoid.DRIVETRAIN);
        drivetrainSolenoid.set(false);
    }

    public DifferentialDrive getDrive() {
        return drive;
    }

    public void enterHighGear() { drivetrainSolenoid.set(true); }

    public void enterLowGear() { drivetrainSolenoid.set(false); }

    public boolean isHighGear() { return drivetrainSolenoid.get(); }

    public double getHeading() { return Math.IEEEremainder(navX.getAngle(), 360) * (Constants.Robot.Auto.GYRO_REVERSED ? -1 : 1); }

    public void resetHeading(){
        navX.reset();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Angle", getHeading());
        SmartDashboard.putBoolean("High Gear", isHighGear());
    }
}