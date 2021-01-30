package com.team2502.robot2021.subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.team2502.robot2021.Constants;
import com.team2502.robot2021.Constants.RobotMap.Motors;
import com.team2502.robot2021.Constants.Robot.Auto;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

    private final Solenoid drivetrainSolenoid;

    private final AHRS navX;

    private final DifferentialDrive drive;

    private final WPI_TalonFX drivetrainFrontLeft;
    private final WPI_TalonFX drivetrainBackLeft;
    private final WPI_TalonFX drivetrainFrontRight;
    private final WPI_TalonFX drivetrainBackRight;

    public DrivetrainSubsystem() {
        drivetrainBackLeft = new WPI_TalonFX(Motors.DRIVE_BACK_LEFT);
        drivetrainFrontLeft = new WPI_TalonFX(Motors.DRIVE_FRONT_LEFT);
        drivetrainBackRight = new WPI_TalonFX(Motors.DRIVE_BACK_RIGHT);
        drivetrainFrontRight = new WPI_TalonFX(Motors.DRIVE_FRONT_RIGHT);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

        drivetrainFrontRight.configAllSettings(configs);
        drivetrainFrontLeft.configAllSettings(configs);

        drivetrainBackLeft.follow(drivetrainFrontLeft);
        drivetrainBackRight.follow(drivetrainFrontRight);

        drivetrainFrontRight.setInverted(TalonFXInvertType.CounterClockwise);
        drivetrainFrontLeft.setInverted(TalonFXInvertType.CounterClockwise);

        drivetrainFrontRight.setNeutralMode(NeutralMode.Coast);
        drivetrainFrontLeft.setNeutralMode(NeutralMode.Coast);

        navX = new AHRS(SPI.Port.kMXP);
        resetHeading();
        resetEncoders();

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

    public double getLeftEncoderPosition(WPI_TalonFX talon, boolean highGear){
        if(highGear){
            return talon.getSelectedSensorPosition() * Auto.ENCODER_DPP_HIGH;
        }
        else{
            return talon.getSelectedSensorPosition() * Auto.ENCODER_DPP_LOW;
        }
    }

    public double getRightEncoderPosition(WPI_TalonFX talon, boolean highGear){
        if(highGear){
            return -talon.getSelectedSensorPosition() * Auto.ENCODER_DPP_HIGH;
        }
        else{
            return -talon.getSelectedSensorPosition() * Auto.ENCODER_DPP_LOW;
        }
    }

    public double getLeftEncoderSpeed(WPI_TalonFX talon, boolean highGear){
        if(highGear){
            return talon.getSelectedSensorVelocity() * 10 * Auto.ENCODER_DPP_HIGH;
        }
        else {
            return talon.getSelectedSensorVelocity() * 10 * Auto.ENCODER_DPP_LOW;
        }
    }

    public double getRightEncoderSpeed(WPI_TalonFX talon, boolean highGear){
        if(highGear){
            return -talon.getSelectedSensorVelocity() * 10 * Auto.ENCODER_DPP_HIGH;
        }
        else {
            return -talon.getSelectedSensorVelocity() * 10 * Auto.ENCODER_DPP_LOW;
        }
    }

    public void resetEncoders() {
        drivetrainFrontRight.setSelectedSensorPosition(0);
        drivetrainFrontLeft.setSelectedSensorPosition(0);
        drivetrainBackLeft.setSelectedSensorPosition(0);
        drivetrainBackRight.setSelectedSensorPosition(0);
    }

    public double getHeading() { return Math.IEEEremainder(navX.getAngle(), 360) * (Constants.Robot.Auto.GYRO_REVERSED ? -1 : 1); }

    public void resetHeading(){
        navX.reset();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Front Position", getLeftEncoderPosition(drivetrainFrontLeft, isHighGear()));
        SmartDashboard.putNumber("Right Front Position", getRightEncoderPosition(drivetrainFrontRight, isHighGear()));
        SmartDashboard.putNumber("Left Back Position", getLeftEncoderPosition(drivetrainBackLeft, isHighGear()));
        SmartDashboard.putNumber("Right Back Position", getRightEncoderPosition(drivetrainBackRight, isHighGear()));

        SmartDashboard.putNumber("Left Front Enc Ticks", drivetrainFrontLeft.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Front Enc Ticks", drivetrainFrontRight.getSelectedSensorPosition());
        SmartDashboard.putNumber("Pose Angle", getHeading());

        SmartDashboard.putNumber("Right Front Velocity", getRightEncoderSpeed(drivetrainFrontRight, isHighGear()));
        SmartDashboard.putNumber("Left Front Velocity", getLeftEncoderSpeed(drivetrainFrontLeft, isHighGear()));
        SmartDashboard.putNumber("Right Back Velocity", getRightEncoderSpeed(drivetrainBackRight, isHighGear()));
        SmartDashboard.putNumber("Left Back Velocity", getLeftEncoderSpeed(drivetrainBackLeft, isHighGear()));

        SmartDashboard.putBoolean("High Gear", isHighGear());
    }
}
