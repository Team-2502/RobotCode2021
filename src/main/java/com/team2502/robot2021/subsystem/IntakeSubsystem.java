package com.team2502.robot2021.subsystem;

import com.team2502.robot2021.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

    private final CANSparkMax intakeMotor;
    private final CANSparkMax squeezeMotor;

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.RobotMap.Motors.INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        intakeMotor.setSmartCurrentLimit(25);
        squeezeMotor = new CANSparkMax(Constants.RobotMap.Motors.SQUEEZE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        squeezeMotor.setSmartCurrentLimit(25);
    }

    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }

    public void runSqueeze(double speed) {
        squeezeMotor.set(speed);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    public void stopSqueeze() {
        squeezeMotor.set(0);
    }
    
}
