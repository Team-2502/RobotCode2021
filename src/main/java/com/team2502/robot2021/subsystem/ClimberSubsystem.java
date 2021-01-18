package com.team2502.robot2021.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team2502.robot2021.Constants;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team2502.robot2021.Constants.RobotMap.Motors;

public class ClimberSubsystem extends SubsystemBase {
    private final WPI_TalonSRX climberMotor;
    private final Solenoid climberSolenoid;

    public ClimberSubsystem() {
        climberMotor = new WPI_TalonSRX(Motors.CLIMBER);
        climberSolenoid = new Solenoid(Constants.RobotMap.Solenoid.CLIMBER);
        deploySolenoid();
    }

    public void runClimber(double speed) { climberMotor.set(speed); }

    public void stopClimber() { climberMotor.set(0);}

    public void retractSolenoid() { climberSolenoid.set(true); }

    public void deploySolenoid() {
        climberSolenoid.set(false);
    }

    public boolean isRetracted() { return climberSolenoid.get(); }

}


