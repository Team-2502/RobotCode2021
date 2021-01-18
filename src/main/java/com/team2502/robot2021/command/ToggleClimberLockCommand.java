package com.team2502.robot2021.command;

import com.team2502.robot2021.subsystem.ClimberSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ToggleClimberLockCommand extends InstantCommand {
    private final ClimberSubsystem climber;

    public ToggleClimberLockCommand(ClimberSubsystem climberSubsystem){
        climber = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        if(climber.isRetracted()){
            climber.deploySolenoid();
        }
        else{
            climber.retractSolenoid();
        }
    }
}

