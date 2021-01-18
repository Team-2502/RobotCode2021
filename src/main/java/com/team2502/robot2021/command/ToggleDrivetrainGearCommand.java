package com.team2502.robot2021.command;

import com.team2502.robot2021.subsystem.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ToggleDrivetrainGearCommand extends InstantCommand {

    private final DrivetrainSubsystem drivetrain;

    public ToggleDrivetrainGearCommand(DrivetrainSubsystem drivetrain)
    {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        if(drivetrain.isHighGear()){
            drivetrain.enterLowGear();
        }
        else{
            drivetrain.enterHighGear();
        }
    }
}
