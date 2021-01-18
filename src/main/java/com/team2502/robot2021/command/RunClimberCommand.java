package com.team2502.robot2021.command;

import com.team2502.robot2021.subsystem.ClimberSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunClimberCommand extends CommandBase {
    private final ClimberSubsystem climber;
    private final double speed;

    public RunClimberCommand(ClimberSubsystem climber, double speed) {
        this.climber = climber;
        this.speed = speed;
        addRequirements(climber);
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() { climber.runClimber(speed); }

    @Override
    public void end(boolean interrupted) {
        climber.stopClimber();
    }

    @Override
public boolean isFinished() { return false;/*return !climber.isRetracted();*/ }
}
