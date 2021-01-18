package com.team2502.robot2021.command;

import com.team2502.robot2021.subsystem.HopperSubsystem;
import com.team2502.robot2021.subsystem.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntakeCommand extends CommandBase {
    private final double speedIntake;
    private final double speedSqueeze;
    private final double speedBottom;
    private final IntakeSubsystem intake;
    private final HopperSubsystem hopper;

    public RunIntakeCommand(IntakeSubsystem subsystem, HopperSubsystem hopper, double speedIntake, double speedSqueeze, double speedBottom) {
        intake = subsystem;
        this.hopper = hopper;
        this.speedIntake = speedIntake;
        this.speedSqueeze = speedSqueeze;
        this.speedBottom = speedBottom;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        intake.runIntake(speedIntake);
        intake.runSqueeze(speedSqueeze);
        hopper.runBottomBelt(speedBottom);
    }

    @Override
    public void execute() {
        intake.runIntake(speedIntake);
        intake.runSqueeze(speedSqueeze);
        hopper.runBottomBelt(speedBottom);
    }

    @Override
    public void end(boolean kInterrupted){
        intake.stopIntake();
        intake.stopSqueeze();
        hopper.runBottomBelt(0);
    }

    @Override
    public boolean isFinished() {
        return false;

    }

}
