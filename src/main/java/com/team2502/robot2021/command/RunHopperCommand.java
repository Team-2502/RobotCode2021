package com.team2502.robot2021.command;

import com.team2502.robot2021.subsystem.HopperSubsystem;
import com.team2502.robot2021.subsystem.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunHopperCommand extends CommandBase {

    private final HopperSubsystem hopper;
    private final ShooterSubsystem shooter;
    private final double speedLeft;
    private final double speedRight;
    private final double speedBottom;
    private final double speedWheel;
    private final boolean waitForFlywheel;

    public RunHopperCommand(HopperSubsystem hopper, ShooterSubsystem shooter,
                            double speedLeft,
                            double speedRight,
                            double speedBottom,
                            double speedWheel,
                            boolean waitForFlywheel){
        this.speedLeft = speedLeft;
        this.speedRight = speedRight;
        this.speedBottom = speedBottom;
        this.speedWheel = speedWheel;
        this.hopper = hopper;
        this.shooter = shooter;
        this.waitForFlywheel = waitForFlywheel;
        addRequirements(hopper);
    }


    @Override
    public void execute() {
        hopper.runLeftBelt(speedLeft);
        hopper.runRightBelt(speedRight);
        hopper.runBottomBelt(speedBottom);
        hopper.runExitWheel(speedWheel);

    }

    @Override
    public void end(boolean interrupted) {
        hopper.runLeftBelt(0);
        hopper.runRightBelt(0);
        hopper.runBottomBelt(0);
        hopper.runExitWheel(0);
    }

    @Override
    public boolean isFinished() {
        return waitForFlywheel && !shooter.isShooterRunning();
    }
}
