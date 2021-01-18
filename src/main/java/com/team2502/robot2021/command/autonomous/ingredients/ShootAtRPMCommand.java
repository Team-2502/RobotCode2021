package com.team2502.robot2021.command.autonomous.ingredients;

import com.team2502.robot2021.subsystem.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootAtRPMCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final double speed;

    public ShootAtRPMCommand(ShooterSubsystem shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
        addRequirements(shooter);
    }

    @Override
    public void execute(){ shooter.setShooterSpeedRPM(speed); }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){ shooter.stopShooter(); }

}
