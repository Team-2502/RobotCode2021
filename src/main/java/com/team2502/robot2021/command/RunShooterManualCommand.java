
package com.team2502.robot2021.command;

import com.team2502.robot2021.subsystem.ShooterSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunShooterManualCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final double defaultSpeed;
    private final Joystick controlJoystick;

    public RunShooterManualCommand(ShooterSubsystem shooter, double defaultSpeed, Joystick controlJoystick) {
        this.shooter = shooter;
        this.defaultSpeed = defaultSpeed; // middle of range
	this.controlJoystick = controlJoystick; // joystick with lever (throttle)
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterSpeedRPM(defaultSpeed); // set to middle of range on init
    }

    @Override
    public void execute() {
	double speedInput = controlJoystick.getThrottle(); // get lever value
	double targetRpm = (1-speedInput)*defaultSpeed; // value decreases as you twist the lever up, ranges from -1 to 1 (mapped to 0-2)
    	shooter.setShooterSpeedRPM(targetRpm);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter(); // this will coast
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
