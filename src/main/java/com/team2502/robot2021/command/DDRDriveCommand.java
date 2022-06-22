package com.team2502.robot2021.command;

import com.team2502.robot2021.subsystem.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.team2502.robot2021.util.KonamiHandler;
import com.team2502.robot2021.util.KonamiHandler.BUTTONS;
import com.team2502.robot2021.util.Trapezoidal;
import com.team2502.robot2021.Constants;

public class DDRDriveCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final Joystick groovyJoystick;
    private Trapezoidal speedTrapezoidal;
    private Trapezoidal rotationTrapezoidal;
    private KonamiHandler konamiHandler;

    public DDRDriveCommand(DrivetrainSubsystem drivetrain, Joystick groovyJoystick) {
        this.drivetrain = drivetrain;
        this.groovyJoystick = groovyJoystick;
        this.speedTrapezoidal = new Trapezoidal(2);
        this.rotationTrapezoidal = new Trapezoidal(2);
        this.konamiHandler = new KonamiHandler();

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double speed = 0;
        double rotation = 0;

        double topSpeed = konamiHandler.isFinished() ? 1 : .4;

        if (groovyJoystick.getRawButtonPressed(Constants.OI.DDR_UP)) {
            konamiHandler.handle(BUTTONS.UP);
        }
        if (groovyJoystick.getRawButtonPressed(Constants.OI.DDR_DOWN)) {
            konamiHandler.handle(BUTTONS.DOWN);
        }
        if (groovyJoystick.getRawButtonPressed(Constants.OI.DDR_LEFT)) {
            konamiHandler.handle(BUTTONS.LEFT);
        }
        if (groovyJoystick.getRawButtonPressed(Constants.OI.DDR_RIGHT)) {
            konamiHandler.handle(BUTTONS.RIGHT);
        }
        if (groovyJoystick.getRawButtonPressed(Constants.OI.DDR_FLYWHEEL)) {
            konamiHandler.handle(BUTTONS.A);
        }
        if (groovyJoystick.getRawButtonPressed(Constants.OI.DDR_TURRET_RIGHT)) {
            konamiHandler.handle(BUTTONS.B);
        }

        if (groovyJoystick.getRawButton(Constants.OI.DDR_UP)) {
            speed = topSpeed;
        }
        if (groovyJoystick.getRawButton(Constants.OI.DDR_DOWN)) {
            speed = -topSpeed;
        }
        if (groovyJoystick.getRawButton(Constants.OI.DDR_LEFT)) {
            rotation = -topSpeed;
        }
        if (groovyJoystick.getRawButton(Constants.OI.DDR_RIGHT)) {
            rotation = topSpeed;
        }
        
        drivetrain.getDrive().arcadeDrive(speedTrapezoidal.calculate(speed), rotationTrapezoidal.calculate(rotation));
    }
}
