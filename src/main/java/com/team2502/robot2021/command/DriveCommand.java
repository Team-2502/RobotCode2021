package com.team2502.robot2021.command;

import com.team2502.robot2021.subsystem.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
    private final SendableChooser<Drivetype> typeEntry = new SendableChooser<>();

    private final DrivetrainSubsystem drivetrain;
    private final Joystick leftJoystick;
    private final Joystick rightJoystick;

    public DriveCommand(DrivetrainSubsystem drivetrain, Joystick joystickDriveLeft, Joystick joystickDriveRight) {
        this.drivetrain = drivetrain;
        leftJoystick = joystickDriveLeft;
        rightJoystick = joystickDriveRight;

        typeEntry.addOption("Split Arcade", Drivetype.Arcade);
        typeEntry.addOption("Reverse", Drivetype.Reverse);
        typeEntry.setDefaultOption("Tank", Drivetype.Tank);
        SmartDashboard.putData("Drive Type", typeEntry);
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        switch(typeEntry.getSelected()) {
            case Tank:
                drivetrain.getDrive().tankDrive(-leftJoystick.getY(), -rightJoystick.getY(), true);
                break;
            case Arcade:
                drivetrain.getDrive().arcadeDrive(-leftJoystick.getY(), rightJoystick.getX(), true);
                break;
            case Reverse:
                drivetrain.getDrive().tankDrive(leftJoystick.getY(), rightJoystick.getY(), true);
        }
    }

    private enum Drivetype {
        Tank,
        Arcade,
        Reverse
    }
}
