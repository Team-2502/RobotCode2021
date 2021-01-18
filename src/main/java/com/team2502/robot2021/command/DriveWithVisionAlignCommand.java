package com.team2502.robot2021.command;

import com.team2502.robot2021.Constants;
import com.team2502.robot2021.subsystem.DrivetrainSubsystem;
import com.team2502.robot2021.subsystem.VisionSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveWithVisionAlignCommand extends CommandBase {
    private final VisionSubsystem vision;
    private final DrivetrainSubsystem drive;
    private final Joystick leftJoystick;
    private final Joystick rightJoystick;
    double leftPower;
    double rightPower;
    double backOfRange;

    private boolean seesTarget;

    private double p;
    private double frictionConstant;

    public DriveWithVisionAlignCommand(VisionSubsystem vision_subsystem, DrivetrainSubsystem drive_subsystem, Joystick leftJoystick, Joystick rightJoystick, double backOfRange){
        vision = vision_subsystem;
        drive = drive_subsystem;
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
        this.backOfRange = backOfRange;
        seesTarget = false;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        if(drive.isHighGear()){
            frictionConstant = Constants.Robot.Vision.FRICTION_HIGH;
            p = Constants.Robot.Vision.VISION_TURNING_P_HIGH;
        }
        else{
            frictionConstant = Constants.Robot.Vision.FRICTION_LOW;
            p = Constants.Robot.Vision.VISION_TURNING_P_LOW;
        }

    }

    @Override
    public void execute() {

        double tx = vision.getTx();
        double steering_adjust = 0.0f;

        seesTarget = vision.getArea() != 0.0;

        if(seesTarget) {
            double power;
            double userDesiredValue = -(leftJoystick.getY() + rightJoystick.getY()) / 2;

            if(vision.getDistance() < backOfRange - 2) {
                power = Math.min(-0.2 - frictionConstant, userDesiredValue);
            }
            else if (backOfRange - 2 <= vision.getDistance() && vision.getDistance() <= backOfRange) {
                power = Math.min(0, userDesiredValue);
            }
            else {
                power = userDesiredValue;
            }

            boolean useFriction = power < frictionConstant;
            double frictionVal = useFriction ? frictionConstant : 0;

            if (tx > 1.0) {
                steering_adjust = p * tx + frictionVal;
            } else if (tx < 1.0) {    //robot needs to turn left
                steering_adjust = p * tx - frictionVal;
            }
            leftPower = steering_adjust;
            rightPower = -steering_adjust;
            drive.getDrive().tankDrive(leftPower + power, rightPower + power);
        }
        else {
            drive.getDrive().tankDrive(-leftJoystick.getY(), -rightJoystick.getY(), true);
        }
    }

    @Override
    public boolean isFinished() { return false; }
}
