package com.team2502.robot2021.command;

import com.team2502.robot2021.subsystem.ControlPanelWheelSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunControlPanelWheelCommand extends CommandBase {
    private final ControlPanelWheelSubsystem controlPanel;
    private final double speed;

public RunControlPanelWheelCommand(ControlPanelWheelSubsystem control_panel, double speed){
    controlPanel = control_panel;
    this.speed = speed;
    addRequirements(control_panel);
}

@Override
    public void initialize(){
    controlPanel.runMotor(speed);
}
@Override
    public void end(boolean interrupted){
controlPanel.runMotor(0);
}
@Override
    public boolean isFinished(){
    return false;
}
}
