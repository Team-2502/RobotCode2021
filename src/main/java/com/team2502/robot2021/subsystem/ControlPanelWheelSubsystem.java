package com.team2502.robot2021.subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team2502.robot2021.Constants;
import com.team2502.robot2021.Constants.RobotMap.Motors;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControlPanelWheelSubsystem extends SubsystemBase {
 public final CANSparkMax controlPanel;
 private final Solenoid controlPanelSolenoid;

 public ControlPanelWheelSubsystem(){
  controlPanel = new CANSparkMax(Motors.CONTROL_PANEL, CANSparkMaxLowLevel.MotorType.kBrushless);
  controlPanel.setSmartCurrentLimit(20);
  controlPanelSolenoid = new Solenoid(Constants.RobotMap.Solenoid.CONTROL_PANEL);
  retractSolenoid();
 }
 @Override
 public void periodic(){
 }
 public void runMotor(double speed){
  controlPanel.set(speed);
 }

 public void deploySolenoid() { controlPanelSolenoid.set(true); }

 public void retractSolenoid() {
  controlPanelSolenoid.set(false);
 }

 public boolean isUp() { return controlPanelSolenoid.get(); }
}
