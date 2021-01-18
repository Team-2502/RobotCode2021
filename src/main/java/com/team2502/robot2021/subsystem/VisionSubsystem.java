package com.team2502.robot2021.subsystem;

import com.team2502.robot2021.Constants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team2502.robot2021.Constants.Robot.Vision.LIMELIGHT_NETWORK_TABLE;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelight;

    private double tX;
    private double tY;
    private double area;

    public VisionSubsystem() {
        limelight = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NETWORK_TABLE);
    }

    @Override
    public void periodic() {
        NetworkTableEntry TX_ENTRY = limelight.getEntry("tx");
        NetworkTableEntry TY_ENTRY = limelight.getEntry("ty");
        NetworkTableEntry AREA_ENTRY = limelight.getEntry("ta");

        tX = TX_ENTRY.getDouble(0.0);
        tY = TY_ENTRY.getDouble(0.0);
        area = AREA_ENTRY.getDouble(0.0);

        SmartDashboard.putNumber("Limelight X", tX);
        SmartDashboard.putNumber("Limelight Y", tY);
        SmartDashboard.putNumber("Limelight Area", area);
        SmartDashboard.putNumber("Distance from target", getDistance());
    }

    public double getTx(){
        return tX;
    }

    public double getArea(){
        return area;
    }

    public void limeLightOff(){
        limelight.getEntry("ledMode").setNumber(1);
    }

    public void limeLightOn(){
        limelight.getEntry("ledMode").setNumber(3);
    }

    public void limeLightPipeLine(){
        limelight.getEntry("ledMode").setNumber(0);
    }

    /**
     * KnightKrawler method of determining distance: lookup table on tY
     *
     * @return dist in feet
     */
    public double getDistance(){
        return Constants.LookupTables.TY_TO_DIST_TABLE.get(tY);
    }

    public double getOptimalShooterSpeed() {
        double distToTarget = Constants.LookupTables.TY_TO_DIST_TABLE.get(tY);

        return Constants.LookupTables.DIST_TO_RPM_TABLE.get(distToTarget);
    }
}
