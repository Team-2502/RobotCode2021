/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team2502.robot2021;

import com.team2502.robot2021.util.LookupTable;

import java.util.HashMap;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class OI {

        public static final int JOYSTICK_DRIVE_RIGHT = 0;
        public static final int JOYSTICK_DRIVE_LEFT = 1;
        public static final int JOYSTICK_OPERATOR = 2;

        // Buttons on JOYSTICK_DRIVE_RIGHT
        public static final int BUTTON_SHIFT = 1;
        public static final int BUTTON_BLUE_ZONE = 2;
        public static final int BUTTON_YELLOW_ZONE = 3;
        public static final int BUTTON_RED_ZONE = 4;

        // Buttons on JOYSTICK_DRIVE_LEFT
        public static final int BUTTON_VISION_ALIGN = 1;

        // Buttons on JOYSTICK_OPERATOR
        public static final int BUTTON_HOPPER_CONTINUOUS = 1;
        public static final int BUTTON_HOPPER_CONTINUOUS_REVERSE = 2;

        public static final int BUTTON_RUN_SHOOTER_INIT_LINE = 6;
        public static final int BUTTON_RUN_SHOOTER_TRENCH = 4;
        public static final int BUTTON_RUN_INTAKE = 5;
        public static final int BUTTON_RUN_INTAKE_BACKWARDS = 3;

        public static final int BUTTON_CLIMBER = 10;
        public static final int BUTTON_CLIMBER_REVERSE = 12;
        public static final int BUTTON_CLIMBER_ACTUATE = 7;
        public static final int BUTTON_BOTTOM_ROLLER_BACKWARDS = 8;
        public static final int BUTTON_CONTROL_PANEL = 9;
        public static final int BUTTON_ACTUATE_CONTROL_PANEL = 11;
    }

    public static final class RobotMap {

        public static final class Motors {
            // Talon FX
            public static final int DRIVE_FRONT_RIGHT = 1;
            public static final int DRIVE_FRONT_LEFT = 3;
            public static final int DRIVE_BACK_RIGHT = 2;
            public static final int DRIVE_BACK_LEFT = 4;

            // Talon SRX
            public static final int CLIMBER = 15;

            // SparkMax
            public static final int SHOOTER_LEFT = 12;
            public static final int SHOOTER_RIGHT = 2;

            public static final int HOPPER_SIDE_BELTS_LEFT = 3;
            public static final int HOPPER_SIDE_BELTS_RIGHT = 4;
            public static final int HOPPER_BOTTOM_BELT = 5;
            public static final int HOPPER_EXIT_WHEEL = 6;

            public static final int SQUEEZE_MOTOR = 24;
            public static final int INTAKE_MOTOR = 8;
              
            public static final int CONTROL_PANEL = 57;
        }

        public static final class Solenoid {
            public static final int DRIVETRAIN = 0;
            public static final int CLIMBER = 1;
            public static final int CONTROL_PANEL = 2;
        }
    }

    public static final class Robot {

        public static final class Shooter{
            public static final double SHOOTER_P = 0.0008;
            public static final double SHOOTER_I = 0.0;
            public static final double SHOOTER_D = 0.0;
            public static final double SHOOTER_IZ = 0;
            public static final double SHOOTER_FF = 0.00019;
            public static final double SHOOTER_MAX_OUTPUT = 1;
            public static final double SHOOTER_MIN_OUTPUT = -0.1;
        }

        public static final class Vision {

//          public static final double FRICTION_LOW = .255; // Practice bot
            public static final double FRICTION_LOW = 0.29; // Comp bot

//          public static final double FRICTION_HIGH = .27; // Practice Bot
            public static final double FRICTION_HIGH = 0.452; // Comp Bot

            public static final double VISION_TURNING_P_LOW = 0.015;
            public static final double VISION_TURNING_P_HIGH = 0.02;

//          public static final String LIMELIGHT_NETWORK_TABLE = "limelight-acid"; // Practice Bot
            public static final String LIMELIGHT_NETWORK_TABLE = "limelight-orion"; // Comp Bot

            public static final double STANDARD_DISTANCE = 12;
            public static final double YELLOW_ZONE_DISTANCE = 9.6;
            public static final double BLUE_ZONE_DISTANCE = 14.6;
            public static final double RED_ZONE_DISTANCE = 18.5;
        }

        public static final class MotorSpeeds {

            public static final double INTAKE_SPEED_FORWARD = .85;
            public static final double INTAKE_SPEED_BACKWARDS = -1;
            public static final double INTAKE_SQUEEZE_SPEED_FORWARDS = 0.6;
            public static final double INTAKE_SQUEEZE_SPEED_BACKWARDS = -1;

            public static final double HOPPER_LEFT_BELT = 1;
            public static final double HOPPER_RIGHT_BELT = .25;
            public static final double HOPPER_BOTTOM_BELT = 1;
            public static final double HOPPER_BOTTOM_BELT_INTAKE = 0.25;

            public static final double HOPPER_EXIT_WHEEL = 1;
            public static final double HOPPER_LEFT_BELT_REVERSE = -1;
            public static final double HOPPER_RIGHT_BELT_REVERSE = -1;
            public static final double HOPPER_BOTTOM_BELT_REVERSE = -1;
            public static final double HOPPER_EXIT_WHEEL_REVERSE = -1;

            public static final double CLIMBER_FORWARD = 1;
            public static final double CLIMBER_BACKWARD = -1;

            public static final double CONTROL_PANEL = 0.6;
        }

        public static final class Auto {
            public static final double WHEEL_DIAMETER = 0.1524;

            public static final int ENCODER_CPR = 2048;
            public static final double ENCODER_CONVERSION_HIGH = (11f * 24f) / (42f * 50f);
            public static final double ENCODER_CONVERSION_LOW = (11f * 14f) / (42f * 60f);

//            public static final double ENCODER_DPP_HIGH = (WHEEL_DIAMETER * Math.PI) / ((double) ENCODER_CPR * ENCODER_CONVERSION_HIGH);
//            public static final double ENCODER_DPP_LOW = (WHEEL_DIAMETER * Math.PI) / ((double) ENCODER_CPR * ENCODER_CONVERSION_LOW);

            // Units: meters / enc units
            public static final double ENCODER_DPP_HIGH = 2.84189006248405e-5;
            public static final double ENCODER_DPP_LOW = 1.369566308404673e-5;

            public static final boolean GYRO_REVERSED = true;

            public static final double TURN_TOLERANCE_DEG = 1;
            public static final double TURN_RATE_TOLERANCE_DEG_PER_SEC = 1;

            public static final double DRIVE_STRAIGHT_KP = 0.01;
            public static final double TURN_TO_ANGLE_KP = 0.015;

            public static final double KS = 0.629;
            public static final double KV = 3.68;
            public static final double KA = 0.32;
        }
    }

    public static final class LookupTables {
        public static final LookupTable TY_TO_DIST_TABLE;

        public static final LookupTable DIST_TO_RPM_TABLE;

        static {
            HashMap<Double, Double> tyToDistMap = new HashMap<>();
            tyToDistMap.put(2.06D, 8D);
            tyToDistMap.put(1.00D, 9D);
            tyToDistMap.put(0D, 10D);
            tyToDistMap.put(-1.13D, 11D);
            tyToDistMap.put(-2.7, 12D);
            tyToDistMap.put(-4.2, 13D);
            tyToDistMap.put(-5.26, 14D);
            tyToDistMap.put(-6.7, 15D);
            tyToDistMap.put(-7.67, 16D);
            tyToDistMap.put(-8.60, 17D);
            tyToDistMap.put(-9.59, 18D);
            tyToDistMap.put(-10.3, 19D);
            tyToDistMap.put(-10.95, 20D);
            tyToDistMap.put(-11.67, 21D);
            tyToDistMap.put(-12.18, 22D);
            tyToDistMap.put(-12.78, 23D);
            tyToDistMap.put(-13.5, 24D);
            tyToDistMap.put(-13.8, 25D);
            tyToDistMap.put(-14.3D, 26D);

            TY_TO_DIST_TABLE = new LookupTable(tyToDistMap);

            HashMap<Double, Double> distToRpmMap = new HashMap<>();
            distToRpmMap.put(10D, 3840D);
            distToRpmMap.put(15D, 3728D);
            distToRpmMap.put(18D, 3728D);
            distToRpmMap.put(19D, 3728D);
            distToRpmMap.put(20D, 3735D);
            distToRpmMap.put(21D, 3857D);
            distToRpmMap.put(23D, 3862D);
            distToRpmMap.put(25D, 3950D);
            distToRpmMap.put(30D, 4300D);
            distToRpmMap.put(35D, 4900D);

            DIST_TO_RPM_TABLE = new LookupTable(distToRpmMap);
        }
    }

}
