// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * The Operator Constants are primarily user oriented configuration options that
   * can be changed without
   * affecting major functionality of the codebase.
   */
  public static class OperatorConstants {
    // Physical port of driver controller on the Driver Station
    public static final int driverControllerPort = 0;
    // Change this if you want to replay from AdvantageKit in simulation
    public static final boolean replayLogs = false;
    // If no follower motors are connected to the robot, set this to false
    public static final boolean enableAllMotors = false;
  }

  /**
   * The Drive Constants directly control the functionality of the Drive
   * Subsystem, and are not meant to be
   * tampered with unless you understand exactly what each constant is for, and
   * what it may or may not break.
   */
  public static class DriveConstants {
    public static final DifferentialDriveKinematics driveTrainKinematics = new DifferentialDriveKinematics(
        Units.inchesToMeters(20));

    // Default motor configuration
    private static final TalonSRXConfiguration defaultConfig = new TalonSRXConfiguration();
    public static final TalonSRXConfiguration motorConfig() {
      defaultConfig.peakCurrentLimit = 30;
      defaultConfig.peakCurrentDuration = 1500; // time in MS
      defaultConfig.continuousCurrentLimit = 25;
      return defaultConfig;
    }
    
    // Motor CANBUS IDs
    public static final int leftLeadMotorID = 3;
    public static final int leftFollowMotorID = 4;
    public static final int rightLeadMotorID = 2;
    public static final int rightFollowMotorID = 1;
  }

}
