package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;

/**
 * This subsystem controls the Drive Train of the robot using a Differential
 * Drive Style with Commands.
 * 
 * @author Cayden (Kay) Haun/{@link<a href="https://pascalrr.gay">Pascalrr</a>},
 *         FRC Team 1982
 * @version 1.0
 */
public class DriveSubsystem extends SubsystemBase {
    private static DifferentialDrive driveTrain;
    private static DifferentialDriveOdometry driveOdometry;

    private static WPI_TalonSRX leftLeadMotor;
    private static WPI_TalonSRX leftFollowMotor;
    private static WPI_TalonSRX rightLeadMotor;
    private static WPI_TalonSRX rightFollowMotor;

    private static ADIS16448_IMU gyro = new ADIS16448_IMU();

    /**
     * Instantiates a new Drive Subsystem object with the given motor controllers
     * 
     * @param leftLeadID      The CAN Bus ID of the primary left drive motor on the
     *                        drivetrain
     * @param rightLeadID     The CAN Bus ID of the primary right drive motor on the
     *                        drivetrain
     * @param leftFollowerID  The CAN Bus ID of the secondary left drive motor on
     *                        the drivetrain
     * @param rightFollowerID The CAN Bus ID of the secondary right drive motor on
     *                        the drivetrain
     */
    public DriveSubsystem(int leftLeadID, int rightLeadID, int leftFollowerID, int rightFollowerID) {
        // Initialize motors with given IDs
        leftLeadMotor = new WPI_TalonSRX(leftLeadID);
        rightLeadMotor = new WPI_TalonSRX(rightLeadID);

        // Only initalize follower motors if they are enabled
        if (OperatorConstants.enableAllMotors) {
            leftFollowMotor = new WPI_TalonSRX(leftFollowerID);
            rightFollowMotor = new WPI_TalonSRX(rightFollowerID);
        }

        // Configure motors
        leftLeadMotor.configFactoryDefault();
        leftLeadMotor.clearStickyFaults();
        leftLeadMotor.configAllSettings(DriveConstants.motorConfig());
        leftLeadMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        rightLeadMotor.configFactoryDefault();
        rightLeadMotor.clearStickyFaults();
        rightLeadMotor.configAllSettings(DriveConstants.motorConfig());
        rightLeadMotor.setInverted(true);
        rightLeadMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        // Only configure follower motors if they are enabled
        if (OperatorConstants.enableAllMotors) {

            leftFollowMotor.configFactoryDefault();
            leftFollowMotor.clearStickyFaults();
            leftFollowMotor.configAllSettings(DriveConstants.motorConfig());
            leftFollowMotor.follow(leftLeadMotor);

            rightFollowMotor.configFactoryDefault();
            rightFollowMotor.clearStickyFaults();
            rightFollowMotor.configAllSettings(DriveConstants.motorConfig());
            rightFollowMotor.follow(rightLeadMotor);
        }

        // Initialize driveTrain with an empty Pose2d
        driveTrain = new DifferentialDrive(leftLeadMotor, rightLeadMotor);
        driveOdometry = new DifferentialDriveOdometry(getGyroRotation(), leftLeadMotor.getSelectedSensorPosition(),
                rightLeadMotor.getActiveTrajectoryPosition(), new Pose2d());
    }

    @Override
    public void periodic() {
        DifferentialDriveWheelPositions positions = new DifferentialDriveWheelPositions(
                leftLeadMotor.getActiveTrajectoryPosition(), rightLeadMotor.getSelectedSensorPosition());
        driveOdometry.update(getGyroRotation(), positions);
        Logger.recordOutput("Estimated Robot Pose", driveOdometry.getPoseMeters());
    }

    /**
     * Utility function that provides a Rotation2d based of off IMU angles
     * 
     * @return {@link Rotation2d} object from Gyro measurements
     */
    private Rotation2d getGyroRotation() {
        return new Rotation2d(gyro.getGyroAngleX(), gyro.getGyroAngleY());
    }

    /**
     * Utility function that provides a Pose2d estimated from current Odometry data
     * 
     * @return Estimated {@link Pose2d} object from odometry
     */
    public Pose2d getEstimatedPose() {
        return driveOdometry.getPoseMeters();
    }

    /**
     * Utility function that resets the robot odometry to a desired Pose2d
     * 
     * @param pose {@link Pose2d} object to set as current pose
     */
    public void resetPose(Pose2d pose) {
        driveOdometry.resetPose(pose);
    }
    

    /**
     * Utility function to run the Drive Train with a given ChassisSpeeds
     * 
     * @param speeds A {@link ChassisSpeeds} object with desired x, y, and angular velocity
     */
    public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.driveTrainKinematics.toWheelSpeeds(speeds);
        driveTrain.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    /**
     * Function that allows for direct control of the Drive Train
     * 
     * @param driveSpeed Lateral speed to drive
     * @param angularSpeed Rotational speed to drive
     */
    public void simpleArcadeDrive(double driveSpeed, double angularSpeed) {
        driveTrain.arcadeDrive(driveSpeed, angularSpeed);
    }

    /**
     * Command to drive the robot relative to it's current position
     * 
     * @param xSpeed       Supplier for X-direction velocity to drive
     * @param ySpeed       Supplier for Y-direction velocity to drive
     * @param angularSpeed Supplier for Angular velocity to turn
     * @return {@link Command} for running the Drive Train
     */
    public Command driveRobotRelative(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier angularSpeed) {
        return run(() -> {
            ChassisSpeeds speeds = new ChassisSpeeds(xSpeed.getAsDouble(), ySpeed.getAsDouble(),
                    angularSpeed.getAsDouble());
            driveWithChassisSpeeds(speeds);
        });
    }
}
