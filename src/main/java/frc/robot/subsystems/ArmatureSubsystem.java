package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * This subsystem controls the armature of the robot for grabbing game pieces
 * 
 * @author Cayden (Kay) Haun/{@link<a href="https://pascalrr.gay">Pascalrr</a>},
 *         FRC Team 1982
 * @version 1.0
 */
public class ArmatureSubsystem extends SubsystemBase {
    private static WPI_TalonSRX shoulderMotor;

    public ArmatureSubsystem(int shoulderID) {
        shoulderMotor = new WPI_TalonSRX(shoulderID);
    }

    public void raiseArm(double amount) {
        shoulderMotor.set(amount);
    }

    public Command runArmCommand(DoubleSupplier liftAmount) {
        return run(() -> {
            raiseArm(-liftAmount.getAsDouble());
        });
    }

}
