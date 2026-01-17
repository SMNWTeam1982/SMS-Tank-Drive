package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private static SparkMax intakeMotor;

    public IntakeSubsystem(int intakeID) {
        intakeMotor = new SparkMax(intakeID, MotorType.kBrushless);
    }

    public void runIntake(double amount) {
        intakeMotor.set(amount);
    }

    public Command runIntake() {
        return run(() -> {
            runIntake(0.65);
        });
    }

    public Command runEject() {
        return run(() -> {
            runIntake(-0.65);
        });
    }

    public Command stopIntake() {
        return run(() -> {
            intakeMotor.stopMotor();
        });
    }
}
