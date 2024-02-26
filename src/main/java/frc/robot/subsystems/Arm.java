package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Arm subsystem
 */
public class Arm extends SubsystemBase {
	private final int kArmLeftCanID = 35;
	private final int kArmRightCanID = 34;

	private final double kPosOffset = 1.0 - 0.000835;

	/** Follows the right motor controller */
	private final CANSparkMax mArmLeft = new CANSparkMax(kArmLeftCanID, MotorType.kBrushless);
	private final CANSparkMax mArmRight = new CANSparkMax(kArmRightCanID, MotorType.kBrushless);
	private final DutyCycleEncoder mEncoder = new DutyCycleEncoder(0);

	public Arm() {
		this.mArmLeft.follow(this.mArmRight, true);

		SmartDashboard.putNumber("SetMotor", 0.0);

		this.setDefaultCommand(this.run(() -> {
			this.mArmRight.stopMotor();
			this.mArmLeft.stopMotor();
		}));
	}

	public Command cmdRun() {
		return this.run(() -> {
			double set = SmartDashboard.getNumber("SetMotor", 0.0);

			this.mArmRight.set(set);
		});
	}

	public Command cmdRunReverse() {
		return this.run(() -> {
			double set = SmartDashboard.getNumber("SetMotor", 0.0);

			this.mArmRight.set(-set);
		});

	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Encoder", -this.mEncoder.getAbsolutePosition() + kPosOffset);
	}
}
