package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Arm subsystem
 */
public class Arm extends SubsystemBase {
	private final int kArmLeftCanID = 40;
	private final int kArmRightCanID = 41;
	private final int kEncoderDIO = 0;

	private final double kPosOffset = 1.0 - 0.000835;

	/** Follows the right motor controller */
	private final CANSparkMax mArmLeft = new CANSparkMax(kArmLeftCanID, MotorType.kBrushless);
	private final CANSparkMax mArmRight = new CANSparkMax(kArmRightCanID, MotorType.kBrushless);

	private final DutyCycleEncoder mArmEnc = new DutyCycleEncoder(kEncoderDIO);
	/** PID: attempt to use same gains as the SparkMAX */
	private final PIDController mArmPid = new PIDController(0.0, 0.0, 0.0);
	/** Use volts 4 units: S:volts, G:volts, V:(volt*sec)/rot, A:(volt*sec^2)/rot */
	private final ArmFeedforward mArmFf = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);

	public Arm() {
		this.mArmLeft.follow(this.mArmRight, true);

		SmartDashboard.putNumber("Set Radians", 0.0);
		SmartDashboard.putNumber("MotorVoltage", 0.0);

		this.setDefaultCommand(this.run(() -> {
			this.mArmRight.stopMotor();
			this.mArmLeft.stopMotor();
		}));
	}

	public Command cmdRun() {
		return this.run(() -> {
			double rot = SmartDashboard.getNumber("Set Radians", 0.0);

			SmartDashboard.putNumber("MotorVoltage",
				this.mArmPid.calculate(this.mArmEnc.getAbsolutePosition() * 2.0 * Math.PI) +
				this.mArmFf.calculate(rot, 0.0)
			);
		});
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Encoder", (this.mArmEnc.getAbsolutePosition() * 2.0 * Math.PI) - kPosOffset);
	}
}
