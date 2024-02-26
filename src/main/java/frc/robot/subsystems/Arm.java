package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* -- Calculated Gains --
 * kS : 0.34018
 * kV : 6.4525
 * kA : 11.213
 * kG : 0.21586
 * 
 * - REV Brushless Encoder PID
 * kP : 5.8225
 * kD : 3.4811
 * 
 * - WPILib 2020 <
 * kP : 67.437
 * kD : 40.744
 */

/**
 * Arm subsystem
 */
public class Arm extends SubsystemBase {
	private static final double kS = 0.34018;
	private static final double kV = 6.4525;
	private static final double kA = 11.213;
	private static final double kG =  0.21586;

	private static final double kP = 48.0;
	private static final double kD = 36.0;

	private final int kArmLeftCanID = 35;
	private final int kArmRightCanID = 34;
	private final int kEncoderDIO = 0;

	private final double kPosOffset = 1.0 - 0.000835;

	/** Follows the right motor controller */
	private final CANSparkMax mArmLeft = new CANSparkMax(kArmLeftCanID, MotorType.kBrushless);
	private final CANSparkMax mArmRight = new CANSparkMax(kArmRightCanID, MotorType.kBrushless);

	private final DutyCycleEncoder mArmEnc = new DutyCycleEncoder(kEncoderDIO);
	/** PID: attempt to use same gains as the SparkMAX */
	private final PIDController mArmPid = new PIDController(kP, 0.0, kD);
	/** Use volts 4 units: S:volts, G:volts, V:(volt*sec)/rad, A:(volt*sec^2)/rad */
	private final ArmFeedforward mArmFf = new ArmFeedforward(kS, kG, kV, kA);

	private final SparkPIDController mSparkPID = this.mArmRight.getPIDController();

	private final Timer mTimer = new Timer();

	public Arm() {
		this.mArmLeft.follow(this.mArmRight, true);

		SmartDashboard.putNumber("Set Radians", 0.0);
		SmartDashboard.putNumber("MotorVoltage", 0.0);

		this.setDefaultCommand(this.run(() -> {
			this.mArmRight.stopMotor();
			this.mArmLeft.stopMotor();
			this.mArmPid.setSetpoint(this.getEncoderDist());
		}));

		this.mTimer.restart();

		this.mSparkPID.setP(kP / (2.0 * Math.PI) / 12.0);
		this.mSparkPID.setI(0.0);
		this.mSparkPID.setD(kD / (2.0 * Math.PI) / 12.0);

		this.mSparkPID.setSmartMotionMaxVelocity(0.008333333333333333, 0);
	}

	public Command cmdRun() {
		return Commands.sequence(
			this.runOnce(() -> {
				this.mArmPid.setSetpoint(SmartDashboard.getNumber("Set Radians", 0.0));
			}),
			this.run(() -> {
				double rot = SmartDashboard.getNumber("Set Radians", 0.0);

				SmartDashboard.putNumber("MotorVoltage",
					this.mArmPid.calculate(this.getEncoderDist()) +
					this.mArmFf.calculate(rot, 0.0)
				);

				/*
				this.mArmRight.set(
					MathUtil.clamp(
						this.mArmPid.calculate(this.getEncoderDist()) + this.mArmFf.calculate(rot, 0.0),
						-6.0, 6.0
					) / RobotController.getBatteryVoltage()
				);
				*/

				this.mSparkPID.setReference(rot, ControlType.kSmartMotion, 0, this.mArmFf.calculate(rot, 0.0), ArbFFUnits.kVoltage);
			})
		);
	}

	public Command cmdFollowOsc() {
		return Commands.sequence(
			this.run(() -> {
				double rot = this.getOscGoal();
				this.mArmPid.setSetpoint(rot);

				SmartDashboard.putNumber("MotorVoltage",
					this.mArmPid.calculate(this.getEncoderDist()) +
					this.mArmFf.calculate(rot, 0.0)
				);

				/*
				this.mArmRight.set(
					MathUtil.clamp(
						this.mArmPid.calculate(this.getEncoderDist()) + this.mArmFf.calculate(rot, 0.0),
						-6.0, 6.0
					) / RobotController.getBatteryVoltage()
				);
				*/

				this.mSparkPID.setReference(rot, ControlType.kSmartMotion, 0, this.mArmFf.calculate(rot, 0.0), ArbFFUnits.kVoltage);
			})
		);
	}


	@Override
	public void periodic() {
		SmartDashboard.putNumber("Encoder", this.getEncoderDist());
		SmartDashboard.putNumber("OscSetp", this.getOscGoal());
	}

	private double getEncoderDist() {
		return (-this.mArmEnc.getDistance() + kPosOffset) * 2.0 * Math.PI;
	}

	private double getOscGoal() {
		return 0.785398 + 0.5235987755982988 * Math.sin(this.mTimer.get() * 2.0);
	}
}
