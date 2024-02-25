package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * Arm subsystem
 */
public class Arm extends SubsystemBase {
	private final int kArmLeftCanID = 35;
	private final int kArmRightCanID = 34;
	private final int kEncoderDIO = 0;

	private final double kPosOffset = 1.0 - 0.000835;

	/** Follows the right motor controller */
	private final CANSparkMax mArmLeft = new CANSparkMax(kArmLeftCanID, MotorType.kBrushless);
	private final CANSparkMax mArmRight = new CANSparkMax(kArmRightCanID, MotorType.kBrushless);

	private final DutyCycleEncoder mArmEnc = new DutyCycleEncoder(kEncoderDIO);

	/** Arm encoder position (rotations) */
	private double mArmEncPos = 0.0;
	/** Arm encoder velocity (rotations/second) */
	private double mArmEncVel = 0.0;

	private final MutableMeasure<Voltage> mAppliedVolts = MutableMeasure.mutable(Units.Volts.of(0.0));
	private final MutableMeasure<Angle> mArmPos = MutableMeasure.mutable(Units.Radians.of(0.0));
	private final MutableMeasure<Velocity<Angle>> mArmVel = MutableMeasure.mutable(Units.RadiansPerSecond.of(0.0));

	private final SysIdRoutine mSysIdRout = new SysIdRoutine(
		new SysIdRoutine.Config(
			null, Units.Volts.of(1.2), null, null
		),
		new SysIdRoutine.Mechanism(
			(Measure<Voltage> volts) -> {
				this.mArmRight.set(volts.in(Units.Volts) / RobotController.getBatteryVoltage());
			},
			log -> {
				log.motor("arm-motor")
					.voltage(this.mAppliedVolts.mut_replace(
						this.mArmRight.get() * RobotController.getBatteryVoltage(), Units.Volts
					))
					.angularPosition(this.mArmPos.mut_replace(
						this.mArmEncPos * 2.0 * Math.PI, Units.Radians
					))
					.angularVelocity(this.mArmVel.mut_replace(
						this.mArmEncVel * 2.0 * Math.PI, Units.RadiansPerSecond
					));
			},
			this
		)
	);

	public Arm() {
		this.mArmLeft.follow(this.mArmRight, true);

		this.setDefaultCommand(this.run(() -> {
			this.mArmRight.stopMotor();
			this.mArmLeft.stopMotor();
		}));
	}

	@Override
	public void periodic() {
		double pos = -this.mArmEnc.getAbsolutePosition() + kPosOffset;
		this.mArmEncVel = (pos - this.mArmEncPos) / 0.02; // Rotations per second
		this.mArmEncPos = pos;

		SmartDashboard.putNumber("ArmEncVel", this.mArmEncVel);
		SmartDashboard.putNumber("ArmEncPos", this.mArmEncPos);
	}

	public Command cmdQuasistatic(final Direction dir) {
		return this.mSysIdRout.quasistatic(dir);
	}

	public Command cmdDynamic(final Direction dir) {
		return this.mSysIdRout.quasistatic(dir);
	}
}
