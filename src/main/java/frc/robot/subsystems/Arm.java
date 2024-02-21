package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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
	private final int kArmLeftCanID = 40;
	private final int kArmRightCanID = 41;
	private final int kEncoderDIO = 0;

	private final double kPosOffset = 0.0;

	/** Follows the right motor controller */
	private final CANSparkMax mArmLeft = new CANSparkMax(kArmLeftCanID, MotorType.kBrushless);
	private final CANSparkMax mArmRight = new CANSparkMax(kArmRightCanID, MotorType.kBrushless);

	private final DutyCycleEncoder mArmEnc = new DutyCycleEncoder(kEncoderDIO);
	/** PID: attempt to use same gains as the SparkMAX */
	private final PIDController mArmPid = new PIDController(0.0, 0.0, 0.0);
	/** Use volts 4 units: S:volts, G:volts, V:(volt*sec)/rot, A:(volt*sec^2)/rot */
	private final ArmFeedforward mArmFf = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);

	private double mArmEncPos = 0.0;
	private double mArmEncVel = 0.0;

	private final MutableMeasure<Voltage> mAppliedVolts = MutableMeasure.mutable(Units.Volts.of(0.0));
	private final MutableMeasure<Angle> mArmPos = MutableMeasure.mutable(Units.Radians.of(0.0));
	private final MutableMeasure<Velocity<Angle>> mArmVel = MutableMeasure.mutable(Units.RadiansPerSecond.of(0.0));

	private final SysIdRoutine mSysIdRout = new SysIdRoutine(
		new SysIdRoutine.Config(),
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
						this.mArmEncVel * 2.0 * Math.PI / 60.0, Units.RadiansPerSecond
					));
			},
			this
		)
	);

	public Arm() {
		this.mArmLeft.follow(this.mArmRight, true);

		SmartDashboard.putNumber("Set Motor", 0.0);
		SmartDashboard.putNumber("Set Rotations", 0.0);
		SmartDashboard.putNumber("MotorVoltage", 0.0);

		this.setDefaultCommand(this.run(() -> {
			this.mArmRight.stopMotor();
			this.mArmLeft.stopMotor();
		}));
	}

	public Command cmdRun() {
		return this.run(() -> {
			double set = SmartDashboard.getNumber("Set Motor", 0.0);
			double rot = SmartDashboard.getNumber("Set Rotations", 0.0);

			SmartDashboard.putNumber("MotorVoltage",
				this.mArmPid.calculate(this.mArmEnc.getAbsolutePosition()) +
				this.mArmFf.calculate(rot * 2.0 * Math.PI, 0.0)
			);
		});
	}

	@Override
	public void periodic() {
		double pos = this.mArmEnc.getAbsolutePosition();
		this.mArmEncVel = (pos - this.mArmEncPos) / 0.02 / 60.0; // Rotations per minute
		this.mArmEncPos = this.mArmEnc.getAbsolutePosition() - kPosOffset;
	}

	public Command cmdQuasistatic(final Direction dir) {
		return this.mSysIdRout.quasistatic(dir);
	}

	public Command cmdDynamic(final Direction dir) {
		return this.mSysIdRout.quasistatic(dir);
	}	
}
