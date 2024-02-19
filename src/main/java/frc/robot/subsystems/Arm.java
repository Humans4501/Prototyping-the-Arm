package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

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

	private final double kPosOffset = 0.0;

	/** Follows the right motor controller */
	private final CANSparkMax mArmLeft = new CANSparkMax(kArmLeftCanID, MotorType.kBrushless);
	private final CANSparkMax mArmRight = new CANSparkMax(kArmRightCanID, MotorType.kBrushless);

	private final DutyCycleEncoder mArmEnc = new DutyCycleEncoder(kEncoderDIO);
	/** PID: attempt to use same gains as the SparkMAX */
	private final PIDController mArmPid = new PIDController(0.0, 0.0, 0.0);
	/** Use volts 4 units: S:volts, G:volts, V:(volt*sec)/rot, A:(volt*sec^2)/rot */
	private final ArmFeedforward mArmFf = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);

	private double mSet, mP, mI, mD, mFfks, mFfkg, mFfkv;

	public Arm() {
		this.mArmLeft.follow(this.mArmRight, true);

		SmartDashboard.putNumber("Set Motor", 0.0);
		SmartDashboard.putNumber("Set Rotations", 0.0);
		SmartDashboard.putNumber("p", 0.0);
		SmartDashboard.putNumber("i", 0.0);
		SmartDashboard.putNumber("d", 0.0);
		SmartDashboard.putNumber("g", 0.0);
		SmartDashboard.putNumber("v", 0.0);
		SmartDashboard.putNumber("a", 0.0);

		this.setDefaultCommand(this.run(() -> {
			this.mArmRight.stopMotor();
			this.mArmLeft.stopMotor();
		}));
	}

	public Command cmdRun() {
		return this.run(() -> {
			double set = SmartDashboard.getNumber("Set Motor", 0.0);
			double rot = SmartDashboard.getNumber("Set Rotations", 0.0);
			double p = SmartDashboard.getNumber("p", 0.0);
			double i = SmartDashboard.getNumber("i", 0.0);
			double d = SmartDashboard.getNumber("d", 0.0);
			double s = SmartDashboard.getNumber("g", 0.0);
			double g = SmartDashboard.getNumber("v", 0.0);
			double v = SmartDashboard.getNumber("a", 0.0);

			if(mSet != set) { this.mSet = set; this.mArmRight.set(set); }
			if(mP != p) { this.mP = p; this.mArmPid.setP(p); }
			if(mI != i) { this.mI = i; this.mArmPid.setI(i); }
			if(mD != d) { this.mD = d; this.mArmPid.setD(d); }
			if(mFfks != s) { this.mFfks = s; this.mArmFf.setS(s); }
			if(mFfkg != g) { this.mFfkg = g; this.mArmFf.setG(g); }
			if(mFfkv != v) { this.mFfkv = v; this.mArmFf.setV(v); }

			SmartDashboard.putNumber("MotorVoltage",
				this.mArmPid.calculate(this.mArmEnc.getAbsolutePosition()) +
				this.mArmFf.calculate(rot * 2.0 * Math.PI, 0.0)
			);
		});
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Encoder", this.mArmEnc.getAbsolutePosition() - kPosOffset);
	}
}
