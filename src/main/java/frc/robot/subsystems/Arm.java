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
 * I
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
	private final PIDController mArmPid = new PIDController(0.0, 0.0, 0.0);
	private final ArmFeedforward mArmFf = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);

	private double mSet, mP, mI, mD, mFfks, mFfkg, mFfkv;

	public Arm() {
		this.mArmLeft.follow(this.mArmRight, true);
	}

	public Command cmdRun() {
		return this.run(() -> {
			double set = SmartDashboard.getNumber("Set Motor", 0.0);
			double p = SmartDashboard.getNumber("P", 0.0);
			double i = SmartDashboard.getNumber("P", 0.0);
			double d = SmartDashboard.getNumber("P", 0.0);
			double s = SmartDashboard.getNumber("P", 0.0);
			double g = SmartDashboard.getNumber("P", 0.0);
			double v = SmartDashboard.getNumber("P", 0.0);
			if(mSet != set) { this.mSet = set; this.mArmRight.set(set); }
			if(mP != p) { this.mP = p; this.mArmPid.setP(p); }
			if(mI != i) { this.mI = i; this.mArmPid.setI(i); }
			if(mD != d) { this.mD = d; this.mArmPid.setD(d); }
			if(mFfks != s) { this.mFfks = s; this.mArmFf.setS(s); }
			if(mFfkg != g) { this.mFfkg = g; this.mArmFf.setG(g); }
			if(mFfkv != v) { this.mFfkv = v; this.mArmFf.setV(v); }
		});
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Encoder", this.mArmEnc.getAbsolutePosition() - kPosOffset);
	}
}
