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
 * I
 */
public class Arm extends SubsystemBase {
	private final int kArmLeftCanID = 40;
	private final int kArmRightCanID = 41;
	private final int kEncoderDIO = 0;

	private final double kPosOffset = 0.0;

	private static final String kSetName = "Set Motor";

	/** Follows the right motor controller */
	private final CANSparkMax mArmLeft = new CANSparkMax(kArmLeftCanID, MotorType.kBrushless);
	private final CANSparkMax mArmRight = new CANSparkMax(kArmRightCanID, MotorType.kBrushless);
	
	private final DutyCycleEncoder mArmEnc = new DutyCycleEncoder(kEncoderDIO);
	private final PIDController mArmPid = new PIDController(0.0, 0.0, 0.0);
	private final ArmFeedforward mArmFf = new ArmFeedforward(0.0, 0.0, 0.0);

	private double mSet;

	public Arm() {
		this.mArmLeft.follow(this.mArmRight, true);
	}

	public Command cmdRun() {
		return this.run(() -> {
			double set = SmartDashboard.getNumber(kSetName, 0.0);
			if(mSet != set) this.mArmRight.set(set);
		});
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Encoder", this.mArmEnc.getAbsolutePosition() - kPosOffset);
	}
}
