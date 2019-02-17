package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Gains;

public class DriveTrain {

	private double left, right;
	private CANSparkMax frontLeft, frontRight, midLeft, midRight, backLeft, backRight;
	private CANPIDController rightPID, leftPID;
	private CANEncoder rightEncoder, leftEncoder;
	private ADXRS450_Gyro gyro;

	private boolean angleCaptured = false;
	private double setpoint = 0;
	private double error = 0;

	public DriveTrain(int frontLeft, int midLeft, int backLeft, int frontRight, int midRight, int backRight) {

		// left side of drive train
		this.frontLeft = new CANSparkMax(frontLeft, MotorType.kBrushless);
		this.midLeft = new CANSparkMax(midLeft, MotorType.kBrushless);
		this.backLeft = new CANSparkMax(backLeft, MotorType.kBrushless);

		this.frontLeft.setInverted(false);
		this.midLeft.setInverted(false);
		this.backLeft.setInverted(false);


		this.frontLeft.setIdleMode(IdleMode.kBrake);
		this.midLeft.setIdleMode(IdleMode.kBrake);
		this.backLeft.setIdleMode(IdleMode.kBrake);
		// follow front left motor
		this.midLeft.follow(this.frontLeft);
		this.backLeft.follow(this.frontLeft);

		// right side of drive train
		this.frontRight = new CANSparkMax(frontRight, MotorType.kBrushless);
		this.midRight = new CANSparkMax(midRight, MotorType.kBrushless);
		this.backRight = new CANSparkMax(backRight, MotorType.kBrushless);

		this.frontRight.setInverted(true);
		this.midRight.setInverted(true);
		this.backRight.setInverted(true);

		this.frontRight.setIdleMode(IdleMode.kBrake);
		this.midRight.setIdleMode(IdleMode.kBrake);
		this.backRight.setIdleMode(IdleMode.kBrake);
		// follow front right motor
		this.midRight.follow(this.frontRight);
		this.backRight.follow(this.frontRight);

		leftPID = this.frontLeft.getPIDController();
		leftEncoder = this.frontLeft.getEncoder();

		//rightEncoder.setPosition(0);

		leftPID.setP(Gains.kDriveP);
		leftPID.setI(Gains.kDriveI);
		leftPID.setD(Gains.kDriveD);
		leftPID.setIZone(0);
		leftPID.setFF(Gains.kDriveF);
		leftPID.setOutputRange(-1, 1);


		int smartMotionSlot = 0;
	//	rightPID.setSmartMotionMaxVelocity(100, smartMotionSlot);
	//	rightPID.setSmartMotionMaxAccel(50, smartMotionSlot);
	//	rightPID.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
	//	rightPID.setSmartMotionAllowedClosedLoopError(1, smartMotionSlot);

		gyro = new ADXRS450_Gyro();
		gyro.calibrate();
	}

	public void resetGyro() {
		gyro.reset();
	}

	public void turn(double angle) {
		//17.2 revolutions per radian
		//2pi (6.28319) radians per 360 degrees
		double target = angle * 17.2;
		leftPID.setReference(target, ControlType.kPosition);
		SmartDashboard.putNumber("Drive Motor Output", this.frontLeft.getAppliedOutput());

	}

	public void tankDrive(double leftStickValue, double rightStickValue) {
	}

	public void arcadeDrive(double speed, double turn, double turnThreshold) {

		/*
		 * Square the inputs for lower sensitivity
		 */

		speed = Math.copySign(limit(speed * speed), speed);
		turn = Math.copySign(limit(turn * turn), turn);

		if (Math.abs(turn) > turnThreshold) {
			left = speed + turn;
			right = speed - turn;
			angleCaptured = false;
		} else {
			left = speed;
			right = speed;

			/*
			 * if (!angleCaptured) { setpoint = gyro.getAngle(); angleCaptured = true; }
			 * 
			 * error = (gyro.getAngle() - setpoint) * Gains.kDriveAngleP;
			 * 
			 * left = speed + error; right = speed - error;
			 * 
			 * 
			 * }
			 */
		}
		left(left);
		right(right);
	}

	public void driveStraight(double speed) {
		arcadeDrive(speed, 0, 1);
	}

	public void stopDriving() {
		left(0);
		right(0);
	}

	private void left(double speed) {
		frontLeft.set(speed);
	}

	private void right(double speed) {
		frontRight.set(speed);
	}

	private double limit(double input) {
		if (input > 1) {
			input = 1;
		}
		if (input < -1) {
			input = -1;
		}
		return input;
	}

	public void display() {
		SmartDashboard.putNumber("Front Left Encoder Position", leftEncoder.getPosition());
		SmartDashboard.putNumber("Front Left Encoder Velocity", leftEncoder.getVelocity());
		SmartDashboard.putNumber("gyro angle", gyro.getAngle());
	}

}