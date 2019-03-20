package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.CAN;
import frc.robot.constants.Gains;
import frc.robot.constants.Pneumatics;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class DriveTrain {

	private double left, right;
	private CANSparkMax frontLeft, frontRight, midLeft, midRight, backLeft, backRight;
	private CANPIDController rightPID, leftPID;
	private CANEncoder rightEncoder, leftEncoder;
	private ADXRS450_Gyro gyro;
	private DoubleSolenoid shifter;

	private double initialHeading = 0;
	private double currentHeading = 0;
	private double output = 0;
	private double desiredHeading = 0;
	private boolean isAssisting = false;
	public NetworkTable table;
	private NetworkTableEntry angle, driveMode, tapeMode, cargoMode, front;

	private EncoderFollower leftEncoderFollower, rightEncoderFollower;
	private Notifier followerNotifier;
	private Trajectory leftTrajectory, rightTrajectory;
	private double heading = 0;
	private static final String kPathName= "autonomous";
	private boolean pathFinished = false;


	private boolean angleCaptured = false;
	private double setpoint = 0;
	private double error = 0;

	public DriveTrain(int frontLeft, int midLeft, int backLeft, int frontRight, int midRight, int backRight) {

		shifter = new DoubleSolenoid(CAN.kPCM_12, Pneumatics.kDriveTrainForward, Pneumatics.kDriveTrainReverse);

		// left side of drive train
		this.frontLeft = new CANSparkMax(frontLeft, MotorType.kBrushless);
		this.midLeft = new CANSparkMax(midLeft, MotorType.kBrushless);
		this.backLeft = new CANSparkMax(backLeft, MotorType.kBrushless);

		this.frontLeft.setInverted(false);
		this.midLeft.setInverted(false);
		this.backLeft.setInverted(false);


		this.frontLeft.setIdleMode(IdleMode.kCoast);
		this.midLeft.setIdleMode(IdleMode.kCoast);
		this.backLeft.setIdleMode(IdleMode.kCoast);
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

		this.frontRight.setIdleMode(IdleMode.kCoast);
		this.midRight.setIdleMode(IdleMode.kCoast);
		this.backRight.setIdleMode(IdleMode.kCoast);
		// follow front right motor
		this.midRight.follow(this.frontRight);
		this.backRight.follow(this.frontRight);

		this.frontLeft.setSmartCurrentLimit(40);
		this.midLeft.setSmartCurrentLimit(30);
		this.backLeft.setSmartCurrentLimit(40);

		this.frontRight.setSmartCurrentLimit(40);
		this.midRight.setSmartCurrentLimit(30);
		this.backRight.setSmartCurrentLimit(40);

		leftPID = this.frontLeft.getPIDController();
		leftEncoder = this.frontLeft.getEncoder();

		rightPID = this.frontRight.getPIDController();
		rightEncoder = this.frontRight.getEncoder();

		rightEncoder.setPosition(0);
		leftEncoder.setPosition(0);

		leftPID.setP(Gains.kDriveP);
		leftPID.setI(Gains.kDriveI);
		leftPID.setD(Gains.kDriveD);
		leftPID.setIZone(0);
		leftPID.setFF(Gains.kDriveF);
		leftPID.setOutputRange(-1, 1);

		rightPID.setP(Gains.kDriveP);
		rightPID.setI(Gains.kDriveI);
		rightPID.setD(Gains.kDriveD);
		rightPID.setIZone(0);
		rightPID.setFF(Gains.kDriveF);
		rightPID.setOutputRange(-1, 1);
		SmartDashboard.putNumber("rightPID P", rightPID.getP());
		int smartMotionSlot = 0;

		leftPID.setSmartMotionMaxVelocity(800, smartMotionSlot);
		leftPID.setSmartMotionMaxAccel(800, smartMotionSlot);
		leftPID.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
		leftPID.setSmartMotionAllowedClosedLoopError(1, smartMotionSlot);


		rightPID.setSmartMotionMaxVelocity(800, smartMotionSlot);
		rightPID.setSmartMotionMaxAccel(800, smartMotionSlot);
		rightPID.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
		rightPID.setSmartMotionAllowedClosedLoopError(1, smartMotionSlot);

		
		table = NetworkTableInstance.getDefault().getTable("slickvision");
		angle = table.getEntry("tapeYaw");
		tapeMode = table.getEntry("Tape");
		cargoMode = table.getEntry("Cargo");
		driveMode = table.getEntry("Driver");
		front = table.getEntry("Front");

		tapeMode.setBoolean(true);
		cargoMode.setBoolean(false);
		driveMode.setBoolean(false);
		front.setBoolean(true);

		gyro = new ADXRS450_Gyro();
		gyro.calibrate();
	}

	public boolean isPathFinished() {
		return pathFinished;
	}

	public void highGear() {
		shifter.set(Value.kForward);
	} 

	public void lowGear() {
		shifter.set(Value.kReverse);
	}

	public void resetGyro() {
		gyro.reset();
	}

	public void tankDrive(double leftStickValue, double rightStickValue) {
	}

	
	public double getAssistedTurn() {
		double angle = yaw();
		double currentHeading = gyro.getAngle();
		double kP = 0.04;

		desiredHeading = currentHeading + angle;

		System.out.println("Initial Heading :" + initialHeading + " desiredHeading :" + desiredHeading
				+ " : Current Heading : " + currentHeading);
		// double desiredHeading = initialHeading + angle;
		if (Math.abs(desiredHeading - currentHeading) > .5) {
			double out = limit((desiredHeading - currentHeading) * kP,true);
			System.out.println("Output : " + out);
			return out;
		} else {
			System.out.println("not returning");
			return 0;
		}
	}

	public double yaw() {
		return angle.getDouble(0);
	}

	public double getAssistedTurnOnce() {
		double angle = yaw();
		double currentHeading = gyro.getAngle();
		double kP = 0.1;

		if (!isAssisting) {
			desiredHeading = gyro.getAngle() + yaw();
			isAssisting = true;
		}

		if (Math.abs(desiredHeading - currentHeading) > 0.3) {
			double out = limit((desiredHeading - currentHeading) * kP,true);
			System.out.println("Output : " + out + " Desired heading " + desiredHeading);
			return out;
		} else {
			System.out.println("Within Range");
			return 0;
		}
	}

	public void resetAssistance() {
		isAssisting = false;
		initialHeading = 0;
	}

	public void arcadeDrive(double speed, double turn, double turnThreshold) {

		/*
		 * Square the inputs for lower sensitivity
		 */

		speed = Math.copySign(limit(speed * speed,false), speed);
		turn = Math.copySign(limit(turn * turn,false), turn);

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
		arcadeDrive(-speed, 0, 1);
	}

	public void driveAtVelocity(double velocity) {
		leftPID.setReference(velocity, ControlType.kVelocity);
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

	private double limit(double input, boolean assisted) {
		if (!assisted) {
			if (input > 1) {
				input = 1;
			} else if (input < -1) {
				input = -1;
			}
		} else {
			if (input > 0.24) {
				input = 0.24;
			} else if (input < -0.24) {
				input = -0.24;
			}
		}
		return input;

	}

	public void display() {
		SmartDashboard.putNumber("Front Left Encoder Position", leftEncoder.getPosition());
		SmartDashboard.putNumber("Front Left Encoder Velocity", leftEncoder.getVelocity());
		SmartDashboard.putNumber("Front Right Encoder Position", rightEncoder.getPosition());
		SmartDashboard.putNumber("Front Right Encoder Velocity", rightEncoder.getVelocity());
		SmartDashboard.putNumber("Front Right Motor Output", frontRight.getAppliedOutput());
		SmartDashboard.putNumber("Front Left Motor Output", frontLeft.getAppliedOutput());

		SmartDashboard.putNumber("gyro angle", gyro.getAngle());
	}

}