package frc.robot.mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.constants.Gains;

public class Wrist extends TalonSRX {


    private double feedFwd = 0;
    private double wristParallel = Constants.kWristParallel;
    private double wristMaxAngleForward = Constants.kWristMaxAngleForward;
    private double wristMaxAngleReverse = Constants.kWristMaxAngleReverse;

    public Wrist(int i) {
        super(i);

        super.configFactoryDefault();

        super.setNeutralMode(NeutralMode.Brake);

        super.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
        super.setSensorPhase(false);

        super.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
        super.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);

        super.selectProfileSlot(0, 0);
        super.config_kP(0, Gains.kWristP);
        super.config_kI(0, Gains.kWristI);
        super.config_kD(0, Gains.kWristD);
        super.config_kF(0, Gains.kWristF);

        super.configNominalOutputForward(0, 0);
		super.configNominalOutputReverse(0, 0);
		super.configPeakOutputForward(.5, 0);
		super.configPeakOutputReverse(-.5, 0);

        super.configMotionAcceleration(115);
        super.configMotionCruiseVelocity(115);

        super.configAllowableClosedloopError(0, 5);

        super.setSelectedSensorPosition(super.getSensorCollection().getPulseWidthPosition(), 0, 0);

        super.configMotionSCurveStrength(1);
    }

    public void moveTo(double target) {
        if (super.getSelectedSensorPosition() < Constants.kWristParallel) {
            feedFwd = 0.05;
        } else {
            feedFwd = -0.05;
        }
        super.set(ControlMode.MotionMagic, target, DemandType.ArbitraryFeedForward, feedFwd);
    }

    public void stop() {
        super.set(ControlMode.PercentOutput, 0);
    }

    public void display() {
        SmartDashboard.putNumber("Wrist Position", super.getSelectedSensorPosition());
    }
}