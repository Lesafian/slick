
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

/*
Hardware Setup:

1 x 775 pro motor
->
180 degree gearbox 2:1
->
100:1 versaplanetary
->
2:1 gear reduction

1 rot of versaplanetary
= 1/2 rotation output shaft

*/

public class Elbow extends TalonSRX {

    private double feedFwdVoltage = 0.05;
    private boolean extended = false;

    public Elbow(int i) {
        super(i);

        //reset motor controller
        super.configFactoryDefault();

        super.setNeutralMode(NeutralMode.Brake);

        super.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
        super.setSensorPhase(false);

        super.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
        
        super.selectProfileSlot(0,0);
        super.config_kP(0, Gains.kElbowP);
        super.config_kI(0, Gains.kElbowI);
        super.config_kD(0, Gains.kElbowD);
        super.config_kF(0, Gains.kElbowF);

        super.configNominalOutputForward(0, 0);
		super.configNominalOutputReverse(0, 0);
		super.configPeakOutputForward(.7, 0);
		super.configPeakOutputReverse(-.7, 0);

        super.configMotionAcceleration(1700);
        super.configMotionCruiseVelocity(1500);
        super.configMotionSCurveStrength(8);

        super.setSelectedSensorPosition(0,0,0);
    }

    public double getFeedForward() {
        double feedfwd = (super.getSelectedSensorPosition() - Constants.kElbowParallelForward) / ((Constants.kElbowParallelReverse - Constants.kElbowParallelForward) / Math.PI);
        System.out.println(feedfwd);
        return feedfwd;
    }

    public void moveTo(double target) {
        System.out.println(target);
        if (extended) {
            feedFwdVoltage = 0.08;
        } else {
            feedFwdVoltage = 0.05;
        }
        super.set(ControlMode.MotionMagic, target, DemandType.ArbitraryFeedForward, 0.05 * Math.cos(getFeedForward()));
    }

    public void stop() {
        super.set(ControlMode.PercentOutput, 0);
    }

    public void setExtended(boolean extended) {
        this.extended = extended;
    }

    public void display() {
        SmartDashboard.putNumber("Elbow Position", super.getSelectedSensorPosition());
    }
}
