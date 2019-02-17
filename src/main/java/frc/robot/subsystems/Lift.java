package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Mode;
import frc.robot.Position;
import frc.robot.constants.Constants;
import frc.robot.constants.Gains;

public class Lift extends TalonSRX {

    private Position pos;
    private double feedFwd = 0;
    private double level1 = Constants.kLiftBottom;
    private double level2 = Constants.kLiftBottom;
    private double level3 = Constants.kLiftBottom;

    public Lift(int i) {
        super(i);
        super.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        super.config_kP(0, Gains.kLiftP);
        super.config_kI(0, Gains.kLiftI);
        super.config_kD(0, Gains.kLiftD);
        super.config_kF(0, Gains.kLiftF);

        super.setInverted(true);

        super.setSensorPhase(true);
        super.configMotionAcceleration(5000);
        super.configMotionCruiseVelocity(8000);

        super.setNeutralMode(NeutralMode.Brake);

        super.configReverseSoftLimitEnable(true);
        super.configForwardSoftLimitEnable(true);
        super.configReverseSoftLimitThreshold(-1000);
        super.configForwardSoftLimitThreshold(40000);

        super.configAllowableClosedloopError(0, 25);
    }

    private void move(double target) {
        // .0625 = holding %vbus
        if (pos == Position.BOTTOM && super.getSelectedSensorPosition() <= Constants.kLiftBottomThreshold) {
            stop();
        } else {
            if (target < super.getSelectedSensorPosition()) {
                feedFwd = -0.125;
            } else {
                feedFwd = 0.0625;
            }
            super.set(ControlMode.MotionMagic, target, DemandType.ArbitraryFeedForward, feedFwd);
        }
    }

    public void moveTo(Mode mode, boolean ground,Position position) {
        this.pos = position;
        switch(mode) {
            case HATCH_PANEL:
            level1 = Constants.kLiftLevel1HatchPanel;
            level2 = Constants.kLiftLevel2HatchPanel;
            level3 = Constants.kLiftLevel3HatchPanel;
            break;

            case CARGO:
            level1 = Constants.kLiftLevel1Cargo;
            level2 = Constants.kLiftLevel2Cargo;
            level3 = Constants.kLiftLevel3Cargo;
            break;
        }

        switch (position) {
        case BOTTOM:
            move(Constants.kLiftBottom);
            break;
        case PICKUP:
            if (!ground && mode == Mode.HATCH_PANEL) {
                move(Constants.kLiftHatchPanelPickup);
            } else {
                move(Constants.kLiftBottom);
            } 
            break;
        case LEVEL_1:
            move(level1);
            break;
        case LEVEL_2:
            move(level2);
            break;
        case LEVEL_3:
            move(level3);
            break;
        }
    }

    public void stop() {
        super.set(ControlMode.PercentOutput, 0);
    }

    /*
    public Position positionRange() {
        Position position;
        double count = super.getSelectedSensorPosition();
        if (count <= Constants.kLiftLevel1) {
            position = Position.BOTTOM;
        } else if (count <= Constants.kLiftLevel2) {
            position = Position.LEVEL_1;
        } else if (count <= Constants.kLiftLevel2) {
            position = Position.LEVEL_2;
        } else {
            position = Position.LEVEL_3;
        }

        return position;
    }
    */
    
    public void display() {
        SmartDashboard.putNumber("Lift Position", super.getSelectedSensorPosition());
        SmartDashboard.putNumber("Lift Velocity", super.getSelectedSensorVelocity());
        SmartDashboard.putNumber("% Output", super.getMotorOutputPercent());
        SmartDashboard.putNumber("Motor Voltage", super.getMotorOutputVoltage());
        SmartDashboard.putNumber("Motor Current", super.getOutputCurrent());
    }

}