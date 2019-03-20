package frc.robot.mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Mode;
import frc.robot.constants.CAN;
import frc.robot.constants.Pneumatics;

public class Intake {

    private VictorSPX left,right;

    private Mode mode;
    private Solenoid intakeManipulator;
    private Ultrasonic us;

    private boolean cargoObtained = false;
    private boolean hatchObtained = false;
    private boolean hatchLock = false;

    private boolean automatic = false;

    public Intake(int left, int right) {

        this.left = new VictorSPX(left);
        this.right = new VictorSPX(right);

        intakeManipulator = new Solenoid(CAN.kPCM_12, Pneumatics.kIntake);
        us = new Ultrasonic(0, 1);
        us.setDistanceUnits(Ultrasonic.Unit.kInches);
        us.setAutomaticMode(true);
        us.setEnabled(true);
    }

    public static void setAutomaticMode(boolean automatic){
        if (automatic) {
            automatic = true;
        } else {
            automatic = false;
        }
    }

    public boolean getAutomaticMode() {
        return automatic;
    }


    public void autoCargoIntake(double speed) {

        if (us.getRangeInches() <= 8.25) {
            close();
            cargoObtained = true;
        } else {
            cargoObtained = false;
            open();
            intake(speed);
        }
    }


    public void autoHatchIntake() {

        if (us.getRangeInches() <= 11.2) {
            hatchLock = true;
            hatchObtained = true;
            open();
        } else {
            if (!hatchLock) {
                close();
            }
        }

    }

    public void display() {
        SmartDashboard.putNumber("Ultrasonic Distance", us.getRangeInches());
    }

    public void open() {
        intakeManipulator.set(true);
    }

    public void close() {
        intakeManipulator.set(false);
        hatchLock = false;
    }

    public void intake(double speed) {
        left.set(ControlMode.PercentOutput, speed);
        right.set(ControlMode.PercentOutput, -speed);
    }

    public void shoot(double speed) {
        left.set(ControlMode.PercentOutput, -speed);
        right.set(ControlMode.PercentOutput, speed);
    }

    public void stop() {
        left.set(ControlMode.PercentOutput, 0);
        right.set(ControlMode.PercentOutput, 0);
    }

}