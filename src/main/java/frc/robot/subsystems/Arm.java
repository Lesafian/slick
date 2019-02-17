package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Mode;
import frc.robot.Position;
import frc.robot.constants.Constants;
import frc.robot.constants.Pneumatics;
import frc.robot.mechanisms.Elbow;
import frc.robot.mechanisms.Wrist;

public class Arm {

    private Elbow elbow;
    private Wrist wrist;
    private DoubleSolenoid extender;
    private double elbowParallel = Constants.kElbowParallelForward;
    private double elbowMaxAngle = Constants.kElbowMaxAngleForward;
    private double elbowVertical = Constants.kElbowVertical;
    private double wristParallel = Constants.kWristParallel;
    private double wristMaxAngle = Constants.kWristParallel;

    public Arm(Elbow elbow, Wrist wrist) {
        this.elbow = elbow;
        this.wrist = wrist;
        extender = new DoubleSolenoid(1, Pneumatics.kArmForward, Pneumatics.kArmReverse);
    }


    public void extend() {
        extender.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        extender.set(DoubleSolenoid.Value.kReverse);
    }

    /*
     * if !cargo then hatchpanel
     */

    public void moveTo(Mode mode, boolean back, boolean ground, double liftPosition, Position position) {
        if (!back) {
            elbowParallel = Constants.kElbowParallelForward;
            elbowMaxAngle = Constants.kElbowMaxAngleForward;
            wristMaxAngle = Constants.kWristMaxAngleForward;
        } else {
            elbowParallel = Constants.kElbowParallelReverse;
            elbowMaxAngle = Constants.kElbowMaxAngleReverse;
            wristMaxAngle = Constants.kWristMaxAngleReverse;
        }
        switch (position) {
        case CLIMB:
        case BOTTOM:
            retract();
            elbow.moveTo(Constants.kElbowVertical);
            wrist.moveTo(Constants.kWristParallel);
            break;
        case PICKUP:
            extend();
            if (mode == Mode.HATCH_PANEL) {
                if (ground) {
                wrist.moveTo(Constants.kWristHatchPanelPickup);
                } else {
                wrist.moveTo(Constants.kWristParallel);
                }
            } else {
                back = false;
                wrist.moveTo(Constants.kWristParallel);
            }
            elbow.moveTo(elbowParallel);
            break;
        case LEVEL_1:
            if (liftPosition > Constants.kLiftLevel1HatchPanel / 2) {
                retract();
            }
            elbow.moveTo(elbowParallel);
            wrist.moveTo(wristParallel);
            break;
        case LEVEL_2:
        case LEVEL_3:
            extend();
            elbow.moveTo(elbowMaxAngle);
            wrist.moveTo(wristMaxAngle);
            break;
        }
    }
}