package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Mode;
import frc.robot.Position;
import frc.robot.constants.Constants;
import frc.robot.constants.Pneumatics;
import frc.robot.mechanisms.Elbow;

public class Arm {

    private Elbow elbow;
    private DoubleSolenoid extender;
    private double elbowParallel = Constants.kElbowParallelForward;
    private double elbowMaxAngle = Constants.kElbowMaxAngleForward;
    private double elbowVertical = Constants.kElbowVertical;
    private double wristParallel = Constants.kWristParallel;
    private boolean extended = false;
    private double wristMaxAngle = Constants.kWristParallel;

    public Arm(Elbow elbow) {
        this.elbow = elbow;
        extender = new DoubleSolenoid(1, Pneumatics.kArmForward, Pneumatics.kArmReverse);
    }


    public void display() {
        SmartDashboard.putNumber("Elbow Voltage", elbow.getMotorOutputVoltage());
    }

    public void extend() {
        extender.set(DoubleSolenoid.Value.kForward);
        extended = true;
    }

    public void retract() {
        extender.set(DoubleSolenoid.Value.kReverse);
        extended = false;
    }

    /*
     * if !cargo then hatchpanel
     */

    public void moveTo(Mode mode, boolean back, Position position) {
        if (back == false) {
            System.out.println("she not in");
            elbowParallel = Constants.kElbowParallelForward;
            elbowMaxAngle = Constants.kElbowMaxAngleForward;
            wristMaxAngle = Constants.kWristMaxAngleForward;
        } else {
            System.out.println("she in");
            elbowParallel = Constants.kElbowParallelReverse;
            elbowMaxAngle = Constants.kElbowMaxAngleReverse;
            wristMaxAngle = Constants.kWristMaxAngleReverse;
        }
        switch (position) {
        case CLIMB:
        case BOTTOM:
            retract();
            elbow.moveTo(Constants.kElbowVertical);
           // elbow.moveTo(elbowParallel);
            break;
        case PICKUP:
            if (mode == Mode.HATCH_PANEL) {
                retract();
                elbow.moveTo(elbowParallel);
            } else {
                back = true;
                extend();
                elbow.moveTo(Constants.kElbowCargoPickup);
            }
            break;
        case LEVEL_1:
        case LEVEL_2:
        case LEVEL_3:
            retract();
            elbow.moveTo(elbowParallel);
            break;
        }
    }
}