package frc.robot.mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Mode;
import frc.robot.constants.Pneumatics;

public class Intake extends VictorSPX {

    private Mode mode;
    private DoubleSolenoid extender;

    public Intake(int i) {
        super(i);
        extender = new DoubleSolenoid(Pneumatics.kIntakeForward, Pneumatics.kIntakeReverse);
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public Mode getMode() {
        return this.mode;
    }

    public void qeury(Mode _mode, double speed) {
        setMode(_mode);
        
        if (mode == Mode.HATCH_PANEL) {
            extender.set(Value.kForward);
        } else {
        }
    }
}