package org.firstinspires.ftc.teamcode.robot.outreach.mechanisms.motors;

import com.qualcomm.robotcore.hardware.DcMotor;

public class FourBarLink {
    public DcMotor FourBarLinkMotor;
    public final DcMotor.RunMode currentMotorRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
    public final DcMotor.ZeroPowerBehavior currentMotorBrakeMode = DcMotor.ZeroPowerBehavior.BRAKE;
    public static final int TICKS_PER_ROTATION = 1680; // TICKS PER ROTATION NEEDED
    public static final int LAUNCH_DISTANCE = 190; //distance arms will travel.
    public static final double LAUNCH_POWER = 1;
    public static final double RESET_POWER = -.2;

    public FourBarLink (DcMotor fblm) {
        FourBarLinkMotor = fblm;
        FourBarLinkMotor.setDirection(DcMotor.Direction.FORWARD);
//        setMotorRunModes(currentMotorRunMode);
        setCurrentMotorBrakeMode(currentMotorBrakeMode);
    }

    public void setMotorRunModes (DcMotor.RunMode mode) {
        FourBarLinkMotor.setMode(mode);
    }

    public void setCurrentMotorBrakeMode (DcMotor.ZeroPowerBehavior zero) {
        FourBarLinkMotor.setZeroPowerBehavior(zero);
    }

    public void FourBarManualControl (double fourBarPower) {
        FourBarLinkMotor.setPower(fourBarPower);
    }


    public void stopMotor () {
        FourBarLinkMotor.setPower(0);
    }


}
