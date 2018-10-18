package org.firstinspires.ftc.teamcode.robot.outreach.mechanisms.motors;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.outreach.outreachTouchSensorCatapult;

public class CatapultArm {
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public final DcMotor.RunMode currentMotorRunMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public final DcMotor.ZeroPowerBehavior currentMotorBrakeMode = DcMotor.ZeroPowerBehavior.BRAKE;
    public static final int TICKS_PER_ROTATION = 1680; // TICKS PER ROTATION NEEDED
    public static final int LAUNCH_DISTANCE = 190; //distance arms will travel.
    public static final double LAUNCH_POWER = 1;
    public static final double RESET_POWER = -.2;

    public CatapultArm (DcMotor LM, DcMotor RM) {
        leftMotor = LM;
        rightMotor = RM;

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        setMotorRunModes(currentMotorRunMode);
        setCurrentMotorBrakeMode(currentMotorBrakeMode);
    }

    public void setMotorRunModes (DcMotor.RunMode mode) {
        leftMotor.setMode(mode);
        rightMotor.setMode(mode);
    }

    public void setCurrentMotorBrakeMode (DcMotor.ZeroPowerBehavior zero) {
        leftMotor.setZeroPowerBehavior(zero);
        rightMotor.setZeroPowerBehavior(zero);
    }

    public void catapultMotorManualOperation (double armLower, double armRaise, outreachTouchSensorCatapult touchSensor) {
        if (armLower > 0 && touchSensor.checkCatapultTouchSensor() == true) {
            leftMotor.setPower(-armLower);
            rightMotor.setPower(-armLower);
        }
        else if (armRaise > 0) {
            leftMotor.setPower(armRaise);
            rightMotor.setPower(armRaise);

        }
        else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }

    public void catapultLaunch (outreachTouchSensorCatapult touchSensor) {
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(currentMotorRunMode);
        while (leftMotor.getCurrentPosition() <= LAUNCH_DISTANCE) {
            leftMotor.setPower(LAUNCH_POWER);
            rightMotor.setPower(LAUNCH_POWER);
        }
    }

    public void catapultReset (outreachTouchSensorCatapult touchSensor) {
        while (touchSensor.checkCatapultTouchSensor() == true) {
            leftMotor.setPower(RESET_POWER);
            rightMotor.setPower(RESET_POWER);
        }
    }

    public void motorPowerZero () {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }


}
