package org.firstinspires.ftc.teamcode.robot.outreach;

import com.qualcomm.robotcore.hardware.DcMotor;

public class outreachCatapultMotorsRunWithoutEncoders2motors {
    private DcMotor catapultMotor1;
    private DcMotor catapultMotor2;
    private double resetPower;
    private double launchPower;
    private int armTravelDistance;

    public outreachCatapultMotorsRunWithoutEncoders2motors (DcMotor cM1, DcMotor cM2) {
        catapultMotor1 = cM1;
        catapultMotor2 = cM2;

        catapultMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catapultMotor1.setDirection(DcMotor.Direction.FORWARD);
        catapultMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catapultMotor2.setDirection(DcMotor.Direction.REVERSE);
        catapultMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapultMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        resetPower = .1;
        launchPower = 1;

        armTravelDistance = 300;
    }


    public void catapultLaunch (outreachTouchSensorCatapult touchSensor) {
        //catapultReset(touchSensor);
        catapultMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        catapultMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapultMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catapultMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        catapultMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapultMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (catapultMotor1.getCurrentPosition() <= 230) {
            catapultMotor1.setPower(launchPower);
            catapultMotor2.setPower(launchPower);
        }
        catapultMotor1.setPower(0);
        catapultMotor2.setPower(0);
    }

    public void catapultReset (outreachTouchSensorCatapult touchSensor) {
        while (touchSensor.checkCatapultTouchSensor() == true) {
            catapultMotor1.setPower(-resetPower);
            catapultMotor2.setPower(-resetPower);
        }
        catapultMotor1.setPower(0);
        catapultMotor2.setPower(0);
    }

    public void catapultMotorManualOperation (double armLower, double armRaise, outreachTouchSensorCatapult touchSensor) {
        catapultMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapultMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (armLower > 0 && touchSensor.checkCatapultTouchSensor() == true) {
            catapultMotor1.setPower(-armLower);
            catapultMotor2.setPower(-armLower);
        }
        else if (armRaise > 0) {
            catapultMotor1.setPower(armRaise);
            catapultMotor2.setPower(armRaise);

        }
        else {
            catapultMotor1.setPower(0);
            catapultMotor2.setPower(0);
        }
    }

    public void catapultStopMotors () {
        catapultMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapultMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapultMotor1.setPower(0);
        catapultMotor2.setPower(0);
    }
}
