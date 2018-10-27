package org.firstinspires.ftc.teamcode.robot.outreach;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class outreachMotors {
    public DcMotor leftMotor;
    public DcMotor rightMotor;

    public LinearOpMode linearOp = null;


    public final double MINPOWER = .3;
    public final double MAXPOWER = 1;

    public double targetDistance;
    public static final double TICKS_PER_ROTATION = 538; // TICKS (COUNTS) PER ROTATION NEEDED!!!!!!!! :)


    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }



    public outreachMotors (DcMotor lm, DcMotor rm) {
        leftMotor = lm;
        rightMotor = rm;

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        //leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive (double leftY, double rightY) {
//        leftMotor.setDirection(DcMotor.Direction.REVERSE);
//        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        leftY = Range.clip(leftY,-1,+1);
        rightY = Range.clip(rightY, -1 ,1);
        if (leftY < -.1 || leftY > .1) {
            leftMotor.setPower(leftY);
        }
        else {
            leftMotor.setPower(0);
        }
        if (rightY < -.1 || rightY > .1) {
            rightMotor.setPower(rightY);
        }
        else {
            rightMotor.setPower(0);
        }
    }

    public void stopMotors () {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

/*
    public void drivePID (double power, double distance) {
////        leftMotor.setDirection(DcMotor.Direction.REVERSE);
////        rightMotor.setDirection(DcMotor.Direction.FORWARD);
////

////
//        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //linearOp.idle();
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetDistance = distance * TICKS_PER_ROTATION;
        linearOp.telemetry.addData("CURRENT POSITION BEFORE", rightMotor.getCurrentPosition());
        linearOp.telemetry.addData("TARGET POSITION", targetDistance);
        linearOp.telemetry.update();
        leftMotor.setPower(.5);
        rightMotor.setPower(.5);
        linearOp.sleep(2000);
        stopMotors();
        linearOp.telemetry.addData("CURRENT POSITION AFTER", rightMotor.getCurrentPosition());
        linearOp.telemetry.addData("TARGET POSITION", targetDistance);
        linearOp.telemetry.update();
        linearOp.sleep (2000);
    }

*/


    public void drivePID (double power, double distance) {
        //setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearOp.idle();
        targetDistance = distance * TICKS_PER_ROTATION;
//        linearOp.telemetry.addData("CURRENT POSITION", leftMotor.getCurrentPosition());
//        linearOp.telemetry.addData("TARGET POSITION", targetDistance);
//        linearOp.telemetry.update();
//        leftMotor.setPower(.5);
//        rightMotor.setPower(.5);
//        linearOp.sleep(1000);
//        stopMotors();
//        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (targetDistance > leftMotor.getCurrentPosition() && linearOp.opModeIsActive()) {
            linearOp.telemetry.addData("START WHILE", leftMotor.getCurrentPosition());
            linearOp.telemetry.update();
            if (leftMotor.getCurrentPosition() <= targetDistance * .2) {
                linearOp.telemetry.addData("FIRST IF", leftMotor.getCurrentPosition());
                linearOp.telemetry.update();
                leftMotor.setPower(MINPOWER);
                rightMotor.setPower(MINPOWER);
                //linearOp.sleep(1000);
            }
            else if (leftMotor.getCurrentPosition() > targetDistance * .2 && leftMotor.getCurrentPosition() < targetDistance * .8) {  // (target distance - current distance) / total distance
                linearOp.telemetry.addData("FIRST ELSE IF", leftMotor.getCurrentPosition());
                linearOp.telemetry.update();
                leftMotor.setPower(power);
                rightMotor.setPower(power);
            }
            else if (leftMotor.getCurrentPosition() > targetDistance * .8) {  //may need to be just an ELSE if we have a third condition about to check for MINPOWER threshold.
                linearOp.telemetry.addData("SECOND ELSE IF", leftMotor.getCurrentPosition());
                linearOp.telemetry.update();
                leftMotor.setPower(MINPOWER);
                rightMotor.setPower(MINPOWER);
            }
            else {
                linearOp.telemetry.addLine("Else STOP");
                linearOp.telemetry.update();
                stopMotors();
            }
            linearOp.telemetry.addData("END WHILE", leftMotor.getCurrentPosition());
            linearOp.telemetry.update();
            linearOp.sleep(2000);
        }
        //stopMotors();
        linearOp.idle();
    }

}
