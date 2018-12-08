package org.firstinspires.ftc.teamcode.robot.outreach;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class OutreachMotors {
    public DcMotor leftMotor;
    public DcMotor rightMotor;

    public LinearOpMode linearOp = null;


    public final double MINPOWER = .3;
    public final double MAXPOWER = 1;

    public double targetDistance;
    public static final double TICKS_PER_ROTATION = 538; // TICKS (COUNTS) PER ROTATION NEEDED!!!!!!!!! :)


    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }



    public OutreachMotors (DcMotor lm, DcMotor rm) {
        leftMotor = lm;
        rightMotor = rm;

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        //leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive (double leftMotorPower, double rightMotorPower) {
//        leftMotor.setDirection(DcMotor.Direction.REVERSE);
//        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        leftMotorPower = Range.clip(leftMotorPower,-1,+1);
        rightMotorPower = Range.clip(rightMotorPower, -1 ,1);
        if (leftMotorPower < -.1 || leftMotorPower > .1) {
            leftMotor.setPower(leftMotorPower);
        }
        else {
            leftMotor.setPower(0);
        }
        if (rightMotorPower < -.1 || rightMotorPower > .1) {
            rightMotor.setPower(rightMotorPower);
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

        while (targetDistance > rightMotor.getCurrentPosition() && linearOp.opModeIsActive()  && !linearOp.isStopRequested()) {
            linearOp.telemetry.addData("START WHILE", rightMotor.getCurrentPosition());
            linearOp.telemetry.update();
            if (rightMotor.getCurrentPosition() <= (targetDistance * .2) ) {
//                linearOp.telemetry.addData("FIRST IF", rightMotor.getCurrentPosition());
//                linearOp.telemetry.update();
                leftMotor.setPower(MINPOWER);
                rightMotor.setPower(MINPOWER);
                //linearOp.sleep(1000);
            }
            else if (rightMotor.getCurrentPosition() > (targetDistance * .2) && rightMotor.getCurrentPosition() < (targetDistance * .6) ) {  // (target distance - current distance) / total distance
//                linearOp.telemetry.addData("FIRST ELSE IF", rightMotor.getCurrentPosition());
//                linearOp.telemetry.update();
                leftMotor.setPower(power);
                rightMotor.setPower(power);
            }
            else if (rightMotor.getCurrentPosition() >= (targetDistance * .6) && rightMotor.getCurrentPosition() <= (targetDistance * 1) )  {  //may need to be just an ELSE if we have a third condition about to check for MINPOWER threshold.
//                linearOp.telemetry.addData("SECOND ELSE IF", rightMotor.getCurrentPosition());
//                linearOp.telemetry.update();
                leftMotor.setPower(MINPOWER);
                rightMotor.setPower(MINPOWER);
                linearOp.sleep(1000);
            }
            else {
//                linearOp.telemetry.addLine("Else STOP");
//                linearOp.telemetry.update();
                stopMotors();
            }
            //linearOp.telemetry.addData("END WHILE - position: ", rightMotor.getCurrentPosition());
          //  linearOp.telemetry.update();
//            linearOp.sleep(2000);
            linearOp.idle();
        }
        stopMotors();
        //stopMotors();
        linearOp.idle();
    }

}
