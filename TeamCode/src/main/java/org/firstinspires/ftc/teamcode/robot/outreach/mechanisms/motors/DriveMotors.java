package org.firstinspires.ftc.teamcode.robot.outreach.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class DriveMotors {
    public DcMotor leftMotor;
    public DcMotor rightMotor;



    public double MINPOWER = .3;
    public final double MAXPOWER = 1;

    public double targetDistance;
    public static final double TICKS_PER_ROTATION = 538; // TICKS (COUNTS) PER ROTATION NEEDED!!!!!!!! :)

    public LinearOpMode linearOp = null;

    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }

    public DriveMotors (DcMotor lm, DcMotor rm) {
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

    public void drivePID (double power, double distance) {
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
        if (power < 0) {
            MINPOWER = -MINPOWER;
        }


        while (targetDistance > Math.abs(rightMotor.getCurrentPosition()) && linearOp.opModeIsActive()  && !linearOp.isStopRequested()) {
            linearOp.telemetry.addData("START WHILE", rightMotor.getCurrentPosition());
            linearOp.telemetry.update();
            if (Math.abs(rightMotor.getCurrentPosition()) <= (targetDistance * .2) ) {
//                linearOp.telemetry.addData("FIRST IF", rightMotor.getCurrentPosition());
//                linearOp.telemetry.update();
                leftMotor.setPower(MINPOWER);
                rightMotor.setPower(MINPOWER);
                //linearOp.sleep(1000);
            }
            else if (Math.abs(rightMotor.getCurrentPosition()) > (targetDistance * .2) && Math.abs(rightMotor.getCurrentPosition()) < (targetDistance * .6)) {  // (target distance - current distance) / total distance
//                linearOp.telemetry.addData("FIRST ELSE IF", rightMotor.getCurrentPosition());
//                linearOp.telemetry.update();
                leftMotor.setPower(power);
                rightMotor.setPower(power);
            }
            else if (Math.abs(rightMotor.getCurrentPosition()) >= (targetDistance * .6) && Math.abs(rightMotor.getCurrentPosition()) <= (targetDistance * 1) )  {  //may need to be just an ELSE if we have a third condition about to check for MINPOWER threshold.
//                linearOp.telemetry.addData("SECOND ELSE IF", rightMotor.getCurrentPosition());
//                linearOp.telemetry.update();
                leftMotor.setPower(MINPOWER);
                rightMotor.setPower(MINPOWER);
               // linearOp.sleep(1000);
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
        // RECALIBARATE POWERS.
        if (power < 0) {
            power = -power;
        }
        if (MINPOWER < 0) {
            MINPOWER = -MINPOWER;
        }
        linearOp.idle();
    }

    public void stopMotors () {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
