package org.firstinspires.ftc.teamcode.robot.outreach;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class OutreachMotors {
    public DcMotor leftMotor;
    public DcMotor rightMotor;

    public DcMotor frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;

    public double leftMotorValue, rightMotorValue;

    public double leftSquaredValue, rightSquaredValue;


    public LinearOpMode linearOp = null;


    public final double MINPOWER = .3;
    public final double MAXPOWER = 1;

    public double targetDistance;
    public static final double TICKS_PER_ROTATION = 538; // TICKS (COUNTS) PER ROTATION NEEDED!!!!!!!!! :)


    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }


    //2 drive motors
    public OutreachMotors (DcMotor lm, DcMotor rm) {
        leftMotor = lm;
        rightMotor = rm;

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    //4 drive motors
    public OutreachMotors (DcMotor fLM, DcMotor fRM, DcMotor rLM, DcMotor rRM) {
        frontLeftMotor = fLM;
        frontRightMotor = fRM;
        rearLeftMotor = rLM;
        rearRightMotor = rRM;


        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void driveTank (Gamepad gamepad, DriveDirection driveDirection) {
        driveTank(gamepad.left_stick_y, gamepad.right_stick_y, driveDirection);
    }

    public void driveTank (double leftPower, double rightPower, DriveDirection driveDirection) {
        leftPower = Range.clip(leftPower,-1,+1);
        rightPower = Range.clip(rightPower, -1 ,1);
        switch (driveDirection) {
            case FORWARD:
                if (leftPower < -.01 || leftPower > .01) {
                    frontLeftMotor.setPower(leftPower);
                    rearLeftMotor.setPower(leftPower);
                }
                else {
                    frontLeftMotor.setPower(0);
                    rearLeftMotor.setPower(0);
                }
                if (rightPower < -.01 || rightPower > .01) {
                    frontRightMotor.setPower(rightPower);
                    rearRightMotor.setPower(rightPower);
                }
                else {
                    frontRightMotor.setPower(0);
                    rearRightMotor.setPower(0);
                }
                break;
            case REVERSE:
                if (leftPower < -.01 || leftPower > .01) {
                    frontLeftMotor.setPower(-leftPower);
                    rearLeftMotor.setPower(-leftPower);
                }
                else {
                    leftMotor.setPower(0);
                }
                if (rightPower < -.01 || rightPower > .01) {
                    frontRightMotor.setPower(-rightPower);
                    rearRightMotor.setPower(-rightPower);
                }
                else {
                    rightMotor.setPower(0);
                }
                break;
        }

    }

    //Not used at the moment.
    public void driveTank (double leftValue, double rightValue, boolean squareInputs, DriveDirection driveDirection) {
        if (squareInputs) {
            leftSquaredValue = leftValue * leftValue;
            rightSquaredValue = rightValue * rightValue;
            if (leftValue < 0) {
                leftSquaredValue = -leftSquaredValue;
            }
            if (rightValue < 0) {
                rightSquaredValue = - rightSquaredValue;
            }
        }
        driveTank(leftSquaredValue, rightSquaredValue, driveDirection);
    }



    public void arcadeDrive (Gamepad gamepad, DriveDirection driveDirection) {
        arcadeDrive(-gamepad.left_stick_y, gamepad.left_stick_x, driveDirection);
    }

    public void arcadeDrive (double forwardSpeed, double turnRate, DriveDirection driveDirection) {
        leftMotorValue = forwardSpeed + turnRate;
        rightMotorValue = forwardSpeed - turnRate;
        //clip left & right motor values to stay within [-1, +1]
        leftMotorValue = Range.clip(leftMotorValue, -1, 1);
        rightMotorValue = Range.clip(rightMotorValue, -1, 1);
        switch (driveDirection) {
            case FORWARD:
                frontLeftMotor.setPower(-leftMotorValue);
                rearLeftMotor.setPower(-leftMotorValue);
                frontRightMotor.setPower(-rightMotorValue);
                rearRightMotor.setPower(-rightMotorValue);
                break;
            case REVERSE:
//                leftMotor.setPower(-leftMotorValue);
//                rightMotor.setPower(-rightMotorValue);
                break;
        }

//        if (driveDirection == DriveDirection.FORWARD) {
//            leftMotor.setPower(leftMotorValue);
//            rightMotor.setPower(rightMotorValue);
//        }
//        if (driveDirection == DriveDirection.REVERSE) {
//            leftMotor.setPower(-leftMotorValue);
//            rightMotor.setPower(-rightMotorValue);
//        }
    }

    public void stopMotors () {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        rearLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearRightMotor.setPower(0);
    }

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


