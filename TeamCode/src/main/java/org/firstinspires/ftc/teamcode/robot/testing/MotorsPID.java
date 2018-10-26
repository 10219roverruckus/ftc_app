package org.firstinspires.ftc.teamcode.robot.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MotorsPID {
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    //public final DcMotor.RunMode currentMotorRunMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public static final double TICKS_PER_ROTATION = 538; // TICKS (COUNTS) PER ROTATION NEEDED!!!!!!!! :)
    // http://www.andymark.com/NeveRest-20-12V-Planetary-Gearmotor-p/am-3637.htm

    //public int cpr = 538;
    public double targetDistance;

    public LinearOpMode linearOp = null;

    public final double MINPOWER = .2;
    public final double MAXPOWER = 1;



    // To stop loop when stop button pressed.
    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }

    public void setMotorRunModes (DcMotor.RunMode mode) {

        leftMotor.setMode(mode);
        rightMotor.setMode(mode);
    }


    public MotorsPID (DcMotor lm, DcMotor rm) {
        leftMotor = lm;      // LM is left Motor
        rightMotor = rm;     // RM is Right Motor

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);


        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //setMotorRunModes(currentMotorRunMode);
    }


    public void drivePID (double power, double distance) {
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearOp.idle();
        targetDistance = distance * TICKS_PER_ROTATION;
        linearOp.telemetry.addData("CURRENT POSITION", leftMotor.getCurrentPosition());
        linearOp.telemetry.addData("TARGET POSITION", targetDistance);
        linearOp.telemetry.update();
        leftMotor.setPower(.5);
        rightMotor.setPower(.5);
        linearOp.sleep(1000);
        stopMotors();
    }

    /*

    public void drivePID (double power, double distance) {
        //setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearOp.idle();
        targetDistance = distance * TICKS_PER_ROTATION;
        linearOp.telemetry.addData("CURRENT POSITION", leftMotor.getCurrentPosition());
        linearOp.telemetry.addData("TARGET POSITION", targetDistance);
        linearOp.telemetry.update();
        leftMotor.setPower(.5);
        rightMotor.setPower(.5);
        linearOp.sleep(1000);
        stopMotors();
        while (targetDistance > leftMotor.getCurrentPosition() && linearOp.opModeIsActive()) {
            linearOp.telemetry.addData("START WHILE", leftMotor.getCurrentPosition());
            linearOp.telemetry.update();
            if (leftMotor.getCurrentPosition() <= targetDistance * .2) {
                linearOp.telemetry.addData("FIRST IF", leftMotor.getCurrentPosition());
                linearOp.telemetry.update();
                leftMotor.setPower(1);
                rightMotor.setPower(1);
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
            linearOp.telemetry.addData("END WHILE", leftMotor.getCurrentPosition());
            linearOp.telemetry.update();
        }
        stopMotors();
        linearOp.idle();
    }

    */

    public void stopMotors () {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}



