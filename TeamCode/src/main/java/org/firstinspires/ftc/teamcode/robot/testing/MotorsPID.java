package org.firstinspires.ftc.teamcode.robot.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MotorsPID {
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public final DcMotor.RunMode currentMotorRunMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
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

        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMotorRunModes(currentMotorRunMode);
        //counts = counts * cpr; // added counts in here or should it be ticks
    }

    public void drivePID (double power, double distance) {
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearOp.idle();
        targetDistance = distance * TICKS_PER_ROTATION;
        while (targetDistance > leftMotor.getCurrentPosition() && linearOp.opModeIsActive()) {
            if (leftMotor.getCurrentPosition() <= targetDistance * .2) {
                leftMotor.setPower(MINPOWER);
                rightMotor.setPower(MINPOWER);
            }
            else if (leftMotor.getCurrentPosition() > targetDistance * .2 && leftMotor.getCurrentPosition() < .8) {  // (target distance - current distance) / total distance
                leftMotor.setPower(power);
                rightMotor.setPower(power);
            }
            else if (leftMotor.getCurrentPosition() > .8) {  //may need to be just an ELSE if we have a third condition about to check for MINPOWER threshold.
                leftMotor.setPower(MINPOWER);
                rightMotor.setPower(MINPOWER);
            }
        }
        stopMotors();
        linearOp.idle();
    }

    public void stopMotors () {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}



