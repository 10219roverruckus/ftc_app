package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class LiftMotor {
    //instance variables

    public DcMotor liftMotor; //  the arm


    public final DcMotor.RunMode currentRunMode = DcMotor.RunMode.RUN_USING_ENCODER;


    public LinearOpMode liftMotorLinearOp = null;

    public final double TICKS_PER_ROTATION = 538;



    //NEED:
    //topHeight
    //lowHeight

    // constructors
    public LiftMotor (DcMotor liftM) {
        liftMotor = liftM;

        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setLiftMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setLiftMotorRunModes(currentRunMode);
    }



    // methods
    public void liftMotorlinearOp (LinearOpMode Op) {
        liftMotorLinearOp = Op;
    }
    public void stopIntakeMotors () {
        liftMotor.setPower(0);
    }

    public void setLiftMotorRunModes (DcMotor.RunMode mode) {
        liftMotor.setMode(mode);
    }



    public void extendingLiftMotor(double speed, double rotations) {
        liftMotor.setMode(currentRunMode);
    }

    public void retractingLiftMotor(double speed, double rotations) {
        liftMotor.setMode(currentRunMode);

    }


    //function that fully extends arm using distance sensor
    public void extendLiftMotorFully () {
        //set motor to full power WHILE the distance is less than topHeight
        //be sure to stop motor at end!

    }

    //function that fully retracts arm using distance sensor.
    public void retractLiftMotorFully() {
        //set motor to full power WHILE the distance sensor is less than lowHeight
        //be sure to stop motor at end!
    }
}
