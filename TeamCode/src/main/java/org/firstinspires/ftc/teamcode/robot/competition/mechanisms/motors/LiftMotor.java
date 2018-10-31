package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LiftMotor {
    //instance variables

    public DcMotor liftMotor; //  the arm
    double topHeight = 8.4;
    double lowHeight = 3.2;
    double maxArmExtendTime = 8; //max time for arm to run, in SECONDS. (for lowering robot)
    double maxArmRetractTime = 8; //max time for arm to run, in SECONDS. (for lowering robot)


    public final DcMotor.RunMode currentRunMode = DcMotor.RunMode.RUN_USING_ENCODER;


    public LinearOpMode liftMotorLinearOp = null;

    public final double TICKS_PER_ROTATION = 538;

    public ElapsedTime armRunTime;


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

        armRunTime = new ElapsedTime();
        armRunTime.reset();
    }
//seting motors


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
    public void extendLiftMotorFully () { //distance sensor
        armRunTime.reset();
        while (armRunTime.time() <= maxArmExtendTime) {
           liftMotor.setPower(-1);
        }
        liftMotor.setPower(0);
    }

    //function that fully retracts arm using distance sensor.
    public void retractLiftMotorFully() {               //DistanceSensor distanceSensor
        //set motor to full power WHILE the distance sensor is less than lowHeight
        //be sure to stop motor at end!
        armRunTime.reset();
        while (armRunTime.time() <= maxArmRetractTime) {
            liftMotor.setPower(1);
        }
        liftMotor.setPower(0);
    }
}
