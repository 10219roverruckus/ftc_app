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
    double maxArmExtendTime = 2; //max time for arm to run, in SECONDS. (for lowering robot)
    double maxArmRetractTime = 2; //max time for arm to run, in SECONDS. (for lowering robot)


    // ##########################  NO CHANGES ABOVE THIS LINE #######################

    // these are the two variables to change if the lift is not working in auto

    double getMaxArmExtendTimeEncoder = 3.4;
    int liftTargetPosition = -5600;


    // ##########################  NO CHANGES BELOW THIS LINE #######################

    public final DcMotor.RunMode currentRunMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;


    public LinearOpMode liftMotorLinearOp = null;
    public LinearOpMode linearOp = null;

    public final double TICKS_PER_ROTATION = 538;

    public ElapsedTime armRunTime;


    // constructors
    public LiftMotor (DcMotor liftM) {
        liftMotor = liftM;

        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setLiftMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setLiftMotorRunModes(currentRunMode);

        armRunTime = new ElapsedTime();
        armRunTime.reset();
    }
//seting motors

    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }


    // methods
    public void stopMotors () {
        liftMotor.setPower(0);
    }

    public void setLiftMotorRunModes (DcMotor.RunMode mode) {
        liftMotor.setMode(mode);
    }



    public void extendLift() {
        liftMotor.setPower(1);
    }

    public void retractLift() {
        liftMotor.setPower(-1);
    }


    //function that fully extends arm using distance sensor
    public void extendLiftMotorFully () { //distance sensor
        armRunTime.reset();
        while (armRunTime.time() <= maxArmExtendTime && linearOp.opModeIsActive()) {
           liftMotor.setPower(-1);
        }
        liftMotor.setPower(0);
    }

    //function that fully retracts arm using distance sensor.
    public void retractLiftMotorFully() {               //DistanceSensor distanceSensor
        //set motor to full power WHILE the distance sensor is less than lowHeight
        //be sure to stop motor at end!
        armRunTime.reset();
        while (armRunTime.time() <= maxArmRetractTime && linearOp.opModeIsActive()) {
            liftMotor.setPower(1);
        }
        liftMotor.setPower(0);
    }

    public void extendLiftMotorFullyEncoders () {
        armRunTime.reset();
        while (liftMotor.getCurrentPosition() > liftTargetPosition && linearOp.opModeIsActive()) {
            linearOp.telemetry.addData("ENCODER", liftMotor.getCurrentPosition());
            linearOp.telemetry.update();
            liftMotor.setPower(-.75);
            if (armRunTime.time() >= getMaxArmExtendTimeEncoder) {
                linearOp.telemetry.addData("ENCODER", liftMotor.getCurrentPosition());
                linearOp.telemetry.addLine("BREAK");
                linearOp.telemetry.update();
                break;
            }
            linearOp.sleep(300);
            linearOp.idle();
        }
        liftMotor.setPower(0);
    }
}
