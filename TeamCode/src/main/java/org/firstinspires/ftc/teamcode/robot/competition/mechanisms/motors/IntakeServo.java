package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeServo {
    //instance variables

    public Servo intakeServoL; //  the motor
    public Servo intakeServoR;


    public final DcMotor.RunMode currentRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
    public double extendPosition = 1;    // help confused
    public int retractPosition = 0;   // help confused

    public LinearOpMode intakeLinearOp = null;
    public LinearOpMode linearOp = null;

    public final double TICKS_PER_ROTATION = 538;

    double maxIntakeSpinnerTime = 2; //max time for arm to run, in SECONDS. (for lowering robot)
    double getMaxIntakeSpinnerTimeEncoder = 3;
    double maxSpinnerRetractTime = 2; //max time for arm to run, in SECONDS. (for lowering robot)
    int RotatorTargetPositionForawrd = -2000;
    int RotatorTargetPositionReverse = 2000;

    public int sleepTime = 5;


    public ElapsedTime IntakeServoRunTime;




    // constructors
    public IntakeServo (Servo inServoL, Servo inServoR) {
        inServoL = intakeServoL;
        inServoR = intakeServoR;

        intakeServoL.setDirection(Servo.Direction.FORWARD);
        intakeServoR.setDirection(Servo.Direction.REVERSE);

    }


    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }

    // methods
    public void intakelinearOp (LinearOpMode Op) {
        intakeLinearOp = Op;
    }

    public void stopIntakeServo () {
        intakeServoL.setPosition(.5);
        intakeServoR.setPosition(.5);
    }

    public void setIntakeMotorRunModes (DcMotor.RunMode mode) {
        //intakeMotor.setMode(mode);
    }


    // autonomous methods

    public void IntakeServoReverse () {
        intakeServoL.setPosition(0);
        intakeServoR.setPosition(0);
    }

    public void IntakeServoForward () {
        intakeServoL.setPosition(1);
        intakeServoR.setPosition(1);
    }

    public void IntakeServoForwardTime () {
        intakeServoL.setPosition(1);
        intakeServoR.setPosition(1);
        linearOp.sleep(sleepTime);
        linearOp.idle();
        stopIntakeServo();
    }

    public void IntakeServoReverseTime () {
        intakeServoR.setPosition(0);
        intakeServoL.setPosition(0);
        linearOp.sleep(sleepTime);
        linearOp.idle();
        stopIntakeServo();
    }

}
