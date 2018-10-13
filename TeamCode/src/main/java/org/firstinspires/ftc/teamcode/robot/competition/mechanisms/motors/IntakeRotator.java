package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeRotator {
    //instance variables

    public DcMotor intakeRotator; //  the arm


    public final DcMotor.RunMode currentRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
    public double extendPosition = 1;    // help confused
    public int retractPosition = 0;   // help confused

    public LinearOpMode IntakeRotatorLinearOp = null;

    public final double TICKS_PER_ROTATION = 538;



    // constructors
    public IntakeRotator (DcMotor inRot) {
        intakeRotator = inRot;

        intakeRotator.setDirection(DcMotor.Direction.FORWARD);
        setIntakeRotatorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setIntakeRotatorRunModes(currentRunMode);
    }



    // methods
    public void IntakeRotatorlinearOp (LinearOpMode Op) {
       IntakeRotatorLinearOp = Op;
    }
    public void stopIntakeRotatorMotors () {
        intakeRotator.setPower(0);
    }

    public void setIntakeRotatorRunModes (DcMotor.RunMode mode) {
        intakeRotator.setMode(mode);
    }



    public void extendingIntakeRotator(double speed, double rotations) {
        intakeRotator.setMode(currentRunMode);
    }

    public void retractingIntakeRotator(double speed, double rotations) {
        intakeRotator.setMode(currentRunMode);
    }
    }


