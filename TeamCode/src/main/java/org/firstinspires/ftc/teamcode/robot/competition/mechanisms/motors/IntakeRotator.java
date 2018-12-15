package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeRotator {
    //instance variables

    public DcMotor intakeRotator; //  the arm


    public final DcMotor.RunMode currentRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
    public double extendPosition = 1;    // help confused
    public int retractPosition = 0;   // help confused

    public LinearOpMode IntakeRotatorLinearOp = null;
    public LinearOpMode linearOp = null;


    public final double TICKS_PER_ROTATION = 538;


    double maxRotatorExtendTime = 2; //max time for arm to run, in SECONDS. (for lowering robot)
    double getMaxRotatorExtendTimeEncoder = 3;
    double maxArmRetractTime = 2; //max time for arm to run, in SECONDS. (for lowering robot)
    int RotatorTargetPosition = -2000;

    public ElapsedTime armRunTime;


    // constructors
    public IntakeRotator (DcMotor inRot) {
        intakeRotator = inRot;

        intakeRotator.setDirection(DcMotor.Direction.FORWARD);
        setIntakeRotatorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setIntakeRotatorRunModes(currentRunMode);
    }

    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
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


    // autonomous methods
    // methods are for knocking off the minerals

    public void mineralRotateLowerEncoder () {
        armRunTime.reset();
        while (intakeRotator.getCurrentPosition() > RotatorTargetPosition) {
            linearOp.telemetry.addData("ENCODER", intakeRotator.getCurrentPosition());
            linearOp.telemetry.update();
            intakeRotator.setPower(-.75);
            if (armRunTime.time() >= getMaxRotatorExtendTimeEncoder) {
                linearOp.telemetry.addLine("BREAK");
                linearOp.telemetry.update();
                break;
            }
            linearOp.idle();
        }
        intakeRotator.setPower(0);
    }



    public void mineralRotateRaiseEncoder () {
        armRunTime.reset();
        while (armRunTime.time() <= maxArmRetractTime) {
            intakeRotator.setPower(1);
        }
        intakeRotator.setPower(0);
    }


    }


