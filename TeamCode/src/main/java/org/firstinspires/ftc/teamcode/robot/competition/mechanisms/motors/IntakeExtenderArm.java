package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeExtenderArm {

    //instance variables

    public DcMotor intakeExtenderArm; //  the arm


    public final DcMotor.RunMode currentRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
    public double extendPosition = 1;    // help confused
    public int retractPosition = 0;   // help confused

    public LinearOpMode intakeLinearOp = null;

    public final double TICKS_PER_ROTATION = 538;



    // constructors
    public void IntakeExtenderArm (DcMotor inArm) {
        intakeExtenderArm = inArm;

        intakeExtenderArm.setDirection(DcMotor.Direction.FORWARD);
        setIntakeArmRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setIntakeArmRunModes(currentRunMode);
    }



    // methods
    public void intakelinearOp (LinearOpMode Op) {
        intakeLinearOp = Op;
    }
    public void stopIntakeMotors () {
        intakeExtenderArm.setPower(0);
    }

    public void setIntakeArmRunModes (DcMotor.RunMode mode) {
        intakeExtenderArm.setMode(mode);
    }



    public void extendingIntakeArm(double speed, double rotations) {
        intakeExtenderArm.setMode(currentRunMode);
    }

    public void retractingIntakeArm(double speed, double rotations) {
        intakeExtenderArm.setMode(currentRunMode);
    }
}
