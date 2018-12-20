package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MineralLift {
    //instance variables

    public DcMotor mineralLift; //  the arm


    public final DcMotor.RunMode currentRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
    public double extendPosition = 1;    // help confused
    public int retractPosition = 0;   // help confused

    public LinearOpMode linearOp = null;

    public final double TICKS_PER_ROTATION = 538;



    // constructors
    public MineralLift (DcMotor MinM) {
        mineralLift = MinM;

        mineralLift.setDirection(DcMotor.Direction.REVERSE);
        setMineralLiftRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMineralLiftRunModes(currentRunMode);
    }

    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }


    // methods
    public void stopMotors () {
        mineralLift.setPower(0);
    }

    public void setMineralLiftRunModes (DcMotor.RunMode mode) {
        mineralLift.setMode(mode);
    }



    public void extendingMinerallLift(double speed, double rotations) {
        mineralLift.setMode(currentRunMode);
    }

    public void retractingMineralLift(double speed, double rotations) {
        mineralLift.setMode(currentRunMode);
    }

    public void RaiseMineralLift (double power) {
        mineralLift.setPower(power);
    }

    public void LowerMineralLift (double power) {
        mineralLift.setPower(power);
    }
}
