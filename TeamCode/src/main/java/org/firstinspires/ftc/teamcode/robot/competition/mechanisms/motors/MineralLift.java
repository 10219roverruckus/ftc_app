package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.RevColorDistance;

public class MineralLift {
    //instance variables

    public DcMotor mineralLift; //  the arm


    public final DcMotor.RunMode currentRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
    public double extendPosition = 1;    // help confused
    public int retractPosition = 0;   // help confused

    public LinearOpMode linearOp = null;

    public final double TICKS_PER_ROTATION = 538;

    public ElapsedTime liftRunTime;
    double maxArmExtendTime = 2; //max time for arm to run, in SECONDS. (for lowering robot)
    double getMaxArmExtendTimeEncoder = 3;
    double maxLiftRetractTime = 2; //max time for arm to run, in SECONDS. (for lowering robot)
    int mineralLiftTargetPosition = 3900;



    // constructors
    public MineralLift (DcMotor MinM) {
        mineralLift = MinM;

        mineralLift.setDirection(DcMotor.Direction.REVERSE);
        mineralLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public void RaiseMineralLift () {
        mineralLift.setPower(-1);
    }

    public void LowerMineralLift () {
        mineralLift.setPower(1);
    }


    public void retractLiftMotorFully() {               //DistanceSensor distanceSensor
        //set motor to full power WHILE the distance sensor is less than lowHeight
        //be sure to stop motor at end!
        liftRunTime.reset();
        while (liftRunTime.time() <= maxLiftRetractTime) {
            mineralLift.setPower(1);
        }
        mineralLift.setPower(0);
    }

    public void extendLiftMotorFullyEncoders () {
        liftRunTime.reset();
        while (mineralLift.getCurrentPosition() > mineralLiftTargetPosition) {
            linearOp.telemetry.addData("ENCODER", mineralLift.getCurrentPosition());
            linearOp.telemetry.update();
            mineralLift.setPower(-.75);
            if (liftRunTime.time() >= getMaxArmExtendTimeEncoder) {
                linearOp.telemetry.addLine("BREAK");
                linearOp.telemetry.update();
                break;
            }
            linearOp.idle();
        }
        mineralLift.setPower(0);
    }

}
