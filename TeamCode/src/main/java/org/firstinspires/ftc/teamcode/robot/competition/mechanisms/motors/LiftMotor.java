package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LiftMotor {
    //instance variables

    public DcMotor liftMotor; //  the arm
    double topHeight = 8.4;
    double lowHeight = 3.0;

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
    public void extendLiftMotorFully (DistanceSensor distanceSensor) {
       while (distanceSensor.getDistance(DistanceUnit.INCH)  < topHeight) {
           liftMotor.setPower(-1);
       }
        liftMotor.setPower(0);
    }

    //function that fully retracts arm using distance sensor.
    public void retractLiftMotorFully(DistanceSensor distanceSensor) {
        //set motor to full power WHILE the distance sensor is less than lowHeight
        //be sure to stop motor at end!
        while (distanceSensor.getDistance(DistanceUnit.INCH) > lowHeight) {
            liftMotor.setPower(1);
        }
        liftMotor.setPower(0);
    }
}
