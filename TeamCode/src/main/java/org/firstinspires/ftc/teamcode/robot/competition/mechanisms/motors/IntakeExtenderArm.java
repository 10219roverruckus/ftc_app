package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeExtenderArm {

    //instance variables                        ***** had to change it to motor instead of servo 11/24/18 Emma ****

    public DcMotor intakeExtenderArm; //  the arm


    public double extendPosition = 1;
    public int retractPosition = 0;

    public LinearOpMode intakeLinearOp = null;
    public LinearOpMode linearOp = null;

    public final double TICKS_PER_ROTATION = 538;
    public final double autononomousPower = .75;

    public double rateOfChange = .001;
    public double currentPosition;

    // instance variables for auto
    double maxIntakeArmExtendTime = 2; //max time for arm to run, in SECONDS. (for lowering robot)
    double MaxIntakeArmRetractTime = 2; //max time for arm to run, in SECONDS. (for lowering robot)
    int intakeArmExtendTargetPosition = -2400;        // fully extended
    int autoIntakeArmExtendTargetPosition = -1700;      //extend for minerals
    int intakeArmExtendTargetPositionOverCrater = -900;
    int intakeArmRetractTargetPosition = -0;

    public ElapsedTime ExtenderArmRunTime;





    // constructors
    public IntakeExtenderArm (DcMotor inArm) {
        intakeExtenderArm = inArm;

        intakeExtenderArm.setDirection(DcMotor.Direction.FORWARD);
        intakeExtenderArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ExtenderArmRunTime = new ElapsedTime();
    }



    // methods

    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }

    public void intakelinearOp (LinearOpMode Op) {
        intakeLinearOp = Op;
    }

    public void extendIntakeArm (double motorPower) {
        intakeExtenderArm.setPower(Math.abs(motorPower));
    }

    public void retractIntactArm (double motorPower) {
        intakeExtenderArm.setPower(-Math.abs(motorPower));
    }

    public void stopIntakeArm () {
        intakeExtenderArm.setPower(0);
    }


    // autonomous methods

    public void extendIntakeArmAuto () {
        ExtenderArmRunTime.reset();
        while (intakeExtenderArm.getCurrentPosition() > autoIntakeArmExtendTargetPosition) {
            linearOp.telemetry.addData("extender encoder EXTEND ", intakeExtenderArm.getCurrentPosition());
            linearOp.telemetry.update();
            intakeExtenderArm.setPower(-autononomousPower);
            if (ExtenderArmRunTime.time() >= maxIntakeArmExtendTime) {
//                linearOp.telemetry.addLine("BREAK");
//                linearOp.telemetry.update();
                break;
            }
            linearOp.idle();
        }
        stopIntakeArm();
    }

    public void retractIntakeArmAuto () {
        ExtenderArmRunTime.reset();
        while (intakeExtenderArm.getCurrentPosition() <  intakeArmRetractTargetPosition) {
            linearOp.telemetry.addData("extender encoder RETRACT: ", intakeExtenderArm.getCurrentPosition());
            linearOp.telemetry.update();

            intakeExtenderArm.setPower(autononomousPower);
            if (ExtenderArmRunTime.time() >= maxIntakeArmExtendTime) {
//                linearOp.telemetry.addLine("BREAK");
//                linearOp.telemetry.update();
                break;
            }
            linearOp.idle();
        }
        stopIntakeArm();
    }

    public void extendIntakeArmAllTheWay () {
        ExtenderArmRunTime.reset();
        while (intakeExtenderArm.getCurrentPosition() > intakeArmExtendTargetPosition) {
            linearOp.telemetry.addData("extender encoder EXTEND ", intakeExtenderArm.getCurrentPosition());
            linearOp.telemetry.update();
            intakeExtenderArm.setPower(-autononomousPower);
            if (ExtenderArmRunTime.time() >= maxIntakeArmExtendTime) {
//                linearOp.telemetry.addLine("BREAK");
//                linearOp.telemetry.update();
                break;
            }
            linearOp.idle();
        }
        stopIntakeArm();
    }


    public void extendOverCrater () {
        ExtenderArmRunTime.reset();
        while (intakeExtenderArm.getCurrentPosition() > intakeArmExtendTargetPositionOverCrater) {
            linearOp.telemetry.addData("extender encoder EXTEND ", intakeExtenderArm.getCurrentPosition());
            linearOp.telemetry.update();
            intakeExtenderArm.setPower(-autononomousPower);
            if (ExtenderArmRunTime.time() >= maxIntakeArmExtendTime) {
//                linearOp.telemetry.addLine("BREAK");
//                linearOp.telemetry.update();
                break;
            }
            linearOp.idle();
        }
        stopIntakeArm();
    }


}
