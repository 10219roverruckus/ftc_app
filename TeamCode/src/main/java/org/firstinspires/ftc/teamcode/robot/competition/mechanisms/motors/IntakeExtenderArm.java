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

    public double rateOfChange = .001;
    public double currentPosition;

    // instance variables for auto
    double maxIntakeArmExtendTime = 2; //max time for arm to run, in SECONDS. (for lowering robot)
    double getMaxIntakeArmExtendTimeEncoder = 3;
    double maxIntakeArmRetractTime = 2; //max time for arm to run, in SECONDS. (for lowering robot)
    int intakeArmTargetPosition = -6700;

    public ElapsedTime armRunTime;





    // constructors
    public IntakeExtenderArm (DcMotor inArm) {
        intakeExtenderArm = inArm;

        intakeExtenderArm.setDirection(DcMotor.Direction.FORWARD);
        intakeExtenderArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        armRunTime.reset();
        while (intakeExtenderArm.getCurrentPosition() > intakeArmTargetPosition) {
            linearOp.telemetry.addData("ENCODER", intakeExtenderArm.getCurrentPosition());
            linearOp.telemetry.update();
            intakeExtenderArm.setPower(-.75);
            if (armRunTime.time() >= getMaxIntakeArmExtendTimeEncoder) {
                linearOp.telemetry.addLine("BREAK");
                linearOp.telemetry.update();
                break;
            }
            linearOp.idle();
        }
        intakeExtenderArm.setPower(0);

    }

    public void retractIntakeArmAuto () {
        armRunTime.reset();
        while (armRunTime.time() <= maxIntakeArmExtendTime) {
            intakeExtenderArm.setPower(1);
        }
        intakeExtenderArm.setPower(0);
    }


}
