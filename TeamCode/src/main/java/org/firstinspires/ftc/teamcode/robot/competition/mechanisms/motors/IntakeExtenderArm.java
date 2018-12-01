package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeExtenderArm {

    //instance variables                        ***** had to change it to motor instead of servo 11/24/18 Emma ****

    public DcMotor intakeExtenderArm; //  the arm


    public double extendPosition = 1;
    public int retractPosition = 0;

    public LinearOpMode intakeLinearOp = null;

    public final double TICKS_PER_ROTATION = 538;

    public double rateOfChange = .001;
    public double currentPosition;





    // constructors
    public IntakeExtenderArm (DcMotor inArm) {
        intakeExtenderArm = inArm;

        intakeExtenderArm.setDirection(DcMotor.Direction.FORWARD);
        intakeExtenderArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }



    // methods
    public void intakelinearOp (LinearOpMode Op) {
        intakeLinearOp = Op;
    }

    public void extendIntakeArm (double motorPower) {

        intakeExtenderArm.setPower(motorPower);
    }

    public void retractIntactArm (double motorPower) {
        intakeExtenderArm.setPower(motorPower);
    }

    public void stopIntakeArm () {
        intakeExtenderArm.setPower(0);
    }


}
