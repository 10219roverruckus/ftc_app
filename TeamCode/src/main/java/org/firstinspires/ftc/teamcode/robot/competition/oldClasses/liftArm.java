package org.firstinspires.ftc.teamcode.robot.competition.oldClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class liftArm {

    private DcMotor liftArmMotor;
    private int extendDuration;
    private double extendPower;
    private int retractDuration;
    private double retractPower;

    public liftArm (DcMotor liftAM) {
        liftArmMotor = liftAM;
        liftArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendDuration = 360;
        extendPower = .5;
        retractDuration = 360;
        retractPower = .5;
    }


    public void extend () {
        liftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (liftArmMotor.getCurrentPosition()<= extendDuration){
            liftArmMotor.setPower(extendPower);
        }
        liftArmMotor.setPower(0);
    }
    public void retract(){
        liftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (liftArmMotor.getCurrentPosition()<= retractDuration) {
            liftArmMotor.setPower(retractPower);
        }
        liftArmMotor.setPower(0);
    }
}
