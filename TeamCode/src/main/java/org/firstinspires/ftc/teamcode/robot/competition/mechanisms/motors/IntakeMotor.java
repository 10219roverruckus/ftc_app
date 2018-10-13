package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeMotor {
    //instance variables

    public DcMotor intakeMotor; //  the motor


    public final DcMotor.RunMode currentRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
    public double extendPosition = 1;    // help confused
    public int retractPosition = 0;   // help confused

    public LinearOpMode intakeLinearOp = null;

    public final double TICKS_PER_ROTATION = 538;



    // constructors
    public void IntakeMotor (DcMotor inMotor) {
        intakeMotor = inMotor;

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        setIntakeMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setIntakeMotorRunModes(currentRunMode);
    }



    // methods
    public void intakelinearOp (LinearOpMode Op) {
        intakeLinearOp = Op;
    }
    public void stopIntakeMotor () {
        intakeMotor.setPower(0);
    }

    public void setIntakeMotorRunModes (DcMotor.RunMode mode) {
        intakeMotor.setMode(mode);
    }

    public void spinIntakeMotor(double speed, double rotations) {
        intakeMotor.setMode(currentRunMode);
    }

}
