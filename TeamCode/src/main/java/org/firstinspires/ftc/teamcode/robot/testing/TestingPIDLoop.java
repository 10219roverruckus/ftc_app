package org.firstinspires.ftc.teamcode.robot.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Testing PID Loops")
public class TestingPIDLoop extends LinearOpMode{

    MecanumDrive myMechDrive;


    public void runOpMode() throws InterruptedException {

        myMechDrive = new MecanumDrive(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));
        myMechDrive.setLinearOp(this);

        while (opModeIsActive()) {
            myMechDrive.driveForwardPID(1, 1);
        }

    }
}
