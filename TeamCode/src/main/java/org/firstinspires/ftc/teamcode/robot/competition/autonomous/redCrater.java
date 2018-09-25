package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.competition.contructor.motors.mechDriveAuto;

public class redCrater extends LinearOpMode {

    mechDriveAuto myMechDriveAuto;


    @Override
    public void runOpMode() throws InterruptedException {
        myMechDriveAuto = new mechDriveAuto(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));


    }
}
