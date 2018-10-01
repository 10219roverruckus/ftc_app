package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.competition.constructor.motors.mechDriveAuto;
import org.firstinspires.ftc.teamcode.robot.competition.constructor.motors.liftArm;

public class redCrater extends LinearOpMode {
    private int movement = 0;
    mechDriveAuto myMechDriveAuto;
    liftArm myLiftArm;



    @Override
    public void runOpMode() throws InterruptedException {
        myMechDriveAuto = new mechDriveAuto(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));
        myLiftArm = new liftArm(hardwareMap.dcMotor.get("lift_motor"));
        waitForStart();

        while (opModeIsActive()){
            switch (movement) {
                case 0:
                    movement++;
                    break;
                case 1:
                    myLiftArm.extend();
                    sleep(100);
                    movement++;
                    break;
                case 2:
                    movement++;
                    break;
                case 3:
                    movement++;
                    break;
                case 4:
                    movement++;
                    break;

            }
        }
    }

}
