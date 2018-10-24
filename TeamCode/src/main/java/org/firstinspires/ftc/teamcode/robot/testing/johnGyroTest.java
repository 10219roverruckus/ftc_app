package org.firstinspires.ftc.teamcode.robot.testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.testing.mechanisms.Gyro;

public class johnGyroTest extends LinearOpMode  {

    MecanumDrive myMechDrive;

    //gyro
    Gyro myGyro;

    @Override
    public void runOpMode() throws InterruptedException {

        final long sleepTime = 200;
        final double SPD_DRIVE_MED = .5;


        myGyro = new Gyro(hardwareMap.get(BNO055IMU.class, "imu"));
        myMechDrive = new MecanumDrive(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));

        myMechDrive.driveForward(SPD_DRIVE_MED, .65); // move away from the lander toward crater
        myMechDrive.rotateLeft(SPD_DRIVE_MED, 1);
        myMechDrive.driveForward(SPD_DRIVE_MED, 2.6);
        myMechDrive.rotateRight(SPD_DRIVE_MED, .7);

        //testing gyro to adjust to face the depot after it reaches the wall
        sleep(100);
        //LET ORIENT OURSELVES!  (???)
        //myGyro.gyroOrient(70, myMechDrive);

    }
}