package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.RevColorDistance;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.TeamMarker;


@Autonomous(name = "Depot - competition")

public class Depot extends LinearOpMode  {

MecanumDrive myMechDrive;

GyroCompetition myGyro;
MecanumMineralMiner myMineralMiner;
RevColorDistance myRevColorDistance;

    LiftMotor myLiftMotor;
    TeamMarker myTeamMarker;

@Override
public void runOpMode() throws InterruptedException {



    final long sleepTime = 100;
    final double SPD_DRIVE_MED = .5;


    myMechDrive = new MecanumDrive(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));
    myMechDrive.setLinearOp(this);

    myGyro = new GyroCompetition(hardwareMap.get(BNO055IMU.class, "imu"));
    myGyro.setLinearOp(this);

    myMineralMiner = new MecanumMineralMiner();
    myMineralMiner.setLinearOp(this);

    myLiftMotor = new LiftMotor(hardwareMap.dcMotor.get("lift_motor"));
    myLiftMotor.setLinearOp(this);

    myTeamMarker = new TeamMarker(hardwareMap.servo.get("team_marker_arm"));
    myTeamMarker.setLinearOp(this);

    myRevColorDistance = new RevColorDistance(hardwareMap.get(ColorSensor.class, "rev_sensor_color_distance"), hardwareMap.get(DistanceSensor.class, "rev_sensor_color_distance"));



    waitForStart();

    boolean active = true;
    while (opModeIsActive() && !isStopRequested()) {
        while (active && !isStopRequested()) {
            idle();
            /*
            Find the correct gold mineral
             */
            myMineralMiner.findingMineral();
            sleep(sleepTime);
            idle();
            /*
            1) drives forward from lander a short distance so doesn't interfere with gyro turn
            2) angles self with gold mineral based on myMineralMiner.findingMineral
            3) Drives forward to knock of gold mineral.
             */
            myMineralMiner.driveMineral(myGyro, myMechDrive, myLiftMotor);
            sleep(sleepTime);
            idle();
            /*
            1) BACKS UP TO TAPE
            2) TURNS TO A) MISS LANDER AND AND B) MISS MINERALS WHEN GOING STRAIGHT
            3) GOES STRAIGHT TOWARDS WALL
             */
            myMineralMiner.mineralToDepot (myGyro, myMechDrive, myRevColorDistance, myTeamMarker);
            sleep(sleepTime);

            idle();
            /*
            Will angle robot to be parallel with robot, score in depot, and then go to crater.
             */
         //   myMineralMiner.depotToCrater(myGyro, myMechDrive, myRevColorDistance);

            active = false;
        }
        idle();
        requestOpModeStop();
    }
}
}