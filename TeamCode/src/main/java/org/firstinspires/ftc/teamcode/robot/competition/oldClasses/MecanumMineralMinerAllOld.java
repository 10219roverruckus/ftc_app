package org.firstinspires.ftc.teamcode.robot.competition.oldClasses;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.competition.autonomous.GoldPosition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.RevColorDistance;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.TeamMarker;


public class MecanumMineralMinerAllOld {

    public GoldPosition goldPosition = null;
    public LinearOpMode linearOp = null;
    public final int RED_THRESHOLD = 30;                //maybe a higher threshold (12.5)
    public final int BLUE_THRESHOLD = 100;              //maybe a higher threshold (12.5)
    float hsvValues[] = {0F, 0F, 0F};

    // created constant variables that are used for speed (different setting)

    final double SPD_DRIVE_LOW = .20;                  //Lowest speed
    final double SPD_DRIVE_MED = .4;                   //Default is  SPD_MED
    final double SPD_DRIVE_HIGH = .75;
    final double SPD_DRIVE_MAX = 1.0;
    final double SPD_ARM_MED = .5;
    final long sleepTime = 0;

    // variables and constants used by color sensor

    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;


    public void setLinearOp(LinearOpMode Op) {
        linearOp = Op;
    }
        //in future, will pass Camera as Parameter
    public MecanumMineralMinerAllOld() {
    }


    public void findingMineralCamera(double cameraGoldLocation) {

         //find location of the mineral using camera

        if (cameraGoldLocation < 300 && cameraGoldLocation > 1) {
            goldPosition = GoldPosition.MIDDLE;                           //commented out while camera does not work
        } else if (cameraGoldLocation > 300) {                            //program works perfectly DO NOT CHANGE THE CODE
            goldPosition = GoldPosition.RIGHT;
        } else {
            goldPosition = GoldPosition.LEFT;
        }

        goldPosition = goldPosition.RIGHT;
    }



    public void driveMineral(GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor) {

        linearOp.telemetry.addData("MINERAL", goldPosition);
        linearOp.telemetry.update();
        myLiftMotor.extendLiftMotorFullyEncoders();                        // using encoders rather than distance sensor

        myMechDrive.strafeRight(.3, .3);                    // get away from the lander
        myMechDrive.driveForward(.3, .3);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER


        switch (goldPosition) {                                            //Gyro angles robot to push off mineral
            case LEFT:
                myGyro.gyroOrientMecanum(36, myMechDrive);           // different (34)
                myMechDrive.stopMotors();
                myMechDrive.driveForward(SPD_DRIVE_MED, 2.15);    // Moves forward to push off mineral (Originally 1.8)
                break;

            case MIDDLE:
                myGyro.gyroOrientMecanum(4, myMechDrive);            //turning too much towards the right. Need to adjust?
                myMechDrive.stopMotors();
                myMechDrive.driveForward(SPD_DRIVE_MED, 1.7);      // Moves forward to push off mineral
                break;

            case RIGHT:
                myGyro.gyroOrientMecanum(-14, myMechDrive);          // Gyro angles appears correct.
                myMechDrive.stopMotors();
                myMechDrive.driveForward(SPD_DRIVE_MED, 2);        // Moves forward to push off mineral (Originally 1.8)
                break;
        }
    }


}

