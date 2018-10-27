package org.firstinspires.ftc.teamcode.robot.competition.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.competition.autonomous.GoldPosition;
import org.firstinspires.ftc.teamcode.robot.testing.mechanisms.Gyro;
import org.firstinspires.ftc.teamcode.robot.outreach.mechanisms.motors.DriveMotors;



public class MineralMiner {

    public GoldPosition goldPosition = null;
    public LinearOpMode linearOp = null;

    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }

    //in future, will pass Camera as Parameter
    public MineralMiner () { }

    public void findingMineral () {
//        if (detector.getXPosition() < 280) {
//            goldPosition = GoldPosition.LEFT;
//        }
//        else if (detector.getXPosition() > 500) {
//            goldPosition = GoldPosition.RIGHT;
//        }
//        else {
//            goldPosition = GoldPosition.MIDDLE;
//        }

        goldPosition = GoldPosition.RIGHT;
    }

    public void driveMineral (Gyro myGyro, DriveMotors myDriveMotors) {
        myDriveMotors.drivePID(.3, .5);
        switch(goldPosition) {
            case LEFT:
                linearOp.telemetry.addData("MINERAL POSITION: ", goldPosition);
                linearOp.telemetry.update();
                linearOp.sleep(2000);
                myGyro.gyroOrientOutreach(34, myDriveMotors);
                myDriveMotors.drivePID(.4, 3.4);
                break;

            case MIDDLE:
                linearOp.telemetry.addData("MINERAL POSITION: ", goldPosition);
                linearOp.telemetry.update();
                linearOp.sleep(2000);
                myGyro.gyroOrientOutreach(7, myDriveMotors);
                myDriveMotors.drivePID(.4, 2.8);
                break;

            case RIGHT:
                linearOp.telemetry.addData("MINERAL POSITION: ", goldPosition);
                linearOp.telemetry.update();
                linearOp.sleep(2000);
                myGyro.gyroOrientOutreach(-11, myDriveMotors);
                myDriveMotors.drivePID(.4, 2.7);
                break;
        }
        // now, all 3 cases can do the depot thing.
        // use same method for all.
    }
    public void redCraterMineralToWall (Gyro myGyro, DriveMotors myDriveMotors) {
        //back up to red tape
        // gyro turn 90
        //switch goldPosition
            //left - forward to wall
            //middle - forward to wall
            //right - forward to wall
        //calls new method to turn straight twoards depot and do "stuff"
        craterToDepot ();

    }

    public void craterToDepot () {

    }

}
