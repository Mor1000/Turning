package org.firstinspires.ftc.teamcode;

import android.provider.BaseColumns;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.sun.tools.javac.tree.DCTree;
import com.sun.tools.javac.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


/**
 * Summary: This class stores functions for turning the robot.
 */
public class TurnUtilities {
    private static final String TAG = "message";


    /**
     * @param goalAngle   desired position angle for turning
     * @param driveMotors the robot drivetrain motors
     * @param imu         gyro sensor for measuring the angle
     * @param power       turning motors power
     */
    void Turn(double goalAngle, DcMotor[][] driveMotors, BNO055IMU imu, double power, Telemetry telemetry) {
        boolean sideOfTurn = true;//true = turn clockwise, false = turn counter-clockwise
        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        //finding the shorter direction to turn
        if (goalAngle == 0) {
            if (currentAngle < 0) {
                sideOfTurn = false;    //counter-clockwise
            } else if (currentAngle > 0) {
                sideOfTurn = true;    //clockwise
            }
        } else if (currentAngle <= 0 && goalAngle < 0 &&
                currentAngle < goalAngle) {
            sideOfTurn = false;     //counter-clockwise

        } else if (currentAngle <= 0 && goalAngle < 0
                && currentAngle > goalAngle) {
            sideOfTurn = true;    //clockwise

        } else if (currentAngle >= 0 && goalAngle > 0 &&
                currentAngle < goalAngle)

        {
            sideOfTurn = false;    //counter-clockwise

        } else if (currentAngle >= 0 && goalAngle > 0 &&
                currentAngle > goalAngle) {
            sideOfTurn = true;    //clockwise

        } else if (currentAngle >= 0 && goalAngle < 0)

        {
            if (currentAngle - goalAngle >
                    360 - (currentAngle - goalAngle)) {
                sideOfTurn = false;    //counter-clockwise
            } else {
                sideOfTurn = true;    //clockwise
            }
        } else if (currentAngle <= 0 && goalAngle > 0)

        {
            if (goalAngle - currentAngle <
                    360 - (goalAngle - currentAngle)) {
                sideOfTurn = false;    //counter-clockwise
            } else {
                sideOfTurn = true;    //clockwise
            }

        }
        android.util.Log.d(TAG, "Turn direction: " + sideOfTurn);
        if (sideOfTurn) {
            setMotorPower(driveMotors, new double[][]{{power, -power}, {power, -power}});
            if (goalAngle > 0 && currentAngle < 0)//edge case
                while (goalAngle > currentAngle) {
                    currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                    telemetry.addData("angle:", currentAngle);
                    telemetry.update();
                }
                //motors running

            else {

                while (goalAngle < currentAngle) {
                    currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                    telemetry.addData("angle:", currentAngle);
                    telemetry.update();
                }

                //motors running

            }
        } else {
            setMotorPower(driveMotors, new double[][]{{-power, power}, {-power, power}});
            if (goalAngle < 0 && currentAngle > 0) //edge case
                while (goalAngle < currentAngle) {
                    currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                    telemetry.addData("angle:", currentAngle);
                    telemetry.update();
                }
                //motors running


            else
                while (goalAngle > currentAngle) {
                    currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                    telemetry.addData("angle:", currentAngle);
                    telemetry.update();
                }
            //motors running

        }
        setMotorPower(driveMotors, new double[][]{{0, 0}, {0, 0}});
    }

    public static void ScaledTurn(double goalAngle, DcMotor[][] driveMotors, BNO055IMU imu, double power, Telemetry telemetry) {
        boolean sideOfTurn = true;
        double currentAngle = getCurrentScaledAngle(imu);
        if (currentAngle < goalAngle) {
            if (goalAngle - currentAngle <= 360 - (goalAngle - currentAngle))
                sideOfTurn = false;
            else
                sideOfTurn = true;

        } else {
            if (currentAngle - goalAngle <= 360 - (currentAngle - goalAngle))
                sideOfTurn = true;
            else
                sideOfTurn = false;
        }
        if (sideOfTurn) {
            setMotorPower(driveMotors, new double[][]{{power, -power}, {power, -power}});
            if (goalAngle > 180 && currentAngle < 180)//edge case
                while (goalAngle > currentAngle) {
                    currentAngle = getCurrentScaledAngle(imu);
                    telemetry.addData("angle:", currentAngle);
                    telemetry.update();
                }
                //motors running

            else {

                while (goalAngle < currentAngle) {
                    currentAngle = getCurrentScaledAngle(imu);
                    telemetry.addData("angle:", currentAngle);
                    telemetry.update();
                }

                //motors running

            }
        } else {
            setMotorPower(driveMotors, new double[][]{{-power, power}, {-power, power}});
            if (goalAngle < 180 && currentAngle > 180) //edge case
                while (goalAngle < currentAngle) {
                    currentAngle = getCurrentScaledAngle(imu);
                    telemetry.addData("angle:", currentAngle);
                    telemetry.update();
                }
                //motors running


            else
                while (goalAngle > currentAngle) {
                    currentAngle = getCurrentScaledAngle(imu);
                    telemetry.addData("angle:", currentAngle);
                    telemetry.update();
                }
            //motors running

        }
        setMotorPower(driveMotors, new double[][]{{0, 0}, {0, 0}});
    }

    public static void ScaledTurn2(double goalAngle, DcMotor[][] driveMotors, BNO055IMU imu, double power, Telemetry telemetry) {
        boolean sideOfTurn = true;
        double deltaAngle = 0;
        boolean directTurn=true;
        double currentAngle = getCurrentScaledAngle(imu);
        double angle0 = currentAngle;
        if (currentAngle < goalAngle) {
            if (goalAngle - currentAngle <= 360 - (goalAngle - currentAngle)) {
                sideOfTurn = false;
                deltaAngle = goalAngle - currentAngle;
            } else {
                sideOfTurn = true;
                deltaAngle = 360 - (goalAngle - currentAngle);
                directTurn=false;
            }


        } else {
            if (currentAngle - goalAngle <= 360 - (currentAngle - goalAngle)) {
                sideOfTurn = true;
                deltaAngle = currentAngle - goalAngle;
            } else {
                sideOfTurn = false;
                deltaAngle = 360 - (currentAngle - goalAngle);
                directTurn=false;
            }
        }
        if (sideOfTurn)
            setMotorPower(driveMotors, new double[][]{{power, -power}, {power, -power}});
        else
            setMotorPower(driveMotors, new double[][]{{-power, power}, {-power, power}});

        if (!directTurn&&goalAngle > 180 && currentAngle < 180)//edge case
        while ((Math.abs(angle0 - currentAngle) <= deltaAngle) || (currentAngle > 180 && 360 - Math.abs((angle0 - currentAngle)) <= deltaAngle)) {//motors running
                currentAngle = getCurrentScaledAngle(imu);
                telemetry.addData("angle case 1:", currentAngle);
                telemetry.update();
            }

        else if (!directTurn&&goalAngle < 180 && currentAngle > 180) //edge case
            while ((Math.abs(angle0 - currentAngle) <= deltaAngle) || (currentAngle < 180 && 360 - Math.abs((angle0 - currentAngle)) <= deltaAngle)) {//motors running
                currentAngle = getCurrentScaledAngle(imu);
                telemetry.addData("angle case 2:", currentAngle);
                telemetry.update();
            }

        else {

            while (Math.abs(angle0 - currentAngle) <= deltaAngle) {  //motors running
                currentAngle = getCurrentScaledAngle(imu);
                telemetry.addData("angle case 3:", currentAngle);
                telemetry.update();
            }
        }
        setMotorPower(driveMotors, new double[][]{{0, 0}, {0, 0}});
    }

    public static void setMotorPower(DcMotor[][] motors, double[][] powers) {
        for (int i = 0; i < motors.length; i++)
            for (int j = 0; j < motors[i].length; j++)
                motors[i][j].setPower(powers[i][j]);
    }

    public static double getCurrentScaledAngle(BNO055IMU imu) {
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (angle < 0)
            angle += 360;
        return angle;
    }
}