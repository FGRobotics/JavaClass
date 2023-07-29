package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.simple_rr_example.MecBot;

@Autonomous(name = "Week1Challenge", group = "Challenge")
public class Week1Challenge extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor m1 = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor m2 = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor m3 = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor m4 = hardwareMap.dcMotor.get("back_right_motor");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";
        imu.initialize(parameters);
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");

        //Actions start after this
        waitForStart();

        while(opModeIsActive()){

            while(backDistance.getDistance(DistanceUnit.CM) >= 65){
                m1.setPower(-1);
                m2.setPower(-1);
                m3.setPower(-1);
                m4.setPower(-1);
            }
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);
            double length = m1.getCurrentPosition();

            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle * 180.0 / Math.PI <= 90){
                m1.setPower(-1);
                m2.setPower(-1);
                m3.setPower(1);
                m4.setPower(1);
                telemetry.addData("imu value:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle * 180.0 / Math.PI);
                telemetry.addData("",length);
                telemetry.update();
            }
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);

            m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while(m1.getCurrentPosition() >= length){
                m1.setPower(-1);
                m2.setPower(-1);
                m3.setPower(-1);
                m4.setPower(-1);
            }
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);

            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle * 180.0 / Math.PI <= 175){
                m1.setPower(-1);
                m2.setPower(-1);
                m3.setPower(1);
                m4.setPower(1);
                telemetry.addData("imu value:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle * 180.0 / Math.PI);
                telemetry.update();
            }
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);

            m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while(m1.getCurrentPosition() >= length){
                m1.setPower(-1);
                m2.setPower(-1);
                m3.setPower(-1);
                m4.setPower(-1);
            }
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);

            while(-85 < imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle * 180.0 / Math.PI ||
                    imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle * 180.0 / Math.PI< -95){
                m1.setPower(-1);
                m2.setPower(-1);
                m3.setPower(1);
                m4.setPower(1);
                telemetry.addData("imu value:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle * 180.0 / Math.PI);
                telemetry.update();
            }
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);

            m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while(m1.getCurrentPosition() >= length){
                m1.setPower(-1);
                m2.setPower(-1);
                m3.setPower(-1);
                m4.setPower(-1);
            }
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);

            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle * 180.0 / Math.PI <= 0){
                m1.setPower(-1);
                m2.setPower(-1);
                m3.setPower(1);
                m4.setPower(1);
                telemetry.addData("imu value:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle * 180.0 / Math.PI);
                telemetry.update();
            }
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);

           break;
        }

    }
}
