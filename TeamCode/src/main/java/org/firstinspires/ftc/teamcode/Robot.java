package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// i am cool
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;

public class Robot {

    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;

    public void init(HardwareMap ahwMap, Telemetry t){
        HardwareMap hwMap;
        hwMap = ahwMap;

        front_left = hwMap.get(DcMotor .class, "fldrive");
        front_right = hwMap.get(DcMotor.class, "frdrive");
        back_left = hwMap.get(DcMotor.class, "bldrive");
        back_right = hwMap.get(DcMotor.class, "brdrive");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        setDriveStop(0);

        // Set all motors to run without encoders.
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDriveStop(double power) {
        front_left.setPower(power);
        front_right.setPower(power);
        back_left.setPower(power);
        back_right.setPower(power);
    }

    public void setRunMode(DcMotor.RunMode mode){
        front_right.setMode(mode);
        front_left.setMode(mode);
        back_right.setMode(mode);
        back_left.setMode(mode);
    }

}
