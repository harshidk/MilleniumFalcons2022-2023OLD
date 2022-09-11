package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

public class RobotBrain {
    OpMode opmode;
    Robot robot;

    public RobotBrain(Robot therobot, OpMode theopmode) {
        robot = therobot;
        opmode = theopmode;
    }
    public boolean opModeIsActive() {
        if (opmode instanceof LinearOpMode)
            return ((LinearOpMode) opmode).opModeIsActive();
        else return false;
    }


}
