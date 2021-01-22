package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous Basic", group="Production")

public class Autonomous_Basic extends LinearOpMode {
    Robot robot = new Robot (this);
    int Target_Zone;
    boolean run;

    private boolean inInitializationState() {
        return (!opModeIsActive() && !isStopRequested());
    }
    @Override
    public void runOpMode(){
        String ringpattern = "";
        robot.getDriveTrain().init();
        robot.getComputerVision().init();
        robot.getImuControl().init();
        robot.getImuControl().readimuheading();
        robot.getRingControl().init();
        robot.getWobbleGripper().init();
        sleep(500);
        
        ElapsedTime timer = new ElapsedTime();
        int count = 0;

        while (inInitializationState()) {
            robot.getWobbleGripper().setWobbleGripper(0);

            telemetry.addData("Imu", robot.getImuControl().readimuheading());
            telemetry.update();
        }
        if (opModeIsActive()) {

            telemetry.addData("Imu", robot.getImuControl().readimuheading());
            telemetry.update();
            robot.getRingControl().DriverControlledRingShooter(.5);

            robot.getDriveTrain().gyroDrive(-44.5, -44.5, -44.5, -44.5, .3, 1);
            robot.getRingControl().ConstantRingShooter(0.9);
            sleep(500);
            robot.getRingControl().DriverControlledRingShooter(0.24);
            sleep(500);
            robot.getRingControl().DriverControlledRingShooter(0.5);
            sleep(500);
            robot.getRingControl().DriverControlledRingShooter(0.24);
            sleep(500);
            robot.getRingControl().DriverControlledRingShooter(0.5);
            sleep(500);
            robot.getRingControl().DriverControlledRingShooter(0.24);
            sleep(500);
            robot.getRingControl().DriverControlledRingShooter(0.5);
            sleep(500);
            robot.getDriveTrain().moveToColor("white", -0.1);

            }

    }

}









