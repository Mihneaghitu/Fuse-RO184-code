package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.disnodeteam.dogecv.filters.GrayscaleFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AssetsTrajectoryManager;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name ="RR si OpenCv",group = "drive")

public class RR_OpenCv extends LinearOpMode {
    public static double DISTANCE = 30;
    private OpenCvWebcam webcam;
    private SkystoneDetector skystoneDetector;
    public static double yellow = 100;
    public static int gray = 40;
    public static String position = null;
    public ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class,"camera porno"));
        webcam.openCameraDevice();
        skystoneDetector = new SkystoneDetector();
        skystoneDetector.blackFilter = new GrayscaleFilter(0, gray);
        skystoneDetector.yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, yellow);
        webcam.setPipeline(skystoneDetector);
        webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard ftcDashboard = FtcDashboard.getInstance();
        ftcDashboard.startCameraStream(webcam, 30);


        waitForStart();


        if (isStopRequested()) return;
        position = detecteaza(robot);
        runtime.reset();
        while (runtime.seconds() < 1.5){
            telemetry.addData("pozitie: ", position);
            telemetry.update();
        }
        if(position == "dreapta") tr_dr(robot);
        else if(position == "mijloc") tr_mij(robot);
        else tr_stg(robot);

    }

    //cod detectare skystone

    public String detecteaza(SampleMecanumDrive robot){
        double x_position =0 , y_position= 0;
        sleep(200);

        runtime.reset();
        while (runtime.seconds() < 2.0){
            x_position = skystoneDetector.getScreenPosition().x;
            y_position = skystoneDetector.getScreenPosition().y;
        }

        if( x_position < 330){
            return "stanga";
        }else if( x_position > 380 && x_position< 500){
            return "mijloc";
        }else return "dreapta";
    }

    public void tr_dr(SampleMecanumDrive robot){
        Trajectory traj0 = robot.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Pose2d(30,30))
                .build();
        Trajectory traj1 = robot.trajectoryBuilder(new Pose2d())
                .addDisplacementMarker(()->{
                    robot.prins.setPosition(0.5);
                })
                .build();
        robot.followTrajectory(traj0);
        robot.followTrajectoryAsync(traj1);
    }
    public void tr_mij(SampleMecanumDrive robot){

    }
    public void tr_stg(SampleMecanumDrive robot){

    }


}
