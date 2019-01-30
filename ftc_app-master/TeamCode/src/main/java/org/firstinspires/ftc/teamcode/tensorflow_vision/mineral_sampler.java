package org.firstinspires.ftc.teamcode.tensorflow_vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
@TeleOp(name="sampler", group="Iterative Opmode")

public class mineral_sampler extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    public int chosen=1;

    private static final String VUFORIA_KEY = "AfskXVz/////AAAAmVUQUJaKPkpnoCuHva4WBHp7gHrMII3cdGcUGQd5dtWp2UD5UEJL3PxVqiF65V/8Il/lOcm2ce4l4GntFtvH53aaJXHIgNdwgyxSDdstWn2tGJMIn8SFkcKs+h/aGypSX/hUG9MmaUdAVCbrXA1W7+euYEhZjs1tAHButW4T7kZx8tObN4kvM14eX1mGdDAz2I3KhNfkjWP3X+vYibQlClN+i3k5IwpYJdZhYYydFyaWYir0QSsYKmriA/qSbw1RqGUGD7t38EbhmXGslGNVTgQxukKhcO1LlXEiZx/wx1O9MEfObcD1dSSjwI1zQPqRWBFO2tWVWg6HkK9j1IV9YA9LBMwDXsMiC5DztTB6/LZ7";

    private VuforiaLocalizer vuforia;


    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        chosen = 1;
        begin(hardwareMap);
        waitForStart();
        update();
        telemetry.addData("Ales:",chosen);
        telemetry.update();
        end();

            while (opModeIsActive()) {

                    }

    }

    public void update() // finding the position of the gold cube
    {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        if (updatedRecognitions.size() == 3)
        {
            int goldMineralX = -1;
            int silverMineral1X = -1;
            int silverMineral2X = -1;
            for (Recognition recognition : updatedRecognitions)
            {
                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
                {
                    
                    goldMineralX = (int) recognition.getLeft();
                }
                else if (silverMineral1X == -1)
                {
                    silverMineral1X = (int) recognition.getLeft();
                }
                else
                {
                    silverMineral2X = (int) recognition.getLeft();
                }
            }
            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1)
            {
                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X)
                {
                    chosen = 2;
                 //   telemetry.addData("Gold Mineral Position", "Left");
                }
                else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X)
                {
                    chosen = 0;
                   // telemetry.addData("Gold Mineral Position", "Right");
                }
                else
                {
                    chosen = 1;
                    //telemetry.addData("Gold Mineral Position", "Center");
                }
            }
        }
    }

    public void begin(HardwareMap x) // initializing
    {
        initVuforia();
        initTfod(x);
        tfod.activate();
    }

    public void end() // closing
    {
        tfod.shutdown();
        com.vuforia.CameraDevice.getInstance().setFlashTorchMode(false);
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        com.vuforia.CameraDevice.getInstance().setFlashTorchMode(false);
    }


    private void initTfod(HardwareMap x) {
        int tfodMonitorViewId = x.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", x.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.20f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
