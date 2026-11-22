package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

public final class CameraLL {
    public static class PARAMS {
        public static List<Integer> BLUE_AUD_TARGET_TAG_ID = Arrays.asList(38,39,40,41);
        public static List<Integer> BLUE_NOTAUD_TARGET_TAG_ID = Arrays.asList(42,43,44,45);
        public static List<Integer> RED_AUD_TARGET_TAG_ID = Arrays.asList(34,35,36,37);
        public static List<Integer> RED_NOTAUD_TARGET_TAG_ID = Arrays.asList(30,31,32,33);

        private static boolean audienceSide = false;
        public static String Alliance = "Blue";
        public static boolean tip = false;
    }
    public Limelight3A limelight;






    public CameraLL(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);
        limelight.start();


    }

    public void detectAprilTag() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                if (Objects.equals(PARAMS.Alliance, "Blue")){
                    if ((PARAMS.BLUE_AUD_TARGET_TAG_ID.contains(fid.getFiducialId()) && PARAMS.audienceSide) || (PARAMS.BLUE_NOTAUD_TARGET_TAG_ID.contains(fid.getFiducialId()) && !PARAMS.audienceSide)) {
                        PARAMS.tip = true;
                        break;
                    }
                } else {
                    if ((PARAMS.RED_AUD_TARGET_TAG_ID.contains(fid.getFiducialId()) && PARAMS.audienceSide) || (PARAMS.RED_NOTAUD_TARGET_TAG_ID.contains(fid.getFiducialId()) && !PARAMS.audienceSide)) {
                        PARAMS.tip = true;
                        break;
                    }

                }
            }
        }

    }

}