package com.example.wireless.indoorlocalization;

import android.app.Activity;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PorterDuff;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.util.DisplayMetrics;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

import java.io.IOException;
import java.math.BigInteger;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import uk.co.senab.photoview.PhotoViewAttacher;

public class IndoorLocalization extends Activity {
    private static final byte[] PACKET_HEADER = new BigInteger("7e4500ffff0000080089", 16).toByteArray();

    private static final int P0 = -38;
    private static final float MU = (float) 2.5;

    private static final Anchor2D[] ANCHOR_2Ds = {
            new Anchor2D(1.5, 1.5, P0, MU),

            new Anchor2D(0, 0, P0, MU),
            new Anchor2D(3, 0, P0, MU),
            new Anchor2D(6, 0, P0, MU),
            new Anchor2D(6, 3, P0, MU),

            new Anchor2D(3, 3, P0, MU),
            new Anchor2D(0, 3, P0, MU),
    };

    private static final float NLOS_DETECTION_THRESHOLD = (float) 1.4;
    private static final int MAX_ITERATION_DEPTH = 0;

    private static final int SIZE_MARKER = 5;
    private static final int SENSOR_INIT_TIME = 100;
    // Sensor Warm-up time in ms
    private static final float MIN_MAX_INTERVAL = (float) 0.1;
    private static final float STEP_INTERVAL = (float) 0.2;
    // Minimum Intervals in Step Detection
    private static final float MAX_PEAK = (float) 2.3;
    private static final float MIN_PEAK = (float) 1.8;
    // Acc. Norm Threshold in Step Detection
    private static final float THETA_ABS_DEG = 5;
    private static final float THETA_ABS_RAD = (float) (THETA_ABS_DEG * (Math.PI / 180));
    // The angle between Mag North and x-axis (CCW)

    private static final float W_PREV = 2, W_MAG = 1, W_GYRO = 2;
    private static final float TH_COR = (float) (5 * Math.PI / 180);
    private static final float TH_MAG = (float) (2 * Math.PI / 180); // in radian
    // Heading Estimation Parameters

    private static final float STEP_CONSTANT = (float) 0.43;
    private static Paint RED = new Paint(), BLUE = new Paint(), BLACK = new Paint();

    /*---------- Member Variables ----------*/
    private TextView tv_info;
    private Button btn_start;
    // UI Views

    private ImageView imgv_map;
    private Bitmap bitmap_map;
    private Canvas canvas_map;
    private PhotoViewAttacher mAttacher;
    private int screenWidth_pixel;
    private float mapWidth_x, mapWidth_y, mapWidth;
    // Map Display

    private HashMap<Integer, Anchor2D> anchors;

    // PDR
    // 멤버 선언
    private SensorManager mSensorManager;
    private SensorEventListener mSensorEventListener = new SensorEventListener() {
        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {
        }

        @Override
        public void onSensorChanged(SensorEvent event) {
            if (anchors.size() < 4) {
                tv_info.setText("Not enough Anchor nodes.. (" + anchors.size() + ")\n");
                return;
            }

            if (!isCalibrated) {
                tv_info.setText("RSS Calibrating.." + "\n");
                for (Anchor2D loc : anchors.values()) {
                    if (!loc.isCalibrated()) {
                        return;
                    }
                }
                isCalibrated = true;
            }

            switch (event.sensor.getType()) {
                case Sensor.TYPE_ROTATION_VECTOR:
                    if (System.currentTimeMillis() - t_begin > SENSOR_INIT_TIME) {
                        rot_vector = event.values.clone();
                        getOrientation();
                    }
                    break;
                case Sensor.TYPE_LINEAR_ACCELERATION:
                    data_acc = event.values.clone();
                    updateLocation();
                    break;
                case Sensor.TYPE_GYROSCOPE:
                    data_gyro = event.values.clone();
                    break;
            }
        }
    };
    private Sensor mSensorLinAcc, mSensorGyro, mSensorRot; // IMU

    private boolean isCalibrated;
    private boolean isRun = false;
    private boolean isLOS = false;

    private long t_begin;
    private float t_current_s, dt;
    private float t_prior_s;

    private float[] data_acc = null;
    private float acc_norm;
    private float[] data_gyro = null;
    private float[] rot_vector = null;

    // 데이터 변수 선언
    private Location2D PDRLocation2D = new Location2D(0,0);
    private Location2D LSLocation2D = new Location2D(0,0);
    private Location2D currentLocation2D = new Location2D(0,0);
    private Location2D previousLocation2D = new Location2D(0,0);

    private float scattering_distance;
    private int n_step;

    private float angle_prev, angle_mag, angle_mag_prev, angle_gyro, angle_cur;
    private float delta_cor, delta_mag;

    private float acc_p;            // 이전 가속도
    private float t_max_peak;    // 이전 걸음 시간
    private float t_step;

    private boolean isStep;

    private float azimuth; // 방위각
    private float pitch;    // 피치
    private float roll;  // 롤

    private float acc_max, acc_min;
    private float steplength;

    private final ExecutorService mExecutor = Executors.newSingleThreadExecutor();
    private List<UsbSerialPort> mEntries = new ArrayList<>();
    private UsbSerialPort mPort = null;
    private UsbManager mUsbManager = null;
    private UsbDeviceConnection connection;
    private SerialInputOutputManager mSerialIoManager = null;

    private final SerialInputOutputManager.Listener mListener = new SerialInputOutputManager.Listener() {
        @Override
        public void onNewData(final byte[] data) {
            IndoorLocalization.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    IndoorLocalization.this.updateReceivedData(data);
                }
            });
            // 시리얼 데이터 수신은 별도 쓰레드에서 함.
            // UI 쓰레드로 데이터를 보내 view 처리
        }

        @Override
        public void onRunError(Exception e) {

        }
    };

    private boolean updateReceivedData(byte[] data) {
        // Serial Packet Data Format (29 Bytes)
        // Header 10 Bytes: 7e 45 00 ff ff 00 00 08 00 89
        // Data 16 Bytes: Source RSS X Y (2 bytes short each)
        // Footer 3 Bytes: xx xx 7e

        if (data.length < 10) {
            return false;
        }

        byte[] header = Arrays.copyOfRange(data, 0, 10);
        try {
            if (Arrays.equals(header, PACKET_HEADER)) {
                getInfo(data);
            }
        } catch (Exception e) {
            return false;
        }

        return true;
    }

    private void getInfo(byte[] data) {
        ByteBuffer data_short = ByteBuffer.allocate(2);
        data_short.order(ByteOrder.LITTLE_ENDIAN);
        short[] info = new short[4];

        for (int i = 10, j = 0; i <= 16; i += 2, j++) {
            data_short.clear();
            data_short.put(data[i + 1]);
            data_short.put(data[i]);
            info[j] = data_short.getShort(0);
        }

        final int source = info[0];
        final int rss = info[1] - 45;
        final int x = (int) info[2];
        final int y = (int) info[3];

        if (!anchors.containsKey(source)) {
            anchors.put(source, ANCHOR_2Ds[source]);
            clearMap();

            for (Anchor2D loc1 : anchors.values()) {
                for (Anchor2D loc2 : anchors.values()) {
                    if (Math.abs(loc1.getX() - loc2.getX()) > mapWidth_x) {
                        mapWidth_x = Math.abs(loc1.getX() - loc2.getX());
                    }
                    if (Math.abs(loc1.getY() - loc2.getY()) > mapWidth_y) {
                        mapWidth_y = Math.abs(loc1.getY() - loc2.getY());
                    }
                }
            }
            if(mapWidth_x >= mapWidth_y){
                mapWidth = mapWidth_x;
            }else{
                mapWidth = mapWidth_y;
            }
        }
        anchors.get(source).addRSS(rss);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        tv_info = (TextView) findViewById(R.id.tv_info);

        anchors = new HashMap<>();

        RED.setColor(Color.RED);
        BLUE.setColor(Color.BLUE);
        BLACK.setColor(Color.BLACK);

        DisplayMetrics displaymetrics = new DisplayMetrics();
        getWindowManager().getDefaultDisplay().getMetrics(displaymetrics);
        screenWidth_pixel = displaymetrics.widthPixels;

        imgv_map = (ImageView) findViewById(R.id.map);
        bitmap_map = Bitmap.createBitmap(screenWidth_pixel, screenWidth_pixel, Bitmap.Config.ARGB_8888);
        //bitmap_map = BitmapFactory.decodeResource(this.getApplicationContext().getResources(), R.drawable.map).copy(Bitmap.Config.ARGB_8888, true);
        canvas_map = new Canvas(bitmap_map);
        imgv_map.setImageBitmap(bitmap_map);
        mAttacher = new PhotoViewAttacher(imgv_map);
        // 맵 이미지를 카피한 bitmap을 canvas에 올림. canvas에서는 좌표를 찍으면서 bitmap 변경.
        // imageview는 이 bitmap과 연결. canvas에서 좌표를 찍으면 invalidate로 bitmap 업데이트.

        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        mSensorLinAcc = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        mSensorGyro = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mSensorRot = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);

        btn_start = (Button) findViewById(R.id.btn_start);
        btn_start.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                if (!isRun) {
                    btn_start.setText("Stop");
                    init();
                    resume();
                } else {
                    btn_start.setText("Start");
                    isCalibrated = false;
                    for(Anchor2D loc: anchors.values()){
                        loc.clearRSSTable();
                    }
                    clearMap();
                    pause();
                }
            }
        });
    }

    private void pause(){
        mSensorManager.unregisterListener(mSensorEventListener);
        isRun = false;

        try {
            mPort.close();
            connection.close();
            mSerialIoManager.stop();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void resume(){
        try {
            // 1. Connect to the device
            mUsbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
            final List<UsbSerialDriver> drivers = UsbSerialProber.getDefaultProber().findAllDrivers(mUsbManager);
            // getting all the connected drivers

            if (drivers == null || drivers.size() == 0) {
                tv_info.setText("No devices attached");
                return;
            }

            mEntries.clear();
            for (final UsbSerialDriver driver : drivers) {
                final List<UsbSerialPort> ports = driver.getPorts();
                mEntries.addAll(ports);
            }
            mPort = mEntries.get(0);

            connection = mUsbManager.openDevice(mPort.getDriver().getDevice());
            try {
                mPort.open(connection);
                mPort.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
            } catch (IOException e) {
                tv_info.setText("Connection Failed");
                e.printStackTrace();
            }
            tv_info.setText("Device " + mPort.getDriver().getDevice().getDeviceName() + " Connected");

            mSerialIoManager = new SerialInputOutputManager(mPort, mListener);
            mExecutor.submit(mSerialIoManager);

            // 2. Register Sensors
            mSensorManager.registerListener(mSensorEventListener, mSensorGyro, SensorManager.SENSOR_DELAY_GAME);
            mSensorManager.registerListener(mSensorEventListener, mSensorLinAcc, SensorManager.SENSOR_DELAY_GAME);
            mSensorManager.registerListener(mSensorEventListener, mSensorRot, SensorManager.SENSOR_DELAY_GAME);
            // 20000us (20ms) sampling time.
            // http://developer.android.com/guide/topics/sensors/sensors_overview.html
            isRun = true;
            init();
        }catch(Exception e){
            tv_info.setText(e.toString());
        }
    }

    private void init() {
        PDRLocation2D.resetLocation();
        mapWidth_x = 1;
        mapWidth_y = 1;

        angle_prev = 0;
        angle_mag = 0;
        angle_gyro = 0;
        angle_cur = 0;
        angle_mag_prev = 0;

        n_step = -1;
        acc_p = -10;
        t_prior_s = 0;
        t_max_peak = 0;
        t_step = 0;

        isCalibrated = false;
        isStep = false;
        acc_max = 0;
        acc_min = 0;
        steplength = 0;

        t_begin = System.currentTimeMillis();
    }

    private void getOrientation() {
        /* rot_vector 사용하는 경우 */
        float[] rotMat = new float[9];
        float[] orientation = new float[3];

        SensorManager.getRotationMatrixFromVector(rotMat, rot_vector);
        SensorManager.getOrientation(rotMat, orientation);

        azimuth = (-1) * orientation[0]; // CW에서 CCW로 변환
        pitch = orientation[1];
        roll = orientation[2];
        // in radian
        // target orientation angle in PDR coordinate system.

        t_prior_s = (System.currentTimeMillis() - t_begin) / 1000;
        angle_cur = (azimuth + THETA_ABS_RAD);
        angle_prev = angle_cur;
        angle_mag = (azimuth + THETA_ABS_RAD);
        angle_mag_prev = angle_mag;

        // target orientation in absolute coordinate system
    }

    private void updateLocation() {
        try {
            isLOS = updateLSLocation();
            if (data_acc != null && data_gyro != null) {
                updatePDRLocation();
            }

            if (isFirstStep()) {
                if(isLOS){
                    PDRLocation2D.setLocation(LSLocation2D);
                    currentLocation2D.setLocation(LSLocation2D);

                    plotCurrentLocation();
                    previousLocation2D.setLocation(currentLocation2D);
                    n_step++;
                }
                else{
                    tv_info.setText("Locating Initial Location...");
                }
            }else{
                if(isLOS){
                    PDRLocation2D.setLocation(LSLocation2D);
                    currentLocation2D.setLocation(LSLocation2D);
                }else{
                    currentLocation2D.setLocation(PDRLocation2D);
                }

                plotCurrentLocation();
                previousLocation2D.setLocation(currentLocation2D);
            }
        }catch (Exception e){
            tv_info.setText("[UpdateLocation()]" + e.toString());
        }
    }

    private void updatePDRLocation() {
        /*----- Heading Estimation -----*/
        tv_info.setText("");
        t_current_s = (float) (System.currentTimeMillis() - t_begin) / (float) 1000;
        dt = t_current_s - t_prior_s;
        t_prior_s = t_current_s;

        angle_gyro = (angle_prev + (float) (dt * Math.round(data_gyro[2] * 100) / 100));
        // noise reduced by rounding up
        angle_mag_prev = angle_mag;
        angle_mag = (THETA_ABS_RAD + azimuth);
        // all angles in radian

        delta_mag = (float) (Math.PI - (Math.abs((angle_mag - angle_mag_prev) - Math.PI)));
        delta_cor = (float) (Math.PI - (Math.abs((angle_mag - angle_gyro) - Math.PI)));

        if (delta_cor <= TH_COR && delta_mag <= TH_MAG)
            angle_cur = ((W_PREV * angle_prev) + (W_MAG * angle_mag) + (W_GYRO * angle_mag)) / (W_PREV + W_MAG + W_GYRO);
        else if (delta_cor <= TH_COR && delta_mag > TH_MAG)
            angle_cur = ((W_MAG * angle_mag) + (W_GYRO * angle_mag)) / (W_MAG + W_GYRO);
        else if (delta_cor > TH_COR && delta_mag <= TH_MAG)
            angle_cur = angle_prev;
        else if (delta_cor > TH_COR && delta_mag > TH_MAG)
            angle_cur = ((W_PREV * angle_prev) + (W_GYRO * angle_gyro)) / (W_PREV + W_GYRO);

        angle_prev = angle_cur;
            /*----- Step Detection, Step Length Estimation, and PDR Location Update -----*/

        acc_norm = (float) Math.sqrt(data_acc[0] * data_acc[0] + data_acc[1] * data_acc[1] + data_acc[2] * data_acc[2]);
        steplength = 0;

        if (acc_norm >= MAX_PEAK && acc_norm < acc_p && (t_current_s - t_step) > STEP_INTERVAL && isStep == false) {
            // On Max Peak
            isStep = true;
            acc_max = acc_p;
            t_max_peak = t_prior_s;
        } else if (acc_norm <= MIN_PEAK && acc_norm > acc_p && (t_prior_s - t_max_peak) > MIN_MAX_INTERVAL && isStep == true) {
            // On Min Peak
            isStep = false;
            n_step++; // 걸음수 추가
            t_step = t_prior_s;

            acc_min = acc_p;

            steplength = (float) (Math.sqrt(Math.sqrt(acc_max - acc_min)));
            steplength = STEP_CONSTANT * steplength;

            PDRLocation2D.setLocation(PDRLocation2D.getX() + (float) (steplength * Math.cos(angle_cur)), PDRLocation2D.getY() + (float) (steplength * Math.sin(angle_cur)));
            // Update PDR Location
        }
        acc_p = acc_norm;

        tv_info.append("Current Location: " + currentLocation2D.getX() + "," + currentLocation2D.getY() + "\n");
        tv_info.append("Current Angle: " + azimuth * (180 / Math.PI) + ", Step Length: " + steplength + "\n");
        for (int i : anchors.keySet()) {
            tv_info.append("Source: " + i + ", Raw RSS: " + anchors.get(i).getRawRSS() + ", avg RSS: " + anchors.get(i).getRSS() + "\n");
        }
        tv_info.append("Scattering Distance: " + scattering_distance);
    }

    private boolean updateLSLocation() {
        int n = anchors.size();
        if (n < 4) return false; // not enough anchor nodes;

        HashMap<Integer, Float> dist = new HashMap<>(n);
        for (int i : anchors.keySet()) {
            dist.put(i, (float) Math.pow(10, (anchors.get(i).getP0() - anchors.get(i).getRSS()) / (10 * anchors.get(i).getMU())));
        }

        if(LSEstimation(LSLocation2D, anchors, dist, 0)){
            isLOS = true;
            return true;
        }else{
            isLOS = false;
        }
        return false;
    }

    public boolean LSEstimation(Location2D LSLocation2D, HashMap<Integer, Anchor2D> anchors, HashMap<Integer, Float> dist, int depth){
        int n = anchors.size();
        if(n < 4) return false; // first iteration termination criterion
        if(depth > MAX_ITERATION_DEPTH) return false; // 메모리 부족 현상 회피를 위해 1depth 이상의 재귀는 off

        int[][] subsets = Matrix.getSubsets(anchors.keySet());

        ArrayList<Location2D> est = new ArrayList<Location2D>();
        try {
            double[][] mat_X = new double[n - 2][2];
            double[][] vec_b = new double[n - 2][1];

            for (int i = 0; i < subsets.length; i++) {
                for (int j = 0; j < subsets[i].length - 1; j++) {
                    mat_X[j][0] =
                            2 * (anchors.get(subsets[i][n - 2]).getX() - anchors.get(subsets[i][j]).getX());
                    mat_X[j][1] =
                            2 * (anchors.get(subsets[i][n - 2]).getY() - anchors.get(subsets[i][j]).getY());
                    vec_b[j][0] =
                            (Math.pow(dist.get(subsets[i][j]), 2) - Math.pow(dist.get(subsets[i][n - 2]), 2))
                                    - (Math.pow(anchors.get(subsets[i][j]).getX(), 2) - Math.pow(anchors.get(subsets[i][n - 2]).getX(), 2))
                                    - (Math.pow(anchors.get(subsets[i][j]).getY(), 2) - Math.pow(anchors.get(subsets[i][n - 2]).getY(), 2));
                }

                double[][] LSsample =
                        Matrix.matrixMultiplication(Matrix.matrixMultiplication(Matrix.getInverseMatrix(Matrix.matrixMultiplication(Matrix.transposeMatrix(mat_X), mat_X)), Matrix.transposeMatrix(mat_X)), vec_b);
                if (LSsample != null){
                    est.add(new Location2D(LSsample[0][0], LSsample[1][0]));
                }
            }
        } catch (org.apache.commons.math3.linear.SingularMatrixException e) {
            return false;
        }

        float x = 0, y = 0;
        for (Location2D loc : est) {
            x += loc.getX();
            y += loc.getY();
        }
        LSLocation2D.setLocation(x / est.size(), y / est.size());

        float d;
        scattering_distance = -1;

        for (int i = 0; i < est.size() - 1; i++) {
            for (int j = i + 1; j < est.size(); j++) {
                d = (est.get(i).getX() - est.get(j).getX()) * (est.get(i).getX() - est.get(j).getX()) +
                        (est.get(i).getY() - est.get(j).getY()) * (est.get(i).getY() - est.get(j).getY());
                if (Math.sqrt(d) > scattering_distance) {
                    scattering_distance = (float) Math.sqrt(d);
                }
            }
        }

        if(scattering_distance <= NLOS_DETECTION_THRESHOLD){
            return true;
            // 한번 해보고 Threshold 만족하는 경우 바로 리턴.
        }
        else{
            // Full iteration NLOS checking
            // Threshold 만족 못하면 next depth iteration 돌입하는 재귀
            // Memory 부족 현상 발생할 수 있으므로 Depth 제한 필요.
            for(int i=0;i<subsets.length;i++){
                HashMap<Integer, Anchor2D> subanchors = new HashMap<>();
                for(int j=0;j<subsets[i].length;j++){
                    subanchors.put(subsets[i][j], anchors.get(subsets[i][j]));
                }
                if(LSEstimation(LSLocation2D, subanchors, dist, depth+1))
                    return true;
            }
        }
        return false;
    }

    private void plotCurrentLocation() {
        clearMap();
        int current_x_converted = (int) (Math.round((currentLocation2D.getX() + ((1.5 * mapWidth) - mapWidth_x) / 2) * (screenWidth_pixel / (mapWidth * 1.5))));
        int current_y_converted = (int) (screenWidth_pixel - Math.round(((currentLocation2D.getY() + ((1.5 * mapWidth) - mapWidth_y) / 2) * (screenWidth_pixel / (mapWidth * 1.5)))));

        canvas_map.drawCircle(current_x_converted, current_y_converted, SIZE_MARKER, RED);
        imgv_map.invalidate();
    }

    private void clearMap(){
        canvas_map.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR);
        plotAnchors();
    }

    private void plotAnchors() {
        for (Anchor2D loc : anchors.values()) {
            plotRect(loc, BLACK);
        }
        imgv_map.invalidate();
    }

    private void plotRect(Location2D loc, Paint paint) {
        int x_converted = (int) (Math.round((loc.getX() + ((1.5 * mapWidth) - mapWidth_x) / 2) * (screenWidth_pixel / (mapWidth * 1.5))));
        int y_converted = (int) (screenWidth_pixel - Math.round(((loc.getY() + ((1.5 * mapWidth) - mapWidth_y) / 2) * (screenWidth_pixel / (mapWidth * 1.5)))));
        canvas_map.drawRect(
                (x_converted - SIZE_MARKER), (y_converted - SIZE_MARKER),
                (x_converted + SIZE_MARKER), (y_converted + SIZE_MARKER), paint);

        imgv_map.invalidate();
    }

    private boolean isFirstStep() {
        if (n_step == -1) return true;
        else return false;
    }
}