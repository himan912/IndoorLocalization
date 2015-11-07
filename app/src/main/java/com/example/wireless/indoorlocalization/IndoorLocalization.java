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
import android.os.AsyncTask;
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

import java.io.DataOutputStream;
import java.io.IOException;
import java.math.BigInteger;
import java.net.Socket;
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

    private final String SERVERIP = "192.168.1.3";
    //private final String SERVERIP = "203.252.157.35";
    private final int PORT = 5555;

    private static final int P0 = -40;
    private static final float MU = (float) 2;
    private static final float TIME_STAYING = (float) 2.5;

    private static final Anchor2D[] ANCHOR_2Ds = {
            new Anchor2D(1.5, 1.5, P0, MU),

            new Anchor2D(0, 0, P0, MU),
            new Anchor2D(4.5, 0, P0, MU),

            new Anchor2D(5, 0, P0, MU),

            new Anchor2D(4.5, 5, P0, MU),

            new Anchor2D(2.5, 4.2, P0, MU),

            new Anchor2D(0, 5, P0, MU),
    };

    private static final float NLOS_DETECTION_THRESHOLD = (float) 5;
    private static final int MAX_ITERATION_DEPTH = 0;

    private static final int SIZE_MARKER = 5;
    private static final int SENSOR_INIT_TIME = 100;
    // Sensor Warm-up time in ms
    private static final float MIN_MAX_INTERVAL = (float) 0.05;
    private static final float STEP_INTERVAL = (float) 0.1;
    // Minimum Intervals in Step Detection
    private static final float MAX_PEAK = (float) 2;
    private static final float MIN_PEAK = (float) 1.8;
    // Acc. Norm Threshold in Step Detection
    private static float THETA_ABS_DEG = 0;
    private static final float THETA_ABS_RAD = (float) (THETA_ABS_DEG * (Math.PI / 180));
    // The angle between Mag North and x-axis (CCW)

    private static final float W_PREV = 2, W_MAG = 1, W_GYRO = 2;
    private static final float TH_COR = (float) (5 * Math.PI / 180);
    private static final float TH_MAG = (float) (2 * Math.PI / 180); // in radian
    // Heading Estimation Parameters

    private static final float STEP_CONSTANT = (float) 0.45;
    private static Paint RED = new Paint(), BLUE = new Paint(), BLACK = new Paint();

    /*---------- Member Variables ----------*/
    private Socket socket = null;
    private DataOutputStream dos = null;
    private ConnectionTask cTask = null;

    private TextView tv_info;
    private Button btn_start;
    // UI Views

    private ImageView imgv_map;
    private Bitmap bitmap_map;
    private Canvas canvas_map;
    private PhotoViewAttacher mAttacher;
    private int screenWidth_pixel;
    private float mapWidth_x = 1, mapWidth_y = 1, mapWidth;
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

    private boolean isInitialized, isAngleCalibrated;
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
    private float t_step_s;

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
        public void onRunError(Exception e){}
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
            plotAnchors();
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
        BLACK.setTextSize(20);

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
                    resume();
                } else {
                    btn_start.setText("Start");
                    pause();
                    clearMap();
                }
            }
        });
    }

    private void pause(){
        mSensorManager.unregisterListener(mSensorEventListener);
        isRun = false;

        try {
            //cTask.closeSocket();
            //cTask = null;
            mPort.close();
            connection.close();
            mSerialIoManager.stop();
        } catch (IOException e) {
            e.printStackTrace();
        }
        anchors = new HashMap<>();
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
            mSensorManager.registerListener(mSensorEventListener, mSensorGyro, 20000);
            mSensorManager.registerListener(mSensorEventListener, mSensorLinAcc, 20000);
            mSensorManager.registerListener(mSensorEventListener, mSensorRot, 20000);
            // 20000us (20ms) sampling time.
            // http://developer.android.com/guide/topics/sensors/sensors_overview.html
            isRun = true;

            // 3. Connect to Server
            //cTask = new ConnectionTask();
            //cTask.execute(SERVERIP);
            init();
        }catch(Exception e){
            tv_info.setText(e.toString());
        }
    }

    private void init() {
        PDRLocation2D.resetLocation();
        currentLocation2D.setLocation((float) 1.5, (float) 1.5);

        angle_prev = 0;
        angle_mag = 0;
        angle_gyro = 0;
        angle_cur = 0;
        angle_mag_prev = 0;

        mapWidth_x = 1;
        mapWidth_y = 1;

        n_step = -1;
        acc_p = -10;
        t_prior_s = 0;
        t_max_peak = 0;
        t_step_s = 0;

        isInitialized = false;
        isAngleCalibrated = false;
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

        if(!isAngleCalibrated){
            THETA_ABS_DEG = orientation[0];
            isAngleCalibrated = true;
        }

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
            t_current_s = (float) (System.currentTimeMillis() - t_begin) / (float) 1000;
            //isLOS = updateLSLocation();

            if((t_current_s- t_step_s)>=TIME_STAYING){
                isLOS = updateLSLocation();
                t_step_s = t_current_s;
            }else{
                isLOS = false;
            }

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
                    if(NLOS_DETECTION_THRESHOLD == 0){
                        PDRLocation2D.setLocation(1.5f, 1.5f);
                        currentLocation2D.setLocation(PDRLocation2D);
                        n_step++;
                    }
                }
            }else{
                isInitialized = true;
                if(isLOS){
                    PDRLocation2D.setLocation(LSLocation2D);
                    currentLocation2D.setLocation(LSLocation2D);
                    //tv_info.setText("Location Calibrated\n\n");
                }else{
                    currentLocation2D.setLocation(PDRLocation2D);
                }

                plotCurrentLocation();
                previousLocation2D.setLocation(currentLocation2D);
                isLOS = false;
            }

            tv_info.setText("Current Angle: " + azimuth * (180 / Math.PI) + ", Step Length: " + steplength + "\n");
            tv_info.append("Current Location: " + currentLocation2D.getX() + "," + currentLocation2D.getY() + "\n");
            tv_info.append("Current LS Location: " + LSLocation2D.getX() + "," + LSLocation2D.getY() + "\n");
            for (int i : anchors.keySet()) {
                tv_info.append("Source: " + i + ", Raw RSS: " + anchors.get(i).getRSS() + ", avg RSS: " + anchors.get(i).getRSS() + "\n");
            }
            tv_info.append("Scattering Distance: " + scattering_distance);
        }catch (Exception e){
        }
    }

    private void updatePDRLocation() {
        /*----- Heading Estimation -----*/
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

        if (acc_norm >= MAX_PEAK && acc_norm < acc_p && (t_current_s - t_step_s) > STEP_INTERVAL && isStep == false) {
            // On Max Peak
            isStep = true;
            acc_max = acc_p;
            t_max_peak = t_prior_s;
        } else if (acc_norm <= MIN_PEAK && acc_norm > acc_p && (t_prior_s - t_max_peak) > MIN_MAX_INTERVAL && isStep == true) {
            // On Min Peak
            isStep = false;
            n_step++; // 걸음수 추가
            t_step_s = t_prior_s;

            acc_min = acc_p;

            steplength = (float) (Math.sqrt(Math.sqrt(acc_max - acc_min)));
            steplength = STEP_CONSTANT * steplength;

            PDRLocation2D.setLocation(PDRLocation2D.getX() + (float) (steplength * Math.cos(angle_cur)), PDRLocation2D.getY() + (float) (steplength * Math.sin(angle_cur)));

            /*
            for(Anchor2D anc: anchors.values()){
                anc.clearRSSTable();
            }
            */
            // Update PDR Location
        }
        acc_p = acc_norm;
    }

    private boolean updateLSLocation() {

        int n = anchors.size();
        if (n < 4) return false; // not enough anchor nodes;

        HashMap<Integer, Float> dist = new HashMap<>(n); // RSS로 구한 거리
        for (int i : anchors.keySet()) {
            dist.put(i, (float) Math.pow(10, (anchors.get(i).getP0() - anchors.get(i).getRawRSS()) / (10 * anchors.get(i).getMU())));
        }

        // RMSE approach
        int n_grid = 5;
        float stepx = (anchors.get(2).getX() - anchors.get(1).getX())/n_grid;
        float stepy = (anchors.get(6).getY() - anchors.get(1).getY())/n_grid;
        float min_scattering_distance = 999999;

        for(float x=anchors.get(6).getX();x<=anchors.get(4).getX();x+=stepx){
            for(float y=anchors.get(1).getY();y<=anchors.get(6).getY();y+=stepy){
                scattering_distance=0;
                for(int i : dist.keySet()){
                    float r_d = (float) Math.sqrt(((anchors.get(i).getX() - x) * (anchors.get(i).getX() - x)) +
                            ((anchors.get(i).getY() - y) * (anchors.get(i).getY() - y)));
                    scattering_distance += ((r_d - dist.get(i))*(r_d - dist.get(i)));
                }
                scattering_distance = (float) Math.sqrt(scattering_distance/dist.size());
                if(min_scattering_distance > scattering_distance){
                    min_scattering_distance = scattering_distance;
                    LSLocation2D.setLocation(x, y);
                }
            }
        }

        if(scattering_distance < NLOS_DETECTION_THRESHOLD){
            return true;
        }
        return false;


        /*
        // typical LS

        int[] subsets = new int[4];

        for (int i : anchors.keySet()) {
            for (int j=0; j<subsets.length; j++){
                if(subsets[j]==0){
                    subsets[j] = i;
                    break;
                }

                if(dist.get(i) <= dist.get(subsets[j])){
                    for(int k=subsets.length-1;k>j;k--){
                        subsets[k] = subsets[k-1];
                    }

                    subsets[j] = i;
                    break;
                }
            }
        }
        //tv_info.setText(subsets[0] + "," + subsets[1] + "," + subsets[2] + "," + subsets[3]);

        HashMap<Integer, Anchor2D> subanchors = new HashMap<>();
        for(int i=0;i<subsets.length;i++){
            subanchors.put(subsets[i], anchors.get(subsets[i]));
        }

        if(LSEstimation(LSLocation2D, subanchors, dist, 0)){
            isLOS = true;
            return true;
        }else{
            isLOS = false;
        }
        return false;
        */
    }

    public boolean LSEstimation(Location2D LSLocation2D, HashMap<Integer, Anchor2D> anchors, HashMap<Integer, Float> dist, int depth){
        int n = anchors.size();
        if(n < 4) return false; // first iteration termination criterion
        if(depth > MAX_ITERATION_DEPTH) return false; // 메모리 부족 현상 회피를 위해 1depth 이상의 재귀는 off

        int[] weight = new int[]{60, 25, 10, 5};
        int[][] subsets = Matrix.getSubsets(anchors.keySet());
        int div = 0;
        float x = 0, y = 0;

        ArrayList<Location2D> est = new ArrayList<Location2D>();
        double[][] mat_X = new double[n - 2][2];
        double[][] vec_b = new double[n - 2][1];

        for (int i = 0; i < subsets.length; i++) {

            for (int j = 1; j < subsets[i].length; j++) {
                mat_X[j-1][0] =
                        2 * (anchors.get(subsets[i][0]).getX() - anchors.get(subsets[i][j]).getX());
                mat_X[j-1][1] =
                        2 * (anchors.get(subsets[i][0]).getY() - anchors.get(subsets[i][j]).getY());
                vec_b[j-1][0] =
                        (Math.pow(dist.get(subsets[i][j]), 2) - Math.pow(dist.get(subsets[i][0]), 2))
                                - (Math.pow(anchors.get(subsets[i][j]).getX(), 2) - Math.pow(anchors.get(subsets[i][0]).getX(), 2))
                                - (Math.pow(anchors.get(subsets[i][j]).getY(), 2) - Math.pow(anchors.get(subsets[i][0]).getY(), 2));
            }

            /*

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
            */

            try {
                double[][] LSsample =
                        Matrix.matrixMultiplication(Matrix.matrixMultiplication(Matrix.getInverseMatrix(Matrix.matrixMultiplication(Matrix.transposeMatrix(mat_X), mat_X)), Matrix.transposeMatrix(mat_X)), vec_b);
                if (LSsample != null){
                    x += LSsample[0][0]*weight[i];
                    y += LSsample[1][0]* weight[i];
                    est.add(new Location2D(LSsample[0][0], LSsample[1][0]));
                    div += weight[i];
                }
            }catch(org.apache.commons.math3.linear.SingularMatrixException e) {
                tv_info.setText("Singularity");
                continue;
            }
        }

        LSLocation2D.setLocation(x / div, y / div);

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
        /*else{
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
        */
        return false;
    }

    private void plotCurrentLocation() {
        clearMap();
        plotAnchors();
        //cTask.notify(String.format("0,%.5f,%.5f", currentLocation2D.getX(), currentLocation2D.getY()));
        int current_x_converted = (int) (Math.round((currentLocation2D.getX() + ((1.5 * mapWidth) - mapWidth_x) / 2) * (screenWidth_pixel / (mapWidth * 1.5))));
        int current_y_converted = (int) (screenWidth_pixel - Math.round(((currentLocation2D.getY() + ((1.5 * mapWidth) - mapWidth_y) / 2) * (screenWidth_pixel / (mapWidth * 1.5)))));

        canvas_map.drawCircle(current_x_converted, current_y_converted, SIZE_MARKER, RED);
        imgv_map.invalidate();
    }

    private void clearMap(){
        canvas_map.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR);
    }

    private void plotAnchors() {
        for(int source:anchors.keySet()){
            int x_converted = (int) (Math.round((anchors.get(source).getX() + ((1.5 * mapWidth) - mapWidth_x) / 2) * (screenWidth_pixel / (mapWidth * 1.5))));
            int y_converted = (int) (screenWidth_pixel - Math.round(((anchors.get(source).getY() + ((1.5 * mapWidth) - mapWidth_y) / 2) * (screenWidth_pixel / (mapWidth * 1.5)))));

            canvas_map.drawText(source + "", x_converted, y_converted, BLACK);
            imgv_map.invalidate();

//            plotRect(anchors.get(source), BLACK);
            //cTask.notify(String.format("%d,%.5f,%.5f", source, ANCHOR_2Ds[source].getX(), ANCHOR_2Ds[source].getY()));
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

    class ConnectionTask extends AsyncTask<String, Integer, Integer> {
        private final int CONNECTED = 1;
        @Override
        protected Integer doInBackground(String... params) {
            try {
                socket = new Socket(params[0], PORT);
                dos = new DataOutputStream(socket.getOutputStream());
                publishProgress(CONNECTED);
            } catch (IOException e)
            {
                e.printStackTrace();
            }
            return null;
        }

        protected void onProgressUpdate(Integer... values) {
            if(values[0] == CONNECTED){
                tv_info.setText("Connected to " + SERVERIP);
            }
        }

        public void notify(String msg){
            if(dos != null){
                // 서버에 접속되어 있는 경우
                try {
                    dos.writeUTF(msg);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }

        public void closeSocket(){
            if(socket != null){
                try {
                    dos.close();
                    socket.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}