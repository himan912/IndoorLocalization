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

import static com.example.wireless.indoorlocalization.Matrix.getInverseMatrix;
import static com.example.wireless.indoorlocalization.Matrix.matrixMultiplication;
import static com.example.wireless.indoorlocalization.Matrix.transposeMatrix;

public class IndoorLocalization extends Activity {

    /*---------- Fixed Parameters ----------*/
    private static final byte[] PACKET_HEADER = new BigInteger("7e4500ffff0000080089", 16).toByteArray();
    private static final Location[] ANCHORS = {
            new Location(0, 0),
            new Location(0, 0),
            new Location(3, 0),
            new Location(3, 3),
            new Location(0, 3)
    };

    private boolean isCalibrated = false;

    private static final int N_GRID = 10;

    private float anchor_dist_x = 1, anchor_dist_y = 1;

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
    private static final int PDR_DURATION = 6;
    private static final float NLOS_DETECTION_THRESHOLD = (float) 1.1;
    // PDR Parameters

    private static final int P0 = -43;
    private static final float MU = (float) 2.5;
    // Path Loss Model

    private static final float W_PREV = 2, W_MAG = 1, W_GYRO = 2;
    private static final float TH_COR = (float) (5 * Math.PI / 180);
    private static final float TH_MAG = (float) (2 * Math.PI / 180); // in radian
    // Heading Estimation Parameters

    private static final float STEP_CONSTANT = (float) 0.43;
    private static Paint RED = new Paint(), BLUE = new Paint(), BLACK = new Paint(), LINE = new Paint();

    /*---------- Member Variables ----------*/
    private TextView tv_info;
    private Button btn_start;
    // UI Views

    private int screen_width;
    private ImageView imgv_map;
    private Bitmap bitmap_map;
    private Canvas canvas_map;
    private PhotoViewAttacher mAttacher;
    // Map Display

    private boolean isSelected = false;

    private HashMap<Integer, Location> anchors;
    //private HashMap<Integer, Integer> rss_table;

    // PDR
    // 멤버 선언
    private SensorManager mSensorManager;
    private SensorEventListener mSensorEventListener;
    private Sensor mSensorLinAcc, mSensorGyro, mSensorRot; // IMU

    private boolean isRun = false, isNewStep = true;

    private long t_begin;
    private float t_current_s, dt;
    private float t_prior_s;

    private float[] data_acc = null;
    private float acc_norm;
    private float[] data_gyro = null;
    private float[] rot_vector;

    // 데이터 변수 선언
    private Location PDRLocation = new Location(0, 0);
    private Location LSLocation = new Location(0, 0);
    private Location currentLocation = new Location(0, 0);
    private Location previousLocation = new Location(0, 0);

    private float max_d = 0;

    private int n_step = -1;                // 걸음수

    private float angle_prev = 0, angle_mag = 0, angle_mag_prev = 0, angle_gyro = 0, angle_cur = 0;
    private float delta_cor, delta_mag;

    private float acc_p = -10;            // 이전 가속도
    private float t_max_peak = 0;    // 이전 걸음 시간
    private float t_step = 0;

    private boolean isStep = false;
    private boolean needPlot = false;

    private float azimuth; // 방위각
    private float pitch;    // 피치
    private float roll;  // 롤

    private float acc_max, acc_min = 0;
    private float steplength = 0;

    private final ExecutorService mExecutor = Executors.newSingleThreadExecutor();
    private List<UsbSerialPort> mEntries = new ArrayList<>();
    private UsbSerialPort mPort = null;
    private UsbManager mUsbManager = null;
    private UsbDeviceConnection connection;
    private SerialInputOutputManager mSerialIoManager = null;

    private final SerialInputOutputManager.Listener mListener = new SerialInputOutputManager.Listener() {
        @Override
        public void onRunError(Exception e) {
        }

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
            anchors.put(source, ANCHORS[source]);

            for (Location loc1 : anchors.values()) {
                for (Location loc2 : anchors.values()) {
                    if (Math.abs(loc1.getX() - loc2.getX()) > anchor_dist_x) {
                        anchor_dist_x = Math.abs(loc1.getX() - loc2.getX());
                    }
                    if (Math.abs(loc1.getY() - loc2.getY()) > anchor_dist_y) {
                        anchor_dist_y = Math.abs(loc1.getY() - loc2.getY());
                    }
                }
            }

            //anchors.put(source, new Location(x, y));
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

        LINE.setColor(Color.BLUE);
        LINE.setAlpha(100);
        LINE.setStyle(Paint.Style.STROKE);
        LINE.setStrokeWidth(2);

        DisplayMetrics displaymetrics = new DisplayMetrics();
        getWindowManager().getDefaultDisplay().getMetrics(displaymetrics);
        screen_width = displaymetrics.widthPixels;

        imgv_map = (ImageView) findViewById(R.id.map);
        bitmap_map = Bitmap.createBitmap(screen_width, screen_width, Bitmap.Config.ARGB_8888);
        //bitmap_map = BitmapFactory.decodeResource(this.getApplicationContext().getResources(), R.drawable.map).copy(Bitmap.Config.ARGB_8888, true);
        canvas_map = new Canvas(bitmap_map);
        imgv_map.setImageBitmap(bitmap_map);
        mAttacher = new PhotoViewAttacher(imgv_map);
        // 맵 이미지를 카피한 bitmap을 canvas에 올림. canvas에서는 좌표를 찍으면서 bitmap 변경.
        // imageview는 이 bitmap과 연결. canvas에서 좌표를 찍으면 invalidate로 bitmap 업데이트.

        btn_start = (Button) findViewById(R.id.btn_start);
        btn_start.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                if (!isRun) {
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

                    btn_start.setText("Stop");
                    Init_PDR();
                    run();
                } else {
                    btn_start.setText("Start");
                    mSensorManager.unregisterListener(mSensorEventListener);
                    try {
                        mPort.close();
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                    connection.close();
                    isRun = false;
                    isCalibrated = false;
                    plotAnchors();
                }
            }
        });
    }

    protected void onResume() {
        super.onResume();
        //////////////////////
        // 센서 초기화
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        mSensorLinAcc = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        mSensorGyro = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mSensorRot = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);

        mSensorEventListener = new SensorEventListener() {
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
                    plotAnchors();
                    for (Location loc : anchors.values()) {
                        if (!loc.isCalibrated()) {
                            tv_info.setText("Calibrating.." + "\n");
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
                        if (!isNewStep) {
                            data_acc = event.values.clone();
                            updatePosition();
                        }
                        break;
                    case Sensor.TYPE_GYROSCOPE:
                        if (!isNewStep) {
                            data_gyro = event.values.clone();
                        }
                        break;
                }
            }
        };
    }

    @Override
    protected void onPause() {
        super.onPause();
        mSensorManager.unregisterListener(mSensorEventListener);
        isRun = false;
    }

    private void run() {
        mSensorManager.registerListener(mSensorEventListener, mSensorLinAcc, SensorManager.SENSOR_DELAY_GAME);
        mSensorManager.registerListener(mSensorEventListener, mSensorGyro, SensorManager.SENSOR_DELAY_GAME);
        mSensorManager.registerListener(mSensorEventListener, mSensorRot, SensorManager.SENSOR_DELAY_GAME);
        // 20000us (20ms) sampling time.
        // http://developer.android.com/guide/topics/sensors/sensors_overview.html
        isRun = true;
    }

    private void Init_PDR() {
        PDRLocation.resetLocation();
        //PDRLocation.setLocation(2.5, 2.5);
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

        isNewStep = true; // 초기 방향측정 기준
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

        if (isNewStep == true) {
            t_prior_s = (System.currentTimeMillis() - t_begin) / 1000;
            angle_cur = (azimuth + THETA_ABS_RAD);
            angle_prev = angle_cur;
            angle_mag = (azimuth + THETA_ABS_RAD);
            angle_mag_prev = angle_mag;

            // target orientation in absolute coordinate system
            isNewStep = false;
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

            PDRLocation.setLocation(PDRLocation.getX() + (float) (steplength * Math.cos(angle_cur)), PDRLocation.getY() + (float) (steplength * Math.sin(angle_cur)));
            needPlot = true;
            // Update PDR Location
        }
        acc_p = acc_norm;

        tv_info.append("Current Location: " + currentLocation.getX() + "," + currentLocation.getY() + "\n");
        tv_info.append("LS Location: " + LSLocation.getX() + "," + LSLocation.getY() + "\n");
        tv_info.append("Current Angle: " + azimuth * (180 / Math.PI) + "\n");
        for (int i : anchors.keySet()) {
            tv_info.append("Source: " + i + ", RSS: " + anchors.get(i).getRawRSS() + "\n");
        }
        tv_info.append("max_d: " + max_d);
    }

    private float updateLSLocation() {
        ArrayList<Location> est = new ArrayList<Location>();
        int n = anchors.size();

        if (n < 4) {
            return -1;
        }

        HashMap<Integer, Float> dist = new HashMap<>(n);

        try {
            tv_info.setText("");
            for (int i : anchors.keySet()) {
                dist.put(i, (float) Math.pow(10, (P0 - anchors.get(i).getRSS()) / (10 * MU)));
                //tv_info.append("rss: " + anchors.get(i).getRSS() +"d: " + dist.get(i) + "\n");
            }

            double[][] mat_X = new double[n - 2][2];
            double[][] vec_b = new double[n - 2][1];

            int[][] subsets = Matrix.getSubsets(n);

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

                double[][] LSsample = matrixMultiplication(matrixMultiplication(getInverseMatrix(matrixMultiplication(transposeMatrix(mat_X), mat_X)), transposeMatrix(mat_X)), vec_b);
                est.add(new Location(LSsample[0][0], LSsample[1][0]));
                // 역행렬 문제로 인해 4개 Anchor밖에 지원 못함.
            }
        } catch (Exception e) {
            tv_info.setText(e.toString());
        }

        float x = 0, y = 0;
        for (Location loc : est) {
            x += loc.getX();
            y += loc.getY();
        }
        LSLocation.setLocation(x / est.size(), y / est.size());

        float d, max_d = -1;

        for (int i = 0; i < est.size() - 1; i++) {
            for (int j = i + 1; j < est.size(); j++) {
                d = (est.get(i).getX() - est.get(j).getX()) * (est.get(i).getX() - est.get(j).getX()) +
                        (est.get(i).getY() - est.get(j).getY()) * (est.get(i).getY() - est.get(j).getY());
                if (Math.sqrt(d) > max_d) {
                    max_d = (float) Math.sqrt(d);
                }
            }
        }

        needPlot = true;
        return max_d;
    }

    private boolean isLOS(float max_d) {
        if (max_d >= 0 && max_d < NLOS_DETECTION_THRESHOLD)
            return true;
        else
            return false;
    }

    private void updatePosition() {
        if (data_acc != null && data_gyro != null) {
            //max_d = updateLSLocation();
            updatePDRLocation();
        }

        if (isFirstStep()) {
            // 가장 처음에
            max_d = updateLSLocation();
            PDRLocation.setLocation(LSLocation);
            currentLocation.setLocation(LSLocation);

            if (needPlot) {
                plotCurrentLocation();
                previousLocation.setLocation(currentLocation);
                n_step++;
            }
        }

        if (!isFirstStep() && needPlot) {
            if (isPDRDurationExpired()) {
                // PDR duration expired 되어 결정할 때
                max_d = updateLSLocation();
                if (isLOS(max_d)) {
                    PDRLocation.setLocation(LSLocation);
                    currentLocation.setLocation(LSLocation);
                } else {
                    currentLocation.setLocation(PDRLocation);
                }
                isNewStep = true;
            } else if (!isPDRDurationExpired()) {
                // 그냥 PDR만 사용할 때
                currentLocation.setLocation(PDRLocation);
                isSelected = false;
            }

            plotCurrentLocation();
            previousLocation.setLocation(currentLocation);
        }
    }

    private void plotCurrentLocation() {
        int previous_x_converted = Math.round(previousLocation.getX() * ((screen_width / 2) / anchor_dist_x)) + (screen_width / 4);
        int previous_y_converted = screen_width - Math.round(previousLocation.getY() * ((screen_width / 2) / anchor_dist_y)) - (screen_width / 4);
        canvas_map.drawCircle(previous_x_converted, previous_y_converted, SIZE_MARKER, BLUE);

        int current_x_converted = Math.round(currentLocation.getX() * ((screen_width / 2) / anchor_dist_x)) + (screen_width / 4);
        int current_y_converted = screen_width - Math.round(currentLocation.getY() * ((screen_width / 2) / anchor_dist_y)) - (screen_width / 4);
        canvas_map.drawCircle(current_x_converted, current_y_converted, SIZE_MARKER, RED);

        if (!isFirstStep()) {
            canvas_map.drawLine(previous_x_converted, previous_y_converted, current_x_converted, current_y_converted, LINE);
        }
        needPlot = false;
        imgv_map.invalidate();
    }

    private void plotAnchors() {
        canvas_map.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR);
        for (Location loc : anchors.values()) {
            plotRect(loc, BLACK);
        }
        imgv_map.invalidate();
    }

    private void plotRect(Location loc, Paint paint) {
        int x_converted = Math.round(loc.getX() * ((screen_width / 2) / anchor_dist_x));
        int y_converted = screen_width - Math.round(loc.getY() * ((screen_width / 2) / anchor_dist_y));

        canvas_map.drawRect(
                ((screen_width / 4) + x_converted - SIZE_MARKER), (y_converted - (screen_width / 4) - SIZE_MARKER),
                ((screen_width / 4) + x_converted + SIZE_MARKER), (y_converted - (screen_width / 4) + SIZE_MARKER), paint);

        imgv_map.invalidate();
    }

    private boolean isFirstStep() {
        if (n_step == -1) return true;
        else return false;
    }

    private boolean isPDRDurationExpired() {
        if (n_step % PDR_DURATION == 0 && !isSelected) {
            isSelected = true;
            return true;
        } else return false;
    }
}
