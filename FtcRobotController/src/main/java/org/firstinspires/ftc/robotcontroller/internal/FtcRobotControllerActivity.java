/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.robotcontroller.internal;

import android.app.ActionBar;
import android.app.Activity;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.content.res.Configuration;
import android.content.res.Resources;
import android.graphics.Camera;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.net.wifi.WifiManager;
import android.os.Build;
import android.os.Bundle;
import android.os.IBinder;
import android.preference.PreferenceManager;
import android.support.annotation.NonNull;
import android.support.annotation.Nullable;
import android.support.annotation.StringRes;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.SurfaceView;
import android.view.View;
import android.webkit.WebView;
import android.widget.ImageButton;
import android.widget.LinearLayout;
import android.widget.PopupMenu;
import android.widget.TextView;

import com.google.blocks.ftcrobotcontroller.BlocksActivity;
import com.google.blocks.ftcrobotcontroller.ProgrammingModeActivity;
import com.google.blocks.ftcrobotcontroller.ProgrammingModeControllerImpl;
import com.google.blocks.ftcrobotcontroller.ProgrammingWebHandlers;
import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;
import com.qualcomm.ftccommon.ClassManagerFactory;
import com.qualcomm.ftccommon.FtcAboutActivity;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.ftccommon.FtcEventLoopIdle;
import com.qualcomm.ftccommon.FtcRobotControllerService;
import com.qualcomm.ftccommon.FtcRobotControllerService.FtcRobotControllerBinder;
import com.qualcomm.ftccommon.FtcRobotControllerSettingsActivity;
import com.qualcomm.ftccommon.LaunchActivityConstantsList;
import com.qualcomm.ftccommon.LaunchActivityConstantsList.RequestCode;
import com.qualcomm.ftccommon.ProgrammingModeController;
import com.qualcomm.ftccommon.Restarter;
import com.qualcomm.ftccommon.UpdateUI;
import com.qualcomm.ftccommon.configuration.EditParameters;
import com.qualcomm.ftccommon.configuration.FtcLoadFileActivity;
import com.qualcomm.ftccommon.configuration.RobotConfigFile;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.HardwareFactory;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.eventloop.opmode.FtcRobotControllerServiceState;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.util.Device;
import com.qualcomm.robotcore.util.Dimmer;
import com.qualcomm.robotcore.util.ImmersiveMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.wifi.NetworkConnection;
import com.qualcomm.robotcore.wifi.NetworkConnectionFactory;
import com.qualcomm.robotcore.wifi.NetworkType;
import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.ftccommon.external.SoundPlayingRobotMonitor;
import org.firstinspires.ftc.ftccommon.internal.FtcRobotControllerWatchdogService;
import org.firstinspires.ftc.ftccommon.internal.ProgramAndManageActivity;
import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;
import org.firstinspires.ftc.robotcore.internal.hardware.DragonboardLynxDragonboardIsPresentPin;
import org.firstinspires.ftc.robotcore.internal.network.DeviceNameManagerFactory;
import org.firstinspires.ftc.robotcore.internal.network.PreferenceRemoterRC;
import org.firstinspires.ftc.robotcore.internal.network.StartResult;
import org.firstinspires.ftc.robotcore.internal.network.WifiMuteEvent;
import org.firstinspires.ftc.robotcore.internal.network.WifiMuteStateMachine;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.robotcore.internal.system.PreferencesHelper;
import org.firstinspires.ftc.robotcore.internal.system.ServiceController;
import org.firstinspires.ftc.robotcore.internal.ui.LocalByRefIntentExtraHolder;
import org.firstinspires.ftc.robotcore.internal.ui.ThemedActivity;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;
import org.firstinspires.ftc.robotcore.internal.webserver.RobotControllerWebInfo;
import org.firstinspires.ftc.robotcore.internal.webserver.WebServer;
import org.firstinspires.inspection.RcInspectionActivity;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

@SuppressWarnings("WeakerAccess")
public class FtcRobotControllerActivity extends Activity implements CameraBridgeViewBase.CvCameraViewListener2
{
  public static final String TAG = "RCActivity";
  public String getTag() { return TAG; }

    public static Boolean shouldProcessImage = true;
    public static Pair<ArrayList<MatOfPoint>, Mat> imageData;
    public static Size frameSize = null;
  static private CameraBridgeViewBase mOpenCvCameraView;

  static {
    if (!OpenCVLoader.initDebug()) {
      Log.d("ERROR", "It didn't work");
    }
  }
  private static final int REQUEST_CONFIG_WIFI_CHANNEL = 1;
  private static final int NUM_GAMEPADS = 2;

  protected WifiManager.WifiLock wifiLock;
  protected RobotConfigFileManager cfgFileMgr;

  protected ProgrammingWebHandlers programmingWebHandlers;
  protected ProgrammingModeController programmingModeController;

  protected UpdateUI.Callback callback;
  protected Context context;
  protected Utility utility;
  protected StartResult prefRemoterStartResult = new StartResult();
  protected StartResult deviceNameStartResult = new StartResult();
  protected PreferencesHelper preferencesHelper;
  protected final SharedPreferencesListener sharedPreferencesListener = new SharedPreferencesListener();

  protected ImageButton buttonMenu;
  protected TextView textDeviceName;
  protected TextView textNetworkConnectionStatus;
  protected TextView textRobotStatus;
  protected TextView[] textGamepad = new TextView[NUM_GAMEPADS];
  protected TextView textOpMode;
  protected TextView textErrorMessage;
  protected ImmersiveMode immersion;

  protected UpdateUI updateUI;
  protected Dimmer dimmer;
  protected LinearLayout entireScreenLayout;

  protected FtcRobotControllerService controllerService;
  protected NetworkType networkType;

  protected FtcEventLoop eventLoop;
  protected Queue<UsbDevice> receivedUsbAttachmentNotifications;

  //// the beginning of object recognition tutorial stuff for part not in a function  (the SVM stuff is mine)/////
//  static {
//      if (!OpenCVLoader.initDebug()){
//          Log.d("ERROR", "Couldn't load openCV");
//      }
//    }
//    static{ System.loadLibrary("opencv_java3"); } // might break this if commented

  private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
    @Override
    public void onManagerConnected(int status) {
      switch (status) {
        case LoaderCallbackInterface.SUCCESS: {
          Log.i(TAG, "OpenCV loaded successfully");
          mOpenCvCameraView.enableView();
//          try {
//            initializeOpenCVDependencies();
//          } catch (IOException e) {
//            e.printStackTrace();
//            Log.d("ERROR", "Can't load image");
//          }
        }
        break;
        default: {
          super.onManagerConnected(status);
        }
        break;
      }
    }
  };

  static {
    if (!OpenCVLoader.initDebug()) {
      Log.d("ERROR", "It didn't work");
    }
  }


  protected WifiMuteStateMachine wifiMuteStateMachine;
  protected MotionDetection motionDetection;

  protected class RobotRestarter implements Restarter {

    public void requestRestart() {
      requestRobotRestart();
    }

  }
  @Override
  public void onCameraViewStarted(int width, int height) {
    frameSize = new Size(width, height); // because rotated
  }

  @Override
  public void onCameraViewStopped() {
  }

  private Mat recognizeWithContours(Mat inputFrame) {
    inputFrame.convertTo(inputFrame, 0);//CvType.CV_16U
    Core.rotate(inputFrame, inputFrame, Core.ROTATE_90_CLOCKWISE);
//    Imgproc.blur(inputFrame, inputFrame, new Size(5,5));
    Mat hsvInputFrame = new Mat();
    // adding this purple helps in incandescant light
    // it negatively affects accuracy in other types of light
//    Mat purple = new Mat(new Size(inputFrame.cols(), inputFrame.rows()), inputFrame.type(), new Scalar(25,0,25));
//    Core.add(inputFrame, purple, inputFrame);
    Imgproc.cvtColor(inputFrame, hsvInputFrame, Imgproc.COLOR_RGB2HSV);
//    hsvInputFrame
//    Imgproc.cvtColor(inputFrame, inputFrame, Imgproc.COLOR_RGB2GRAY);
//    Imgproc.Canny(inputFrame, inputFrame, 50, 100);
//    Imgproc.adaptiveThreshold(inputFrame, inputFrame, 125, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY_INV, 5, 1.5);

//    inputFrame.convertTo(inputFrame, CvType.CV_8UC1);
    List<MatOfPoint> contours = new ArrayList<>();
//    Imgproc.findContours(inputFrame, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//    for (int i = 0; i < contours.size(); i++) {
//      Imgproc.drawContours(inputFrame, contours, i, new Scalar(255, 255, 255), -1);
//    }
    Mat colorToFilter = new Mat();
    Mat blues = new Mat();
    Mat greens = new Mat();
    Mat redsPart1 = new Mat();
    Mat redsPart2 = new Mat();
    Core.inRange(hsvInputFrame, new Scalar(9, 50,50), new Scalar(38, 255,255), colorToFilter);
//
//    Core.inRange(hsvInputFrame, new Scalar(0, 0,0), new Scalar(180, 50,74), colorToFilter);

    Mat kernal = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(2,2));

    Imgproc.blur(colorToFilter, colorToFilter, new Size(10,10));
    Imgproc.erode(colorToFilter, colorToFilter, kernal);
    Imgproc.dilate(colorToFilter,colorToFilter,kernal);
//    for (int col = 0; col < colorToFilter.cols();col++){
//      for (int row = 0; row < colorToFilter.rows();row++){
//        if (colorToFilter.get(row,col)[0] < 100) {
//          System.out.println("changedValue");
//          System.out.println("value1 " + colorToFilter.get(row,col)[0]);
//          colorToFilter.put(row,col, 0.0);
//        }
//      }
//    }
    imageData = new Pair<>(new ArrayList<MatOfPoint>(contours), hsvInputFrame);

    Imgproc.findContours(colorToFilter, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

//

//
//    Imgproc.dilate(reds, reds, kernal);
    for (int i=0; i < contours.size();i++){
//      System.out.println("contour" + i);
      Rect rect = Imgproc.boundingRect(contours.get(i));
      Imgproc.rectangle(colorToFilter, new Point(rect.x, rect.y), new Point(rect.x+rect.width, rect.y+rect.height), new Scalar(255,255,255));
      Imgproc.drawContours(colorToFilter, contours, i, new Scalar(255,255,255), 6);
    }
    Imgproc.resize(colorToFilter, colorToFilter, inputFrame.t().size());

    return colorToFilter;
  }

  @Override
  public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
//    System.out.println("cols"+inputFrame.rgba().cols());
//    System.out.println("rows"+inputFrame.rgba().rows());
    if (shouldProcessImage) {

//      Pair<Mat, Boolean> processedImg = recognizeWithKeypoints(inputFrame.rgba());
      Mat processedImage = recognizeWithContours(inputFrame.rgba());
      shouldProcessImage = false;
      return processedImage;
    }else{
//      return recognizeWithContours(inputFrame.rgba());
      return inputFrame.rgba();
    }
  }

  protected ServiceConnection connection = new ServiceConnection() {
    @Override
    public void onServiceConnected(ComponentName name, IBinder service) {
      FtcRobotControllerBinder binder = (FtcRobotControllerBinder) service;
      onServiceBind(binder.getService());
    }

    @Override
    public void onServiceDisconnected(ComponentName name) {
      RobotLog.vv(FtcRobotControllerService.TAG, "%s.controllerService=null", TAG);
      controllerService = null;
    }
  };

  @Override
  protected void onNewIntent(Intent intent) {
    super.onNewIntent(intent);

    if (UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(intent.getAction())) {
      UsbDevice usbDevice = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
      RobotLog.vv(TAG, "ACTION_USB_DEVICE_ATTACHED: %s", usbDevice.getDeviceName());

      if (usbDevice != null) {  // paranoia
        // We might get attachment notifications before the event loop is set up, so
        // we hold on to them and pass them along only when we're good and ready.
        if (receivedUsbAttachmentNotifications != null) { // *total* paranoia
          receivedUsbAttachmentNotifications.add(usbDevice);
          passReceivedUsbAttachmentsToEventLoop();
        }
      }
    }
  }

  protected void passReceivedUsbAttachmentsToEventLoop() {
    if (this.eventLoop != null) {
      for (;;) {
        UsbDevice usbDevice = receivedUsbAttachmentNotifications.poll();
        if (usbDevice == null)
          break;
        this.eventLoop.onUsbDeviceAttached(usbDevice);
      }
    }
    else {
      // Paranoia: we don't want the pending list to grow without bound when we don't
      // (yet) have an event loop
      while (receivedUsbAttachmentNotifications.size() > 100) {
        receivedUsbAttachmentNotifications.poll();
      }
    }
  }

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    RobotLog.onApplicationStart();  // robustify against onCreate() following onDestroy() but using the same app instance, which apparently does happen
    RobotLog.vv(TAG, "onCreate()");
    ThemedActivity.appAppThemeToActivity(getTag(), this); // do this way instead of inherit to help AppInventor

    // Oddly, sometimes after a crash & restart the root activity will be something unexpected, like from the before crash? We don't yet understand
    RobotLog.vv(TAG, "rootActivity is of class %s", AppUtil.getInstance().getRootActivity().getClass().getSimpleName());
    Assert.assertTrue(FtcRobotControllerWatchdogService.isFtcRobotControllerActivity(AppUtil.getInstance().getRootActivity()));
    Assert.assertTrue(AppUtil.getInstance().isRobotController());

    // Quick check: should we pretend we're not here, and so allow the Lynx to operate as
    // a stand-alone USB-connected module?
    if (LynxConstants.isRevControlHub()) {
      if (LynxConstants.disableDragonboard()) {
        // Double-sure check that the Lynx Module can operate over USB, etc, then get out of Dodge
        RobotLog.vv(TAG, "disabling Dragonboard and exiting robot controller");
        DragonboardLynxDragonboardIsPresentPin.getInstance().setState(false);
        AppUtil.getInstance().finishRootActivityAndExitApp();
        }
      else {
        // Double-sure check that we can talk to the DB over the serial TTY
        DragonboardLynxDragonboardIsPresentPin.getInstance().setState(true);
      }
    }

    context = this;
    utility = new Utility(this);

    DeviceNameManagerFactory.getInstance().start(deviceNameStartResult);

    PreferenceRemoterRC.getInstance().start(prefRemoterStartResult);

    receivedUsbAttachmentNotifications = new ConcurrentLinkedQueue<UsbDevice>();
    eventLoop = null;

    setContentView(R.layout.activity_ftc_controller);

    preferencesHelper = new PreferencesHelper(TAG, context);
    preferencesHelper.writeBooleanPrefIfDifferent(context.getString(R.string.pref_rc_connected), true);
    preferencesHelper.getSharedPreferences().registerOnSharedPreferenceChangeListener(sharedPreferencesListener);

    entireScreenLayout = (LinearLayout) findViewById(R.id.entire_screen);
    buttonMenu = (ImageButton) findViewById(R.id.menu_buttons);
    buttonMenu.setOnClickListener(new View.OnClickListener() {
      @Override
      public void onClick(View v) {
        PopupMenu popupMenu = new PopupMenu(FtcRobotControllerActivity.this, v);
        popupMenu.setOnMenuItemClickListener(new PopupMenu.OnMenuItemClickListener() {
          @Override
          public boolean onMenuItemClick(MenuItem item) {
            return onOptionsItemSelected(item); // Delegate to the handler for the hardware menu button
          }
        });
        popupMenu.inflate(R.menu.ftc_robot_controller);
        popupMenu.show();
      }
    });

    BlocksOpMode.setActivityAndWebView(this, (WebView) findViewById(R.id.webViewBlocksRuntime));

    ClassManagerFactory.registerFilters();
    ClassManagerFactory.processAllClasses();
    cfgFileMgr = new RobotConfigFileManager(this);

    // Clean up 'dirty' status after a possible crash
    RobotConfigFile configFile = cfgFileMgr.getActiveConfig();
    if (configFile.isDirty()) {
      configFile.markClean();
      cfgFileMgr.setActiveConfig(false, configFile);
    }

    textDeviceName = (TextView) findViewById(R.id.textDeviceName);
    textNetworkConnectionStatus = (TextView) findViewById(R.id.textNetworkConnectionStatus);
    textRobotStatus = (TextView) findViewById(R.id.textRobotStatus);
    textOpMode = (TextView) findViewById(R.id.textOpMode);
    textErrorMessage = (TextView) findViewById(R.id.textErrorMessage);
    textGamepad[0] = (TextView) findViewById(R.id.textGamepad1);
    textGamepad[1] = (TextView) findViewById(R.id.textGamepad2);
    immersion = new ImmersiveMode(getWindow().getDecorView());
    dimmer = new Dimmer(this);
    dimmer.longBright();

    programmingWebHandlers = new ProgrammingWebHandlers();
    programmingModeController = new ProgrammingModeControllerImpl(
            this, (TextView) findViewById(R.id.textRemoteProgrammingMode), programmingWebHandlers);

    updateUI = createUpdateUI();
    callback = createUICallback(updateUI);

    PreferenceManager.setDefaultValues(this, R.xml.app_settings, false);

    WifiManager wifiManager = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);
    wifiLock = wifiManager.createWifiLock(WifiManager.WIFI_MODE_FULL_HIGH_PERF, "");

    hittingMenuButtonBrightensScreen();

    wifiLock.acquire();
    callback.networkConnectionUpdate(NetworkConnection.NetworkEvent.DISCONNECTED);
    readNetworkType();
    ServiceController.startService(FtcRobotControllerWatchdogService.class);
    bindToService();
    logPackageVersions();
    logDeviceSerialNumber();
    RobotLog.logDeviceInfo();

    if (preferencesHelper.readBoolean(getString(R.string.pref_wifi_automute), false)) {
      initWifiMute(true);
    }
    mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.tutorial1_activity_java_surface_view);
    mOpenCvCameraView.setMinimumWidth(384);
    mOpenCvCameraView.setMinimumHeight(288);
    mOpenCvCameraView.setMaxFrameSize( 384,288);
    mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE); // INVISIBLE doesn't work
    mOpenCvCameraView.setCvCameraViewListener(this);
  }

  protected UpdateUI createUpdateUI() {
    Restarter restarter = new RobotRestarter();
    UpdateUI result = new UpdateUI(this, dimmer);
    result.setRestarter(restarter);
    result.setTextViews(textNetworkConnectionStatus, textRobotStatus, textGamepad, textOpMode, textErrorMessage, textDeviceName);
    return result;
  }

  protected UpdateUI.Callback createUICallback(UpdateUI updateUI) {
    UpdateUI.Callback result = updateUI.new Callback();
    result.setStateMonitor(new SoundPlayingRobotMonitor());
    return result;
  }

  @Override
  protected void onStart() {
    super.onStart();
    RobotLog.vv(TAG, "onStart()");

    // If we're start()ing after a stop(), then shut the old robot down so
    // we can refresh it with new state (e.g., with new hw configurations)
    shutdownRobot();

    updateUIAndRequestRobotSetup();

    cfgFileMgr.getActiveConfigAndUpdateUI();

    entireScreenLayout.setOnTouchListener(new View.OnTouchListener() {
      @Override
      public boolean onTouch(View v, MotionEvent event) {
        dimmer.handleDimTimer();
        return false;
      }
    });
  }

  @Override
  protected void onResume() {
    super.onResume();
    RobotLog.vv(TAG, "onResume()");
    if (!OpenCVLoader.initDebug()) {
      Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");

      if (OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION, this, mLoaderCallback)) {
//        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.tutorial1_activity_java_surface_view);
//        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
//        mOpenCvCameraView.setCvCameraViewListener(this);
        mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
      }else{
        Log.d("ERROR", "initAsync failed");
      }
    } else {
      Log.d(TAG, "OpenCV library found inside package. Using it!");
//      mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.tutorial1_activity_java_surface_view);
//      mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
//      mOpenCvCameraView.setCvCameraViewListener(this);
      mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
    }
  }

  @Override
  protected void onPause() {
    super.onPause();
    RobotLog.vv(TAG, "onPause()");
    if (programmingModeController.isActive()) {
      programmingModeController.stopProgrammingMode();
    }
    if (mOpenCvCameraView != null) {
      mOpenCvCameraView.disableView();
    }
  }

  @Override
  protected void onStop() {
    // Note: this gets called even when the configuration editor is launched. That is, it gets
    // called surprisingly often. So, we don't actually do much here.
    super.onStop();
    RobotLog.vv(TAG, "onStop()");
  }

  @Override
  protected void onDestroy() {
    super.onDestroy();
    RobotLog.vv(TAG, "onDestroy()");

    shutdownRobot();  // Ensure the robot is put away to bed
    if (callback != null) callback.close();

    PreferenceRemoterRC.getInstance().stop(prefRemoterStartResult);
    DeviceNameManagerFactory.getInstance().stop(deviceNameStartResult);

    unbindFromService();
    // If the app manually (?) is stopped, then we don't need the auto-starting function (?)
    ServiceController.stopService(FtcRobotControllerWatchdogService.class);
    wifiLock.release();

    preferencesHelper.getSharedPreferences().unregisterOnSharedPreferenceChangeListener(sharedPreferencesListener);
    RobotLog.cancelWriteLogcatToDisk();

    if (mOpenCvCameraView != null){
      mOpenCvCameraView.disableView();
    }
  }

  protected void bindToService() {
    readNetworkType();
    Intent intent = new Intent(this, FtcRobotControllerService.class);
    intent.putExtra(NetworkConnectionFactory.NETWORK_CONNECTION_TYPE, networkType);
    bindService(intent, connection, Context.BIND_AUTO_CREATE);
  }

  protected void unbindFromService() {
    if (controllerService != null) {
      unbindService(connection);
    }
  }

  protected void logPackageVersions() {
    RobotLog.logBuildConfig(com.qualcomm.ftcrobotcontroller.BuildConfig.class);
    RobotLog.logBuildConfig(com.qualcomm.robotcore.BuildConfig.class);
    RobotLog.logBuildConfig(com.qualcomm.hardware.BuildConfig.class);
    RobotLog.logBuildConfig(com.qualcomm.ftccommon.BuildConfig.class);
    RobotLog.logBuildConfig(com.google.blocks.BuildConfig.class);
    RobotLog.logBuildConfig(org.firstinspires.inspection.BuildConfig.class);
  }

  protected void logDeviceSerialNumber() {
    RobotLog.ii(TAG, "410c serial number: " + Build.SERIAL);
  }

  protected void readNetworkType() {

    // The code here used to defer to the value found in a configuration file
    // to configure the network type. If the file was absent, then it initialized
    // it with a default.
    //
    // However, bugs have been reported with that approach (empty config files, specifically).
    // Moreover, the non-Wifi-Direct networking is end-of-life, so the simplest and most robust
    // (e.g.: no one can screw things up by messing with the contents of the config file) fix is
    // to do away with configuration file entirely.
    //
    // Control hubs are always running the access point model.  Everything else, for the time
    // being always runs the wifi direct model.
    if (Device.isRevControlHub() == true) {
      networkType = NetworkType.RCWIRELESSAP;
    } else {
      networkType = NetworkType.fromString(preferencesHelper.readString(context.getString(R.string.pref_pairing_kind), NetworkType.globalDefaultAsString()));
    }

    // update the app_settings
    preferencesHelper.writeStringPrefIfDifferent(context.getString(R.string.pref_pairing_kind), networkType.toString());
  }

  @Override
  public void onWindowFocusChanged(boolean hasFocus){
    super.onWindowFocusChanged(hasFocus);
    // When the window loses focus (e.g., the action overflow is shown),
    // cancel any pending hide action. When the window gains focus,
    // hide the system UI.
    if (hasFocus) {
      if (ImmersiveMode.apiOver19()){
        // Immersive flag only works on API 19 and above.
        immersion.hideSystemUI();
      }
    } else {
      immersion.cancelSystemUIHide();
    }
  }


  @Override
  public boolean onCreateOptionsMenu(Menu menu) {
    getMenuInflater().inflate(R.menu.ftc_robot_controller, menu);
    return true;
  }

  @Override
  public boolean onOptionsItemSelected(MenuItem item) {
    int id = item.getItemId();

    if (id == R.id.action_programming_mode) {
      if (cfgFileMgr.getActiveConfig().isNoConfig()) {
        // Tell the user they must configure the robot before starting programming mode.
        // TODO: as we are no longer truly 'modal' this warning should be adapted
        AppUtil.getInstance().showToast(UILocation.BOTH, context, context.getString(R.string.toastConfigureRobotBeforeProgrammingMode));
      } else {
        Intent programmingModeIntent = new Intent(AppUtil.getDefContext(), ProgrammingModeActivity.class);
        programmingModeIntent.putExtra(
            LaunchActivityConstantsList.PROGRAMMING_MODE_ACTIVITY_PROGRAMMING_WEB_HANDLERS,
            new LocalByRefIntentExtraHolder(programmingWebHandlers));
        startActivity(programmingModeIntent);
      }
      return true;
    } else if (id == R.id.action_program_and_manage) {
      Intent programmingModeIntent = new Intent(AppUtil.getDefContext(), ProgramAndManageActivity.class);
      RobotControllerWebInfo webInfo = programmingWebHandlers.getWebServer().getConnectionInformation();
      programmingModeIntent.putExtra(LaunchActivityConstantsList.RC_WEB_INFO, webInfo.toJson());
      startActivity(programmingModeIntent);
    } else if (id == R.id.action_inspection_mode) {
      Intent inspectionModeIntent = new Intent(AppUtil.getDefContext(), RcInspectionActivity.class);
      startActivity(inspectionModeIntent);
      return true;
    }
    else if (id == R.id.action_blocks) {
      Intent blocksIntent = new Intent(AppUtil.getDefContext(), BlocksActivity.class);
      startActivity(blocksIntent);
      return true;
    }
    else if (id == R.id.action_restart_robot) {
      dimmer.handleDimTimer();
      AppUtil.getInstance().showToast(UILocation.BOTH, context, context.getString(R.string.toastRestartingRobot));
      requestRobotRestart();
      return true;
    }
    else if (id == R.id.action_configure_robot) {
      EditParameters parameters = new EditParameters();
      Intent intentConfigure = new Intent(AppUtil.getDefContext(), FtcLoadFileActivity.class);
      parameters.putIntent(intentConfigure);
      startActivityForResult(intentConfigure, RequestCode.CONFIGURE_ROBOT_CONTROLLER.ordinal());
    }
    else if (id == R.id.action_settings) {
	  // historical: this once erroneously used FTC_CONFIGURE_REQUEST_CODE_ROBOT_CONTROLLER
      Intent settingsIntent = new Intent(AppUtil.getDefContext(), FtcRobotControllerSettingsActivity.class);
      startActivityForResult(settingsIntent, RequestCode.SETTINGS_ROBOT_CONTROLLER.ordinal());
      return true;
    }
    else if (id == R.id.action_about) {
      Intent intent = new Intent(AppUtil.getDefContext(), FtcAboutActivity.class);
      startActivity(intent);
      return true;
    }
    else if (id == R.id.action_exit_app) {
      finish();
      return true;
    }

   return super.onOptionsItemSelected(item);
  }

  @Override
  public void onConfigurationChanged(Configuration newConfig) {
    super.onConfigurationChanged(newConfig);
    // don't destroy assets on screen rotation
  }

  @Override
  protected void onActivityResult(int request, int result, Intent intent) {
    if (request == REQUEST_CONFIG_WIFI_CHANNEL) {
      if (result == RESULT_OK) {
        AppUtil.getInstance().showToast(UILocation.BOTH, context, context.getString(R.string.toastWifiConfigurationComplete));
      }
    }
    // was some historical confusion about launch codes here, so we err safely
    if (request == RequestCode.CONFIGURE_ROBOT_CONTROLLER.ordinal() || request == RequestCode.SETTINGS_ROBOT_CONTROLLER.ordinal()) {
      // We always do a refresh, whether it was a cancel or an OK, for robustness
      cfgFileMgr.getActiveConfigAndUpdateUI();
    }
  }

  public void onServiceBind(final FtcRobotControllerService service) {
    RobotLog.vv(FtcRobotControllerService.TAG, "%s.controllerService=bound", TAG);
    controllerService = service;
    updateUI.setControllerService(controllerService);

    updateUIAndRequestRobotSetup();
    programmingWebHandlers.setState(new FtcRobotControllerServiceState() {
      @NonNull
      @Override
      public WebServer getWebServer() {
        return service.getWebServer();
      }

      @Override
      public EventLoopManager getEventLoopManager() {
        return service.getRobot().eventLoopManager;
      }
    });
  }

  private void updateUIAndRequestRobotSetup() {
    if (controllerService != null) {
      callback.networkConnectionUpdate(controllerService.getNetworkConnectionStatus());
      callback.updateRobotStatus(controllerService.getRobotStatus());
      // Only show this first-time toast on headless systems: what we have now on non-headless suffices
      requestRobotSetup(LynxConstants.isRevControlHub()
        ? new Runnable() {
            @Override public void run() {
              showRestartRobotCompleteToast(R.string.toastRobotSetupComplete);
            }
          }
        : null);
    }
  }

  private void requestRobotSetup(@Nullable Runnable runOnComplete) {
    if (controllerService == null) return;

    RobotConfigFile file = cfgFileMgr.getActiveConfigAndUpdateUI();
    HardwareFactory hardwareFactory = new HardwareFactory(context);
    try {
      hardwareFactory.setXmlPullParser(file.getXml());
    } catch (Resources.NotFoundException e) {
      file = RobotConfigFile.noConfig(cfgFileMgr);
      hardwareFactory.setXmlPullParser(file.getXml());
      cfgFileMgr.setActiveConfigAndUpdateUI(false, file);
    }

    OpModeRegister userOpModeRegister = createOpModeRegister();
    eventLoop = new FtcEventLoop(hardwareFactory, userOpModeRegister, callback, this, programmingModeController);
    FtcEventLoopIdle idleLoop = new FtcEventLoopIdle(hardwareFactory, userOpModeRegister, callback, this, programmingModeController);

    controllerService.setCallback(callback);
    controllerService.setupRobot(eventLoop, idleLoop, runOnComplete);

    passReceivedUsbAttachmentsToEventLoop();
  }

  protected OpModeRegister createOpModeRegister() {
    return new FtcOpModeRegister();
  }

  private void shutdownRobot() {
    if (controllerService != null) controllerService.shutdownRobot();
  }

  private void requestRobotRestart() {
    AppUtil.getInstance().showToast(UILocation.BOTH, AppUtil.getDefContext().getString(R.string.toastRestartingRobot));
    //
    RobotLog.clearGlobalErrorMsg();
    RobotLog.clearGlobalWarningMsg();
    shutdownRobot();
    requestRobotSetup(new Runnable() {
      @Override public void run() {
        showRestartRobotCompleteToast(R.string.toastRestartRobotComplete);
        }
      });
  }

  private void showRestartRobotCompleteToast(@StringRes int resid) {
    AppUtil.getInstance().showToast(UILocation.BOTH, AppUtil.getDefContext().getString(resid));
  }

  protected void hittingMenuButtonBrightensScreen() {
    ActionBar actionBar = getActionBar();
    if (actionBar != null) {
      actionBar.addOnMenuVisibilityListener(new ActionBar.OnMenuVisibilityListener() {
        @Override
        public void onMenuVisibilityChanged(boolean isVisible) {
          if (isVisible) {
            dimmer.handleDimTimer();
          }
        }
      });
    }
  }

  protected class SharedPreferencesListener implements SharedPreferences.OnSharedPreferenceChangeListener {
    @Override public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, String key) {
      if (key.equals(context.getString(R.string.pref_app_theme))) {
        ThemedActivity.restartForAppThemeChange(getTag(), getString(R.string.appThemeChangeRestartNotifyRC));
      } else if (key.equals(context.getString(R.string.pref_wifi_automute))) {
        if (preferencesHelper.readBoolean(context.getString(R.string.pref_wifi_automute), false)) {
          initWifiMute(true);
        } else {
          initWifiMute(false);
        }
      }
    }
  }

  protected void initWifiMute(boolean enable) {
    if (enable) {
      wifiMuteStateMachine = new WifiMuteStateMachine();
      wifiMuteStateMachine.initialize();
      wifiMuteStateMachine.start();

      motionDetection = new MotionDetection(2.0, 10);
      motionDetection.startListening();
      motionDetection.registerListener(new MotionDetection.MotionDetectionListener() {
        @Override
        public void onMotionDetected(double vector)
        {
          wifiMuteStateMachine.consumeEvent(WifiMuteEvent.USER_ACTIVITY);
        }
      });
    } else {
      wifiMuteStateMachine.stop();
      wifiMuteStateMachine = null;
      motionDetection.stopListening();
      motionDetection.purgeListeners();
      motionDetection = null;
    }
  }

  @Override
  public void onUserInteraction() {
    if (wifiMuteStateMachine != null) {
      wifiMuteStateMachine.consumeEvent(WifiMuteEvent.USER_ACTIVITY);
    }
  }
}
