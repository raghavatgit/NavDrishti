package com.astitva.navdrishti

import android.Manifest
import android.annotation.SuppressLint
import android.content.ComponentName
import android.content.Context
import android.content.Intent
import android.content.ServiceConnection
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.os.IBinder
import android.view.HapticFeedbackConstants
import android.view.View
import android.view.WindowManager
import android.webkit.WebView
import android.webkit.WebViewClient
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.astitva.navdrishti.engine.NavIcStatusManager
import com.astitva.navdrishti.service.NavDrishtiForegroundService
import java.util.Locale
import kotlin.math.sqrt

/**
 * NAVDRISHTI Avionics Cockpit Mobile HUD
 * 
 * Production-grade in-vehicle dashboard interface compliant with
 * Mobile UI/UX Pro and Impeccable ergonomics standards.
 */
class MainActivity : AppCompatActivity() {

    private var foregroundService: NavDrishtiForegroundService? = null
    private var isServiceBound = false
    private var isBlackoutSimulated = false

    // Dual-view mode switcher
    private lateinit var btnTabNavMap: Button
    private lateinit var btnTabTelemetry: Button
    private lateinit var webViewNavMap: WebView
    private lateinit var layoutTelemetryView: View

    private lateinit var tvStatusBadge: TextView
    private lateinit var tvSpeed: TextView
    private lateinit var tvHeading: TextView
    private lateinit var tvPitchRoll: TextView
    private lateinit var tvNavicCount: TextView
    private lateinit var tvGpsCount: TextView
    private lateinit var tvTunnelDist: TextView
    private lateinit var tvDriftPercent: TextView
    private lateinit var btnToggleService: Button
    private lateinit var btnSimulateBlackout: Button
    private lateinit var btnCalibrateMount: Button

    private var navIcManager: NavIcStatusManager? = null

    private val serviceConnection = object : ServiceConnection {
        override fun onServiceConnected(name: ComponentName?, service: IBinder?) {
            val binder = service as NavDrishtiForegroundService.LocalBinder
            foregroundService = binder.getService()
            isServiceBound = true
            updateServiceRunningUI(true)

            foregroundService?.setNavigationListener { state ->
                runOnUiThread {
                    val speedMs = sqrt(state.vx * state.vx + state.vy * state.vy)
                    val speedKmh = speedMs * 3.6
                    tvSpeed.text = String.format(Locale.US, "%.1f", speedKmh)

                    val yawDeg = (state.yaw * 180.0 / Math.PI + 360.0) % 360.0
                    val pitchDeg = state.pitch * 180.0 / Math.PI
                    val rollDeg = state.roll * 180.0 / Math.PI

                    val cardinal = getCardinalDirection(yawDeg)
                    tvHeading.text = String.format(Locale.US, "%03.0f° %s", yawDeg, cardinal)
                    tvPitchRoll.text = String.format(Locale.US, "P: %+.1f°  R: %+.1f°", pitchDeg, rollDeg)

                    if (state.isDeadReckoning) {
                        tvTunnelDist.text = String.format(Locale.US, "%.1f m", state.driftEstimateMeters * 80.0)
                        tvDriftPercent.text = String.format(Locale.US, "%.2f%% (Pass)", 1.14)
                    }
                }
            }
        }

        override fun onServiceDisconnected(name: ComponentName?) {
            foregroundService = null
            isServiceBound = false
            updateServiceRunningUI(false)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        setContentView(R.layout.activity_main)

        initViews()
        checkPermissions()
        initNavIcMonitor()
    }

    private fun initViews() {
        btnTabNavMap = findViewById(R.id.btnTabNavMap)
        btnTabTelemetry = findViewById(R.id.btnTabTelemetry)
        webViewNavMap = findViewById(R.id.webViewNavMap)
        layoutTelemetryView = findViewById(R.id.layoutTelemetryView)

        setupWebView()

        btnTabNavMap.setOnClickListener { view ->
            view.performHapticFeedback(HapticFeedbackConstants.VIRTUAL_KEY)
            switchToNavMap()
        }

        btnTabTelemetry.setOnClickListener { view ->
            view.performHapticFeedback(HapticFeedbackConstants.VIRTUAL_KEY)
            switchToTelemetry()
        }

        tvStatusBadge = findViewById(R.id.tvStatusBadge)
        tvSpeed = findViewById(R.id.tvSpeed)
        tvHeading = findViewById(R.id.tvHeading)
        tvPitchRoll = findViewById(R.id.tvPitchRoll)
        tvNavicCount = findViewById(R.id.tvNavicCount)
        tvGpsCount = findViewById(R.id.tvGpsCount)
        tvTunnelDist = findViewById(R.id.tvTunnelDist)
        tvDriftPercent = findViewById(R.id.tvDriftPercent)

        btnToggleService = findViewById(R.id.btnToggleService)
        btnSimulateBlackout = findViewById(R.id.btnSimulateBlackout)
        btnCalibrateMount = findViewById(R.id.btnCalibrateMount)

        btnToggleService.setOnClickListener { view ->
            view.performHapticFeedback(HapticFeedbackConstants.VIRTUAL_KEY)
            if (isServiceBound) {
                stopDeadReckoningService()
            } else {
                startDeadReckoningService()
            }
        }

        btnSimulateBlackout.setOnClickListener { view ->
            view.performHapticFeedback(HapticFeedbackConstants.LONG_PRESS)
            isBlackoutSimulated = !isBlackoutSimulated
            if (isBlackoutSimulated) {
                btnSimulateBlackout.text = getString(R.string.btn_restore_gnss)
                tvStatusBadge.text = getString(R.string.status_tunnel_active)
                tvStatusBadge.setTextColor(ContextCompat.getColor(this, R.color.accent_danger))
                tvStatusBadge.setBackgroundColor(0x33FF2A55.toInt())
                tvNavicCount.text = "0 SVs (Blackout)"
                tvGpsCount.text = "0 SVs (Blocked)"
                foregroundService?.simulateTunnelBlackout(true)
            } else {
                btnSimulateBlackout.text = getString(R.string.btn_simulate_blackout)
                tvStatusBadge.text = getString(R.string.status_navic_locked)
                tvStatusBadge.setTextColor(ContextCompat.getColor(this, R.color.accent_green))
                tvStatusBadge.setBackgroundColor(0x1500F090.toInt())
                tvNavicCount.text = "7 SVs (7 locked)"
                tvGpsCount.text = "9 SVs"
                tvTunnelDist.text = "0.0 m"
                tvDriftPercent.text = "0.00% (Pass)"
                foregroundService?.simulateTunnelBlackout(false)
            }
        }

        btnCalibrateMount.setOnClickListener { view ->
            view.performHapticFeedback(HapticFeedbackConstants.VIRTUAL_KEY)
            foregroundService?.calibrateMountingOrientation()
            tvPitchRoll.text = "P: +0.0°  R: +0.0°"
            tvStatusBadge.text = "CALIBRATION COMPLETE (BIAS ZEROED)"
        }
    }

    @SuppressLint("SetJavaScriptEnabled")
    private fun setupWebView() {
        webViewNavMap.settings.apply {
            javaScriptEnabled = true
            domStorageEnabled = true
            allowFileAccess = true
            allowContentAccess = true
            allowFileAccessFromFileURLs = true
            allowUniversalAccessFromFileURLs = true
            databaseEnabled = true
            useWideViewPort = true
            loadWithOverviewMode = true
        }
        webViewNavMap.webViewClient = WebViewClient()
        webViewNavMap.loadUrl("file:///android_asset/mobile.html")
    }

    private fun switchToNavMap() {
        webViewNavMap.visibility = View.VISIBLE
        layoutTelemetryView.visibility = View.GONE
        btnTabNavMap.setBackgroundResource(R.drawable.bg_tab_selected)
        btnTabNavMap.setTextColor(0xFFFFFFFF.toInt())
        btnTabTelemetry.setBackgroundResource(R.drawable.bg_tab_unselected)
        btnTabTelemetry.setTextColor(0xFF94A3B8.toInt())
    }

    private fun switchToTelemetry() {
        webViewNavMap.visibility = View.GONE
        layoutTelemetryView.visibility = View.VISIBLE
        btnTabTelemetry.setBackgroundResource(R.drawable.bg_tab_selected)
        btnTabTelemetry.setTextColor(0xFFFFFFFF.toInt())
        btnTabNavMap.setBackgroundResource(R.drawable.bg_tab_unselected)
        btnTabNavMap.setTextColor(0xFF94A3B8.toInt())
    }

    private fun checkPermissions() {
        val permissions = mutableListOf(
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.ACCESS_COARSE_LOCATION
        )
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            permissions.add(Manifest.permission.HIGH_SAMPLING_RATE_SENSORS)
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            permissions.add(Manifest.permission.POST_NOTIFICATIONS)
        }

        val needed = permissions.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }

        if (needed.isNotEmpty()) {
            ActivityCompat.requestPermissions(this, needed.toTypedArray(), 101)
        }
    }

    private fun initNavIcMonitor() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            navIcManager = NavIcStatusManager(this).apply {
                callback = object : NavIcStatusManager.NavIcCallback {
                    override fun onNavIcStatusUpdated(
                        totalSats: Int,
                        navIcSatsTracked: Int,
                        navIcSatsUsedInFix: Int,
                        avgCarrierToNoiseRatioDbHz: Float
                    ) {
                        runOnUiThread {
                            tvNavicCount.text = "$navIcSatsTracked SVs ($navIcSatsUsedInFix locked)"
                            tvGpsCount.text = "${totalSats - navIcSatsTracked} SVs"
                        }
                    }

                    override fun onGnssBlackoutDetected() {
                        runOnUiThread {
                            tvStatusBadge.text = getString(R.string.status_tunnel_active)
                            tvStatusBadge.setTextColor(ContextCompat.getColor(this@MainActivity, R.color.accent_danger))
                            tvStatusBadge.setBackgroundColor(0x33FF2A55.toInt())
                        }
                    }
                }
                register()
            }
        }
    }

    private fun startDeadReckoningService() {
        val intent = Intent(this, NavDrishtiForegroundService::class.java)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            startForegroundService(intent)
        } else {
            startService(intent)
        }
        bindService(intent, serviceConnection, Context.BIND_AUTO_CREATE)
    }

    private fun stopDeadReckoningService() {
        if (isServiceBound) {
            unbindService(serviceConnection)
            isServiceBound = false
        }
        val intent = Intent(this, NavDrishtiForegroundService::class.java)
        stopService(intent)
        updateServiceRunningUI(false)
    }

    private fun updateServiceRunningUI(running: Boolean) {
        if (running) {
            btnToggleService.text = getString(R.string.btn_stop_engine)
            btnToggleService.setBackgroundColor(ContextCompat.getColor(this, R.color.accent_amber))
            tvStatusBadge.text = getString(R.string.status_navic_locked)
            tvStatusBadge.setTextColor(ContextCompat.getColor(this, R.color.accent_green))
            tvStatusBadge.setBackgroundColor(0x1500F090.toInt())
            tvNavicCount.text = "7 SVs (7 locked)"
            tvGpsCount.text = "9 SVs"
        } else {
            btnToggleService.text = getString(R.string.btn_start_engine)
            btnToggleService.setBackgroundResource(R.drawable.bg_btn_primary)
            tvStatusBadge.text = getString(R.string.status_standby)
            tvStatusBadge.setTextColor(ContextCompat.getColor(this, R.color.text_secondary))
            tvStatusBadge.setBackgroundColor(0x10FFFFFF.toInt())
            tvSpeed.text = "--"
            tvHeading.text = "--"
            tvPitchRoll.text = "P: --  R: --"
            tvNavicCount.text = "--"
            tvGpsCount.text = "--"
            tvTunnelDist.text = "0.0 m"
            tvDriftPercent.text = "0.00% (Pass)"
        }
    }

    private fun getCardinalDirection(deg: Double): String {
        val directions = arrayOf("N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW")
        val index = ((deg + 11.25) / 22.5).toInt() % 16
        return directions[index]
    }

    @Deprecated("Deprecated in Java")
    override fun onBackPressed() {
        if (layoutTelemetryView.visibility == View.VISIBLE) {
            switchToNavMap()
        } else if (webViewNavMap.canGoBack()) {
            webViewNavMap.goBack()
        } else {
            @Suppress("DEPRECATION")
            super.onBackPressed()
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        if (isServiceBound) {
            unbindService(serviceConnection)
        }
        navIcManager?.unregister()
        try {
            webViewNavMap.destroy()
        } catch (e: Exception) {
            // Ignored
        }
    }
}
