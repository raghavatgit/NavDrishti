package com.astitva.navdrishti.engine

import android.annotation.SuppressLint
import android.content.Context
import android.location.GnssStatus
import android.location.LocationManager
import android.os.Build
import androidx.annotation.RequiresApi

/**
 * NavIC (IRNSS) Satellite Status Manager
 * 
 * Uses Android's GnssStatus API to detect Indian Space Research Organisation (ISRO)
 * NavIC satellites (Constellation Type 7: CONSTELLATION_IRNSS) on L5 (1176.45 MHz) and S bands.
 */
@RequiresApi(Build.VERSION_CODES.N)
class NavIcStatusManager(private val context: Context) {

    interface NavIcCallback {
        fun onNavIcStatusUpdated(
            totalSats: Int,
            navIcSatsTracked: Int,
            navIcSatsUsedInFix: Int,
            avgCarrierToNoiseRatioDbHz: Float
        )
        fun onGnssBlackoutDetected()
    }

    var callback: NavIcCallback? = null
    private val locationManager = context.getSystemService(Context.LOCATION_SERVICE) as LocationManager

    private val gnssStatusCallback = object : GnssStatus.Callback() {
        override fun onSatelliteStatusChanged(status: GnssStatus) {
            val satCount = status.satelliteCount
            var navIcCount = 0
            var navIcUsed = 0
            var cn0Sum = 0f

            for (i in 0 until satCount) {
                val constellationType = status.getConstellationType(i)

                // 7 represents GnssStatus.CONSTELLATION_IRNSS (NavIC)
                if (constellationType == GnssStatus.CONSTELLATION_IRNSS) {
                    navIcCount++
                    val cn0 = status.getCn0DbHz(i)
                    cn0Sum += cn0

                    if (status.usedInFix(i)) {
                        navIcUsed++
                    }
                }
            }

            val avgCn0 = if (navIcCount > 0) cn0Sum / navIcCount else 0f

            // If 0 satellites locked, flag blackout/jamming
            if (satCount == 0 || (navIcCount == 0 && satCount < 3)) {
                callback?.onGnssBlackoutDetected()
            } else {
                callback?.onNavIcStatusUpdated(satCount, navIcCount, navIcUsed, avgCn0)
            }
        }
    }

    @SuppressLint("MissingPermission")
    fun register() {
        locationManager.registerGnssStatusCallback(gnssStatusCallback, null)
    }

    fun unregister() {
        locationManager.unregisterGnssStatusCallback(gnssStatusCallback)
    }
}
