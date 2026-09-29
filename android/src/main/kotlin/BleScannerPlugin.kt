package org.navdrishti.core

class BleScannerPlugin {
    private val rssiHistory = mutableMapOf<String, MutableList<Int>>()

    fun recordRssi(deviceId: String, rssi: Int) {
        val list = rssiHistory.getOrPut(deviceId) { mutableListOf() }
        list.add(rssi)
        if (list.size > 10) list.removeAt(0)
    }

    fun getSmoothedRssi(deviceId: String): Double {
        val list = rssiHistory[deviceId] ?: return -100.0
        return list.average()
    }
}
