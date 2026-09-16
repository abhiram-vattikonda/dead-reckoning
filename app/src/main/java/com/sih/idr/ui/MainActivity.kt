package com.sih.idr.ui

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.activity.viewModels
import androidx.compose.material3.MaterialTheme
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.core.content.ContextCompat
import org.osmdroid.config.Configuration
import java.io.File

class MainActivity : ComponentActivity() {

    private val vm: TrackingViewModel by viewModels()
    private var permissionGranted by mutableStateOf(false)

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { grants ->
        permissionGranted = (grants[Manifest.permission.ACCESS_FINE_LOCATION] == true) ||
            (grants[Manifest.permission.ACCESS_COARSE_LOCATION] == true)
        vm.onLocationPermissionResult(permissionGranted)
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        initOsmDroid()
        permissionGranted = hasLocationPermission()
        if (!permissionGranted) {
            permissionLauncher.launch(
                arrayOf(
                    Manifest.permission.ACCESS_FINE_LOCATION,
                    Manifest.permission.ACCESS_COARSE_LOCATION
                )
            )
        }
        setContent {
            MaterialTheme {
                MainScreen(viewModel = vm, locationGranted = permissionGranted)
            }
        }
    }

    /**
     * osmdroid (OpenStreetMap) setup: no API key needed. A unique User-Agent is
     * required by the OSM tile usage policy; tile cache lives in app-private
     * storage so no storage permission is needed.
     */
    private fun initOsmDroid() {
        val base = File(getExternalFilesDir(null), "osmdroid").also { it.mkdirs() }
        Configuration.getInstance().apply {
            userAgentValue = packageName
            osmdroidBasePath = base
            osmdroidTileCache = File(base, "tiles").also { it.mkdirs() }
        }
    }

    private fun hasLocationPermission(): Boolean {
        return ContextCompat.checkSelfPermission(
            this, Manifest.permission.ACCESS_FINE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED ||
            ContextCompat.checkSelfPermission(
                this, Manifest.permission.ACCESS_COARSE_LOCATION
            ) == PackageManager.PERMISSION_GRANTED
    }
}
