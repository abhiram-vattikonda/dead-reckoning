package com.sih.idr.ui

import androidx.compose.animation.AnimatedVisibility
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ExpandLess
import androidx.compose.material.icons.filled.ExpandMore
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Scaffold
import androidx.compose.material3.SnackbarHost
import androidx.compose.material3.SnackbarHostState
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.material3.OutlinedTextField
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.sih.idr.data.TrajectoryPoint

private fun fmtM(v: Double, digits: Int = 1): String =
    if (v.isNaN()) "—" else "%.${digits}f m".format(v)

private fun fmtPct(v: Double): String =
    if (v.isNaN()) "—" else "%.2f%%".format(v)

@Composable
fun MainScreen(viewModel: TrackingViewModel, locationGranted: Boolean) {
    val snapshot by viewModel.snapshot.collectAsState()
    val isRecording by viewModel.isRecording.collectAsState()
    val testingMode by viewModel.testingMode.collectAsState()
    val navEngineOn by viewModel.navEngineEnabled.collectAsState()
    val seedInfo by viewModel.seedInfo.collectAsState()
    val hasSeed by viewModel.hasGpsSeed.collectAsState()
    val autoRecalc by viewModel.autoRecalculate.collectAsState()
    val recalcInfo by viewModel.recalcInfo.collectAsState()
    val drStatus by viewModel.drStatus.collectAsState()
    val gpsStatus by viewModel.gpsManager.status.collectAsState()
    val imuStatus by viewModel.imuManager.status.collectAsState()
    val exported by viewModel.exportedFiles.collectAsState()
    var destLat by remember { mutableStateOf("") }
    var destLon by remember { mutableStateOf("") }

    var follow by remember { mutableStateOf(true) }
    var controlsExpanded by remember { mutableStateOf(true) }
    val snackbar = remember { SnackbarHostState() }

    LaunchedEffect(Unit) {
        viewModel.messages.collect { msg -> snackbar.showSnackbar(msg) }
    }
    // Re-enable follow whenever a new recording starts.
    LaunchedEffect(isRecording) { if (isRecording) follow = true }

    Scaffold(snackbarHost = { SnackbarHost(snackbar) }) { inner ->
        Column(
            modifier = Modifier.fillMaxSize().padding(inner)
        ) {
            if (!locationGranted) {
                Text(
                    "Location permission required — restart the app and grant it.",
                    color = Color.Red,
                    modifier = Modifier.padding(8.dp)
                )
            }
            // ---------------------------------------------------------- MAP
            MapComponent(
                gpsPoints = snapshot.gpsTrajectory,
                drPoints = snapshot.drTrajectory,
                gpsPos = snapshot.lastGps?.let {
                    TrajectoryPoint(it.timestampMillis, it.latitude, it.longitude)
                },
                drPos = snapshot.lastDr?.let {
                    TrajectoryPoint(
                        it.timestampNanos / 1_000_000, it.latitude, it.longitude
                    )
                },
                plannedRoute = snapshot.plannedRoute,
                follow = follow,
                modifier = Modifier.fillMaxWidth().weight(1f),
                onUserMove = { follow = false },
                onToggleFollow = { follow = !follow }
            )
            // ---------------------------------------------------------- PANEL
            Column(
                modifier = Modifier.fillMaxWidth()
                    .verticalScroll(rememberScrollState())
                    .padding(12.dp),
                verticalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                // Status + statistics (collapsible — header always shows live error + drift)
                var statsExpanded by remember { mutableStateOf(false) }
                Card(modifier = Modifier.fillMaxWidth()) {
                    Column(Modifier.padding(12.dp)) {
                        Row(
                            Modifier.fillMaxWidth(),
                            horizontalArrangement = Arrangement.SpaceBetween,
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Text(
                                "Session info",
                                fontSize = 15.sp,
                                fontWeight = FontWeight.Bold,
                                modifier = Modifier.weight(1f)
                            )
                            Text(
                                "${fmtM(snapshot.stats.currentErrorM)} • ${fmtPct(snapshot.stats.driftPct)}",
                                fontSize = 13.sp,
                                color = Color.DarkGray
                            )
                            IconButton(onClick = { statsExpanded = !statsExpanded }) {
                                Icon(
                                    if (statsExpanded) Icons.Filled.ExpandLess else Icons.Filled.ExpandMore,
                                    contentDescription = if (statsExpanded) "Collapse" else "Expand"
                                )
                            }
                        }
                        AnimatedVisibility(visible = statsExpanded) {
                            Column(verticalArrangement = Arrangement.spacedBy(2.dp)) {
                                Text("GPS: $gpsStatus", fontSize = 13.sp)
                                Text("IMU: $imuStatus  (samples: ${snapshot.imuCount} @ ${snapshot.imuHz.toInt()}Hz, gps: ${snapshot.gpsCount})", fontSize = 13.sp)
                                Text("DR states: ${snapshot.drStateCount}", fontSize = 13.sp)
                                Text(drStatus, fontSize = 13.sp, fontWeight = FontWeight.Bold)
                                Text(viewModel.engineLabel, fontSize = 13.sp, color = Color(0xFF5F6368))
                                Spacer(Modifier.height(4.dp))
                                StatRow("Steps detected", "${snapshot.stepCount}")
                                StatRow("Current DR error", fmtM(snapshot.stats.currentErrorM))
                                StatRow("Distance travelled", fmtM(snapshot.stats.distanceTravelledM))
                                StatRow("Maximum error", fmtM(snapshot.stats.maxErrorM))
                                StatRow("Mean error", fmtM(snapshot.stats.meanErrorM))
                                StatRow("Final error", fmtM(snapshot.stats.finalErrorM))
                                StatRow("Drift", fmtPct(snapshot.stats.driftPct))
                            }
                        }
                    }
                }
                Card(modifier = Modifier.fillMaxWidth()) {
                    Row(
                        Modifier.fillMaxWidth().padding(horizontal = 12.dp, vertical = 4.dp),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Text("Navigation controls", fontWeight = FontWeight.Bold, modifier = Modifier.weight(1f))
                        IconButton(onClick = { controlsExpanded = !controlsExpanded }) {
                            Icon(
                                if (controlsExpanded) Icons.Filled.ExpandLess else Icons.Filled.ExpandMore,
                                contentDescription = if (controlsExpanded) "Collapse navigation controls" else "Expand navigation controls"
                            )
                        }
                    }
                    AnimatedVisibility(visible = controlsExpanded) {
                        Column(
                            Modifier.fillMaxWidth().padding(horizontal = 12.dp, vertical = 4.dp),
                            verticalArrangement = Arrangement.spacedBy(8.dp)
                        ) {
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    Button(
                        onClick = { viewModel.onStart() },
                        enabled = !isRecording && locationGranted,
                        modifier = Modifier.weight(1f)
                    ) { Text("START") }
                    Button(
                        onClick = { viewModel.onStop() },
                        enabled = isRecording,
                        colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFB3261E)),
                        modifier = Modifier.weight(1f)
                    ) { Text("STOP") }
                    OutlinedButton(
                        onClick = { viewModel.onReset() },
                        enabled = !isRecording,
                        modifier = Modifier.weight(1f)
                    ) { Text("RESET") }
                }
                SwitchRow(
                    label = "Testing Mode (GPS = ground truth)",
                    checked = testingMode,
                    onChange = { viewModel.setTestingMode(it) }
                )
                SwitchRow(
                    label = "Navigation Engine (dead reckoning)",
                    checked = navEngineOn,
                    onChange = { viewModel.setNavEngineEnabled(it) }
                )
                Text(seedInfo, fontSize = 13.sp, color = Color.DarkGray)
                if (navEngineOn && !hasSeed) {
                    Text(
                        "No live GPS fix — using offline test reference. " +
                            "DR runs on IMU from this location.",
                        color = Color(0xFF188038), fontSize = 13.sp, fontWeight = FontWeight.Bold
                    )
                }
                Text("Route destination (optional)", fontWeight = FontWeight.Bold)
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                    OutlinedTextField(destLat, { destLat = it }, label = { Text("Latitude") }, modifier = Modifier.weight(1f), singleLine = true)
                    OutlinedTextField(destLon, { destLon = it }, label = { Text("Longitude") }, modifier = Modifier.weight(1f), singleLine = true)
                    Button(onClick = { viewModel.setDestination(destLat.toDoubleOrNull() ?: Double.NaN, destLon.toDoubleOrNull() ?: Double.NaN) }) { Text("ROUTE") }
                }
                if (snapshot.plannedRoute.isNotEmpty()) Text("Green = planned route • ${snapshot.engineHealth}", fontSize = 12.sp, color = Color(0xFF188038))
                else Text("Engine: ${snapshot.engineHealth}", fontSize = 12.sp, color = Color.DarkGray)
                SwitchRow(
                    label = "Auto-recalculate route",
                    checked = autoRecalc,
                    onChange = { viewModel.setAutoRecalculate(it) }
                )
                if (recalcInfo.isNotEmpty()) Text(recalcInfo, fontSize = 12.sp, color = Color(0xFF188038))
                if (!snapshot.locationOn) {
                    Text(
                        "Phone location (GPS) is OFF — blue trail paused, DR keeps running on IMU.",
                        color = Color(0xFFB3261E), fontSize = 13.sp, fontWeight = FontWeight.Bold
                    )
                } else if (navEngineOn && isRecording) {
                    Text(
                        "DR on IMU only from the seed above — GPS (blue) logs independently.",
                        color = Color(0xFFB3261E), fontSize = 13.sp, fontWeight = FontWeight.Bold
                    )
                }
                Row(
                    Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(8.dp),
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    OutlinedButton(
                        onClick = { viewModel.exportCsv() },
                        enabled = !isRecording,
                        modifier = Modifier.weight(1f)
                    ) { Text("EXPORT CSV") }
                }
                        }
                    }
                }
                if (controlsExpanded && exported.isNotEmpty()) {
                    Text("Exported:", fontSize = 12.sp, fontWeight = FontWeight.Bold)
                    exported.forEach { f -> Text(f.absolutePath, fontSize = 11.sp) }
                }
                Text(
                    "Blue = GPS line  •  Red dots = DR samples  •  Green = planned route",
                    fontSize = 12.sp, color = Color.DarkGray
                )
            }
        }
    }
}

@Composable
private fun StatRow(label: String, value: String) {
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
        Text(label, fontSize = 14.sp)
        Text(value, fontSize = 14.sp, fontWeight = FontWeight.Bold)
    }
}

@Composable
private fun SwitchRow(label: String, checked: Boolean, onChange: (Boolean) -> Unit) {
    Row(
        Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(label, fontSize = 14.sp, modifier = Modifier.weight(1f))
        Switch(checked = checked, onCheckedChange = onChange)
    }
}
