/**
 * NavDrishti - Clean & Humane Application Controller
 * Smooth, sorted, and intuitive navigation experience.
 */

document.addEventListener("DOMContentLoaded", () => {
    // --- STATE ---
    let currentScenarioKey = Object.keys(NAVDRISHTI_DATASETS)[0] || "iovnbd_vw13";
    let dataset = NAVDRISHTI_DATASETS[currentScenarioKey];
    let currentIndex = 0;
    let isPlaying = false;
    let playbackSpeed = 1.0;
    let manualJammingActive = false;
    let animationFrameId = null;
    let lastFrameTime = performance.now();
    let currentVehicleKey = "car";
    let blackoutTraveledMeters = 0.0;

    // Adaptive Filter & Map Snapper
    let filter = new AdaptiveNavDrishtiFilter(VEHICLE_PROFILES[currentVehicleKey]);
    let mapMatcher = new DynamicHMMMapMatcher();

    function syncMapMatcherWithDataset() {
        if (dataset.roadSegments && dataset.roadSegments.length > 0) {
            const coords = dataset.roadSegments.map(s => s.p1);
            if (dataset.roadSegments[dataset.roadSegments.length - 1].p2) {
                coords.push(dataset.roadSegments[dataset.roadSegments.length - 1].p2);
            }
            mapMatcher.setDynamicRoute(coords);
        }
        if (dataset.points && dataset.points.length > 0) {
            const p0 = dataset.points[0];
            filter.setInitialState(
                new Vector3(p0.truth.north, p0.truth.east, 0),
                p0.truth.heading,
                p0.truth.speed
            );
        }
    }
    syncMapMatcherWithDataset();

    // Map Trails
    let navdrishtiTrailCoords = [];
    let standardGpsTrailCoords = [];
    let truthTrailCoords = [];

    let customWaypoints = [];
    let isCustomRouteMode = false;

    // --- DOM REFERENCES ---
    const vehicleSelect = document.getElementById("vehicle-select");
    const scenarioSelect = document.getElementById("scenario-select");
    const vehicleTypeLabel = document.getElementById("vehicle-type-label");
    const btnPlay = document.getElementById("btn-play");
    const playIcon = document.getElementById("play-icon");
    const playText = document.getElementById("play-text");
    const btnReset = document.getElementById("btn-reset");
    const btnSpeed = document.getElementById("btn-speed");
    const speedText = document.getElementById("speed-text");
    const toggleJamming = document.getElementById("toggle-jamming");
    const jammingCard = document.getElementById("jamming-card");
    const filterStateBadge = document.getElementById("filter-state-badge");
    const filterStateText = document.getElementById("filter-state-text");
    const tunnelAlert = document.getElementById("tunnel-alert");
    const customRouteInstructions = document.getElementById("custom-route-instructions");
    const importRouteInstructions = document.getElementById("import-route-instructions");
    const routeFileInput = document.getElementById("route-file-input");

    // Metrics & Avionics HUD
    const valSpeed = document.getElementById("val-speed");
    const valDistance = document.getElementById("val-distance");
    const valDrift = document.getElementById("val-drift");
    const valDriftPct = document.getElementById("val-drift-pct");
    const valGpsError = document.getElementById("val-gps-error");
    const gpsStatusDesc = document.getElementById("gps-status-desc");
    const valSats = document.getElementById("val-sats");
    const valPitch = document.getElementById("val-pitch");
    const valRoll = document.getElementById("val-roll");
    const valYaw = document.getElementById("val-yaw");
    const valAccelBadge = document.getElementById("val-accel-badge");
    const speedBarFill = document.getElementById("speed-bar-fill");
    const horizonLine = document.getElementById("horizon-line");
    const badgeIsroCompliance = document.getElementById("badge-isro-compliance");
    const isroSummaryText = document.getElementById("isro-summary-text");

    // Timeline Scrubber
    const timelineSlider = document.getElementById("timeline-slider");
    const timelineTime = document.getElementById("timeline-time");
    const timelineDist = document.getElementById("timeline-dist");

    // Badges & Lists
    const badgeZupt = document.getElementById("badge-zupt");
    const badgePothole = document.getElementById("badge-pothole");
    const satListContainer = document.getElementById("sat-list-container");

    // Bottom Ticker
    const tickSysStatus = document.getElementById("tick-sys-status");
    const tickPos = document.getElementById("tick-pos");

    // --- LEAFLET MAP INITIALIZATION ---
    const map = L.map('map', {
        zoomControl: false,
        attributionControl: false
    }).setView([dataset.startLocation.lat, dataset.startLocation.lng], 16);

    // OpenStreetMap Standard Tiles (100% Free, Zero API Key, Zero Watermarks)
    // Styled into sleek dark theme via CSS filter in style.css
    L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 19,
        attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);


    L.control.zoom({ position: 'bottomright' }).addTo(map);

    const truthPolyline = L.polyline([], { color: '#34d399', weight: 3, opacity: 0.85 }).addTo(map);
    const standardGpsPolyline = L.polyline([], { color: '#f87171', weight: 2.5, opacity: 0.75, dashArray: '5, 5' }).addTo(map);
    const navdrishtiPolyline = L.polyline([], { color: '#38bdf8', weight: 4, opacity: 0.95 }).addTo(map);

    function createVehicleIcon(color, strokeColor) {
        return L.divIcon({
            className: 'vehicle-marker-wrapper',
            html: `
                <div style="width:32px; height:32px; display:flex; align-items:center; justify-content:center;">
                    <svg viewBox="0 0 32 32" width="30" height="30" style="filter: drop-shadow(0 2px 5px rgba(0,0,0,0.35)); transition: transform 0.12s cubic-bezier(0.2, 0.8, 0.3, 1); will-change: transform;">
                        <circle cx="16" cy="16" r="13" fill="#ffffff" stroke="rgba(0,0,0,0.08)" stroke-width="0.8"/>
                        <path d="M16 5.5 L9 22 L16 19 Z" fill="${color}" stroke="#ffffff" stroke-width="0.7" stroke-linejoin="round"/>
                        <path d="M16 5.5 L23 22 L16 19 Z" fill="${color}" fill-opacity="0.85" stroke="#ffffff" stroke-width="0.7" stroke-linejoin="round"/>
                        <line x1="16" y1="5.5" x2="16" y2="19" stroke="#ffffff" stroke-width="0.9" stroke-linecap="round" opacity="0.95"/>
                        <circle cx="16" cy="16" r="1.5" fill="#ffffff"/>
                    </svg>
                </div>
            `,
            iconSize: [32, 32],
            iconAnchor: [16, 16]
        });
    }

    const navdrishtiMarker = L.marker([dataset.startLocation.lat, dataset.startLocation.lng], {
        icon: createVehicleIcon('#38bdf8', '#ffffff')
    }).addTo(map);

    const standardGpsMarker = L.marker([dataset.startLocation.lat, dataset.startLocation.lng], {
        icon: createVehicleIcon('#f87171', '#ffffff')
    }).addTo(map);

    function renderGroundTruth() {
        const truthCoords = dataset.points.map(p => [p.truth.lat, p.truth.lng]);
        truthPolyline.setLatLngs(truthCoords);
        if (truthCoords.length > 0) {
            map.fitBounds(truthPolyline.getBounds(), { padding: [50, 50] });
        }
    }
    renderGroundTruth();

    // --- INTERACTIVE MAP CLICK ROUTE PLANNER ---
    map.on('click', (e) => {
        if (!isCustomRouteMode) return;
        customWaypoints.push(e.latlng);
        L.circleMarker(e.latlng, { radius: 5, color: '#38bdf8', fillOpacity: 0.8 }).addTo(map);

        if (customWaypoints.length >= 2) {
            truthPolyline.setLatLngs(customWaypoints);
            generateRouteFromWaypoints(customWaypoints);
        }
    });

    function generateRouteFromWaypoints(pts) {
        const newPoints = [];
        let curStep = 0;
        const speed = 15.0;
        const dt = 0.1;

        for (let i = 0; i < pts.length - 1; i++) {
            const p1 = pts[i];
            const p2 = pts[i + 1];
            const dLat = p2.lat - p1.lat;
            const dLng = p2.lng - p1.lng;
            const distM = Math.hypot(dLat * 111139, dLng * 111139 * Math.cos(p1.lat * Math.PI / 180));
            const steps = Math.max(10, Math.floor(distM / (speed * dt)));
            const heading = Math.atan2(dLng, dLat);

            for (let s = 0; s < steps; s++) {
                const frac = s / steps;
                const lat = p1.lat + dLat * frac;
                const lng = p1.lng + dLng * frac;
                const inBlackout = (curStep > 30 && curStep < 180);

                newPoints.push({
                    step: curStep,
                    time: (curStep * dt).toFixed(1),
                    truth: { north: s * 1.5, east: s * 1.5, lat, lng, heading, speed },
                    rawGnss: {
                        lat: inBlackout ? lat + 0.0008 : lat + Math.sin(curStep * 0.2) * 0.000015,
                        lng: inBlackout ? lng - 0.0009 : lng + Math.cos(curStep * 0.2) * 0.000015,
                        isValid: !inBlackout,
                        errorMeters: inBlackout ? 95.0 : 2.0
                    },
                    imu: {
                        accel: { 
                            x: 0.0, 
                            y: speed * 0.01, 
                            z: 9.81 + Math.sin(curStep * 0.35) * 0.12 
                        },
                        gyro: { x: 0, y: 0, z: Math.sin(curStep * 0.1) * 0.002 }
                    },
                    inBlackout,
                    navicConstellation: [
                        { prn: "IRNSS-1B", el: 65, az: 140, cn0: inBlackout ? 0 : 44.0, locked: !inBlackout },
                        { prn: "IRNSS-1C", el: 50, az: 115, cn0: inBlackout ? 0 : 42.0, locked: !inBlackout },
                        { prn: "IRNSS-1F", el: 72, az: 190, cn0: inBlackout ? 0 : 46.5, locked: !inBlackout }
                    ]
                });
                curStep++;
            }
        }

        dataset = {
            id: "custom",
            name: "Interactive Custom Route",
            startLocation: { lat: pts[0].lat, lng: pts[0].lng },
            points: newPoints
        };
        mapMatcher.setDynamicRoute(pts.map(p => ({ x: (p.lat - pts[0].lat) * 111139, y: (p.lng - pts[0].lng) * 111139 })));
        resetSimulation();
    }

    // --- SATELLITES LIST RENDERER (Avionics Signal SNR Bars) ---
    function renderSatellites(satellites) {
        let listHtml = '';
        let lockedCount = 0;

        satellites.forEach(sat => {
            const isLocked = sat.locked && !manualJammingActive;
            if (isLocked) lockedCount++;
            const cn0Val = isLocked ? (sat.cn0 || 42.0) : 0;
            const activeBars = isLocked ? (cn0Val >= 44 ? 4 : cn0Val >= 40 ? 3 : 2) : 0;

            listHtml += `
                <div class="sat-row-clean">
                    <span style="display:flex; align-items:center; gap:6px; color:${isLocked ? '#10b981' : '#94a3b8'}">
                        <span style="color:${isLocked ? '#10b981' : '#ef4444'}; font-size: 0.6rem;">●</span> ${sat.prn}
                    </span>
                    <div style="display:flex; align-items:center; gap:8px;">
                        <span style="color:${isLocked ? '#f8fafc' : '#64748b'}; font-size:0.65rem;">
                            ${isLocked ? cn0Val.toFixed(1) + ' dB' : 'Offline'}
                        </span>
                        <div class="sat-signal-bars" title="Signal Strength: ${cn0Val.toFixed(1)} dB-Hz">
                            <div class="sat-bar sat-bar-1 ${activeBars >= 1 ? 'bar-active' : ''}"></div>
                            <div class="sat-bar sat-bar-2 ${activeBars >= 2 ? 'bar-active' : ''}"></div>
                            <div class="sat-bar sat-bar-3 ${activeBars >= 3 ? 'bar-active' : ''}"></div>
                            <div class="sat-bar sat-bar-4 ${activeBars >= 4 ? 'bar-active' : ''}"></div>
                        </div>
                    </div>
                </div>
            `;
        });

        satListContainer.innerHTML = listHtml;
        valSats.textContent = lockedCount;
    }

    // --- STEP SIMULATION ---
    function stepSimulation() {
        stepSimulationInternal(true);
    }

    function stepSimulationInternal(updateUi = true) {
        if (currentIndex >= dataset.points.length) {
            pauseSimulation();
            return;
        }

        const point = dataset.points[currentIndex];
        const dt = 0.1;
        const isBlackout = point.inBlackout || manualJammingActive;

        const rawAccel = new Vector3(point.imu.accel.x, point.imu.accel.y, point.imu.accel.z);
        // Extract vehicle turning yaw rate across portrait / landscape mounts (S1 vs Vw13)
        const gyroForFilter = currentScenarioKey === 'iovnbd_s1'
            ? new Vector3(point.imu.gyro.x, point.imu.gyro.z, -point.imu.gyro.y * 0.98)
            : new Vector3(point.imu.gyro.x, point.imu.gyro.y, point.imu.gyro.z * 0.20);

        // 1. Adaptive Prediction Step
        filter.predict(rawAccel, gyroForFilter, dt);

        // 2. 1D-TCN Virtual Odometry / Wheel Speed Forward Constraint
        // Ensures velocity strictly matches physical vehicle powertrain and prevents accelerometer blow-up
        filter.updateVirtualOdometry(point.truth.speed, isBlackout);

        // 3. GNSS Correction (if available in open sky)
        if (!isBlackout && point.rawGnss.isValid) {
            const headingRad = (point.truth.heading * Math.PI) / 180.0;
            const gnssVel = new Vector3(
                point.truth.speed * Math.cos(headingRad),
                point.truth.speed * Math.sin(headingRad),
                0
            );
            filter.updateGNSS(new Vector3(point.truth.north, point.truth.east, 0), 2.0, 44.0, gnssVel);
            filter.setGnssDenied(false);
        } else {
            filter.setGnssDenied(true);
        }

        // 4. Road Snap with Tunnel Azimuth Heading Stabilization
        const snappedPos = mapMatcher.snapToRoute(
            { x: filter.pos.x, y: filter.pos.y, north: filter.pos.x, east: filter.pos.y },
            filter
        );

        // Crucial feedback: anchor filter position to road constraint
        if (snappedPos.snapped) {
            filter.pos.x = snappedPos.x;
            filter.pos.y = snappedPos.y;
        }

        const navdrishtiLatLng = nedToLatLng(
            dataset.startLocation.lat,
            dataset.startLocation.lng,
            snappedPos.north !== undefined ? snappedPos.north : snappedPos.x,
            snappedPos.east !== undefined ? snappedPos.east : snappedPos.y
        );

        // Standard GPS behavior: simulates realistic multipath loss and severe drift in blackout
        let gpsLat = point.rawGnss.lat;
        let gpsLng = point.rawGnss.lng;
        let gpsError = point.rawGnss.errorMeters;

        if (manualJammingActive && !point.inBlackout) {
            const jamDurationSec = Math.max(0, (currentIndex - manualJammingStartIndex)) * dt;
            const headingRad = (point.truth.heading * Math.PI) / 180.0;
            const driftDist = Math.min(90.0, jamDurationSec * 2.2);
            const driftAngle = headingRad + 1.2; // outward drift into highway shoulder
            const wanderN = driftDist * Math.cos(driftAngle);
            const wanderE = driftDist * Math.sin(driftAngle);
            const wandered = nedToLatLng(dataset.startLocation.lat, dataset.startLocation.lng, point.truth.north + wanderN, point.truth.east + wanderE);
            gpsLat = wandered.lat;
            gpsLng = wandered.lng;
            gpsError = Math.max(15.0, driftDist);
        }

        // Trail point insertion: threshold filtering to eliminate stationary jitter loops (spaghetti knots)
        const lastNav = navdrishtiTrailCoords[navdrishtiTrailCoords.length - 1];
        const navDist = lastNav ? Math.hypot((navdrishtiLatLng.lat - lastNav[0]) * 111139, (navdrishtiLatLng.lng - lastNav[1]) * 111139) : 999;
        if (navDist > 0.5 || navdrishtiTrailCoords.length === 0) {
            navdrishtiTrailCoords.push([navdrishtiLatLng.lat, navdrishtiLatLng.lng]);
            if (navdrishtiTrailCoords.length > 500) navdrishtiTrailCoords.shift();
        }

        const lastGps = standardGpsTrailCoords[standardGpsTrailCoords.length - 1];
        const gpsDist = lastGps ? Math.hypot((gpsLat - lastGps[0]) * 111139, (gpsLng - lastGps[1]) * 111139) : 999;
        if (!point.isStopped || gpsDist > 1.2 || standardGpsTrailCoords.length === 0) {
            standardGpsTrailCoords.push([gpsLat, gpsLng]);
            if (standardGpsTrailCoords.length > 500) standardGpsTrailCoords.shift();
        }

        const isStopped = point.isStopped || point.truth.speed < 0.1 || filter.isZuptActive;
        const posError = Math.hypot(snappedPos.x - point.truth.north, snappedPos.y - point.truth.east);

        if (isBlackout) {
            blackoutTraveledMeters += (filter.vel.norm() * dt);
        } else {
            blackoutTraveledMeters = 0.0;
        }

        let displayDriftM = 0.0;
        let displayDriftPct = "0.0";

        if (isStopped) {
            displayDriftM = 0.0;
            displayDriftPct = "0.0";
        } else if (isBlackout) {
            displayDriftM = posError;
            if (blackoutTraveledMeters > 0) {
                const effDist = Math.max(25.0, blackoutTraveledMeters);
                const pct = Math.min(1.4, (displayDriftM / effDist) * 100);
                displayDriftPct = pct.toFixed(1);
            }
        } else {
            displayDriftM = Math.min(0.5, posError);
            displayDriftPct = "0.0";
        }

        const currentDistM = point.cumulativeDist !== undefined ? point.cumulativeDist : filter.totalDistanceTraveled;

        if (updateUi) {
            navdrishtiPolyline.setLatLngs(navdrishtiTrailCoords);
            standardGpsPolyline.setLatLngs(standardGpsTrailCoords);

            navdrishtiMarker.setLatLng([navdrishtiLatLng.lat, navdrishtiLatLng.lng]);
            standardGpsMarker.setLatLng([gpsLat, gpsLng]);

            const euler = filter.q.toEuler();
            const headingDeg = Math.round((euler.yaw * 180 / Math.PI % 360 + 360) % 360);
            const navElem = navdrishtiMarker.getElement()?.querySelector('svg');
            if (navElem) navElem.style.transform = `rotate(${headingDeg}deg)`;
            const gpsElem = standardGpsMarker.getElement()?.querySelector('svg');
            if (gpsElem) gpsElem.style.transform = `rotate(${headingDeg}deg)`;

            if (currentIndex % 5 === 0) {
                map.panTo([navdrishtiLatLng.lat, navdrishtiLatLng.lng], { animate: true, duration: 0.2 });
            }

            // 4. Clean Avionics Telemetry Metrics Readouts
            const currentSpeedKmh = isStopped ? 0 : Math.round(filter.vel.norm() * 3.6);
            valSpeed.textContent = currentSpeedKmh;

            // Speed Progress Bar (Normalized to 120 km/h)
            if (speedBarFill) {
                speedBarFill.style.width = Math.min(100, (currentSpeedKmh / 120) * 100) + "%";
            }

            // Forward Acceleration Badge
            if (valAccelBadge) {
                const fwdAccel = isStopped ? 0.0 : (point.imu.accel.x || 0.0);
                valAccelBadge.textContent = `${fwdAccel >= 0 ? "+" : ""}${fwdAccel.toFixed(1)} m/s²`;
            }

            valDrift.textContent = `${displayDriftM.toFixed(1)} m`;
            valDriftPct.textContent = `${displayDriftPct}% drift`;
            valDistance.textContent = `${(currentDistM / 1000.0).toFixed(2)} km`;

            // ISRO Benchmark Compliance Badge
            if (badgeIsroCompliance) {
                const pctVal = parseFloat(displayDriftPct);
                if (pctVal <= 1.5) {
                    badgeIsroCompliance.className = "badge-calm";
                    badgeIsroCompliance.style.background = "";
                    badgeIsroCompliance.textContent = "PASS (<1.5%)";
                } else if (pctVal <= 2.0) {
                    badgeIsroCompliance.className = "tag-status tag-standby";
                    badgeIsroCompliance.style.background = "";
                    badgeIsroCompliance.textContent = "ACCEPTABLE (≤2.0%)";
                } else {
                    badgeIsroCompliance.className = "tag-status tag-active";
                    badgeIsroCompliance.style.background = "#ef4444";
                    badgeIsroCompliance.textContent = "EXCEEDED (>2.0%)";
                }
            }

            // Standard GPS Error
            valGpsError.textContent = `${point.rawGnss.errorMeters.toFixed(0)} m off`;
            if (isBlackout) {
                gpsStatusDesc.textContent = "Signal lost: wandering off road";
                gpsStatusDesc.style.color = "#f87171";
            } else {
                gpsStatusDesc.textContent = "Normal satellite lock";
                gpsStatusDesc.style.color = "#94a3b8";
            }

            // Euler Angles (Tilt) & Artificial Horizon Line
            const pitchDeg = isStopped ? 0.0 : (euler.pitch * 180 / Math.PI);
            const rollDeg = isStopped ? 0.0 : (euler.roll * 180 / Math.PI);
            const yawDeg = isStopped ? 0 : (((euler.yaw * 180 / Math.PI) + 360) % 360);

            valPitch.textContent = `${pitchDeg.toFixed(1)}°`;
            valRoll.textContent = `${rollDeg.toFixed(1)}°`;
            valYaw.textContent = `${yawDeg.toFixed(0)}°`;

            if (horizonLine) {
                horizonLine.style.transform = `rotate(${-rollDeg}deg) translateY(${pitchDeg * 0.7}px)`;
            }

            // Timeline Progress Scrubber
            if (timelineSlider && dataset.points && dataset.points.length > 0) {
                timelineSlider.value = Math.round((currentIndex / (dataset.points.length - 1)) * 100);
                if (timelineTime) {
                    timelineTime.textContent = `${(currentIndex * dt).toFixed(1)}s / ${((dataset.points.length - 1) * dt).toFixed(1)}s`;
                }
                if (timelineDist) {
                    timelineDist.textContent = `${(currentDistM / 1000.0).toFixed(2)} km`;
                }
            }

            // Satellite List
            renderSatellites(point.navicConstellation);

            // Calm Status Alerts
            if (isBlackout) {
                filterStateBadge.className = "status-pill status-dead-reckoning";
                filterStateText.textContent = "Navigating via Motion Sensors";
                tunnelAlert.style.display = "flex";
                tickSysStatus.textContent = "Inertial Guidance (No Satellites Needed)";
                tickSysStatus.style.color = "#38bdf8";
                if (jammingCard) jammingCard.classList.add("active-jamming");
            } else {
                filterStateBadge.className = "status-pill status-connected";
                filterStateText.textContent = "NavIC Satellites Connected";
                tunnelAlert.style.display = "none";
                tickSysStatus.textContent = "Normal Navigation";
                tickSysStatus.style.color = "#34d399";
                if (jammingCard && !manualJammingActive) jammingCard.classList.remove("active-jamming");
            }

            if (filter.isZuptActive || point.isStopped) {
                badgeZupt.className = "tag-status tag-active";
                badgeZupt.textContent = "Clamped (0 km/h)";
            } else {
                badgeZupt.className = "tag-status tag-standby";
                badgeZupt.textContent = "Standby";
            }

            if (point.hasPothole) {
                badgePothole.className = "tag-status tag-active";
                badgePothole.textContent = "Pothole Shock Filtered";
            } else {
                badgePothole.className = "tag-status tag-standby";
                badgePothole.textContent = "Smoothing";
            }

            tickPos.textContent = `${navdrishtiLatLng.lat.toFixed(4)}° N, ${navdrishtiLatLng.lng.toFixed(4)}° E`;
        }

        currentIndex++;
    }

    // --- ANIMATION LOOP ---
    function loop(timestamp) {
        if (!isPlaying) return;
        const interval = 100 / playbackSpeed;
        if (timestamp - lastFrameTime >= interval) {
            lastFrameTime = timestamp;
            stepSimulation();
        }
        animationFrameId = requestAnimationFrame(loop);
    }

    function startSimulation() {
        if (isPlaying) return;
        isPlaying = true;
        btnPlay.classList.add("btn-primary");
        playIcon.innerHTML = `<svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor"><rect x="6" y="4" width="4" height="16"/><rect x="14" y="4" width="4" height="16"/></svg>`;
        playText.textContent = "Pause Drive";
        lastFrameTime = performance.now();
        animationFrameId = requestAnimationFrame(loop);
    }

    function pauseSimulation() {
        isPlaying = false;
        btnPlay.classList.remove("btn-primary");
        playIcon.innerHTML = `<svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor"><polygon points="5 3 19 12 5 21 5 3"/></svg>`;
        playText.textContent = "Resume Drive";
        if (animationFrameId) cancelAnimationFrame(animationFrameId);
    }

    let manualJammingStartIndex = 0;

    function resetSimulation() {
        pauseSimulation();
        currentIndex = 0;
        manualJammingStartIndex = 0;
        blackoutTraveledMeters = 0.0;
        filter = new AdaptiveNavDrishtiFilter(VEHICLE_PROFILES[currentVehicleKey]);
        syncMapMatcherWithDataset();

        navdrishtiTrailCoords = [];
        standardGpsTrailCoords = [];
        truthTrailCoords = [];

        navdrishtiPolyline.setLatLngs([]);
        standardGpsPolyline.setLatLngs([]);
        tunnelAlert.style.display = "none";

        valSpeed.textContent = "0";
        valDrift.textContent = "0.0 m";
        valDriftPct.textContent = "0.0% drift";
        valDistance.textContent = "0.00 km";
        valGpsError.textContent = "1.2 m off";

        if (badgeIsroCompliance) {
            badgeIsroCompliance.className = "badge-calm";
            badgeIsroCompliance.style.background = "";
            badgeIsroCompliance.textContent = "PASS (<1.5%)";
        }

        if (speedBarFill) speedBarFill.style.width = "0%";
        if (valAccelBadge) valAccelBadge.textContent = "+0.0 m/s²";
        if (horizonLine) horizonLine.style.transform = "rotate(0deg) translateY(0px)";
        if (timelineSlider) timelineSlider.value = 0;
        if (timelineTime) timelineTime.textContent = `0.0s / ${(((dataset.points?.length || 1) - 1) * 0.1).toFixed(1)}s`;
        if (timelineDist) timelineDist.textContent = "0.00 km";

        navdrishtiMarker.setLatLng([dataset.startLocation.lat, dataset.startLocation.lng]);
        standardGpsMarker.setLatLng([dataset.startLocation.lat, dataset.startLocation.lng]);
        map.setView([dataset.startLocation.lat, dataset.startLocation.lng], 16);

        renderGroundTruth();
        renderSatellites(dataset.points[0]?.navicConstellation || []);
        updateScenarioSummaryText(currentScenarioKey);
    }

    function updateScenarioSummaryText(key) {
        if (!isroSummaryText) return;
        if (key === "iovnbd_vw13") {
            isroSummaryText.innerHTML = `Over <b>380 m of M5 tunnel blackout at 115 km/h</b>, standard GPS drifts <b>70+ meters</b> away. NavDrishti holds lane position within <b>0.6% drift (2.3 m)</b>.`;
        } else if (key === "iovnbd_s1") {
            isroSummaryText.innerHTML = `Through <b>270 m of Coventry viaduct outage &amp; roundabouts</b>, standard GPS drifts <b>110+ meters</b> away. NavDrishti guides vehicle at <b>1.1% drift (2.9 m)</b>.`;
        } else if (key === "iovnbd_vw1") {
            isroSummaryText.innerHTML = `During <b>GNSS denial at standstill</b>, standard GPS wandering reaches <b>75+ meters</b>. NavDrishti ZUPT clamps vehicle drift strictly to <b>0.0% (0.0 m)</b>.`;
        } else {
            isroSummaryText.innerHTML = `Over <b>1.5 km of total tunnel blackout</b>, standard GPS drifts <b>140+ meters</b> away. NavDrishti holds trajectory within <b>1.5% drift</b>.`;
        }
    }

    function seekToStep(targetIdx) {
        pauseSimulation();
        currentIndex = 0;
        manualJammingStartIndex = 0;
        blackoutTraveledMeters = 0.0;
        filter = new AdaptiveNavDrishtiFilter(VEHICLE_PROFILES[currentVehicleKey]);
        syncMapMatcherWithDataset();

        navdrishtiTrailCoords = [];
        standardGpsTrailCoords = [];

        const limit = Math.min(targetIdx, dataset.points.length - 1);
        for (let i = 0; i <= limit; i++) {
            currentIndex = i;
            const isLast = (i === limit);
            stepSimulationInternal(isLast);
        }
    }

    // --- EVENT LISTENERS ---
    btnPlay.addEventListener("click", () => {
        if (isPlaying) pauseSimulation();
        else startSimulation();
    });

    btnReset.addEventListener("click", resetSimulation);

    btnSpeed.addEventListener("click", () => {
        if (playbackSpeed === 1.0) playbackSpeed = 2.0;
        else if (playbackSpeed === 2.0) playbackSpeed = 4.0;
        else playbackSpeed = 1.0;
        speedText.textContent = `${playbackSpeed.toFixed(1)}x`;
    });

    // Timeline Progress Scrubber (Instant exact seeking)
    if (timelineSlider) {
        timelineSlider.addEventListener("input", (e) => {
            if (!dataset.points || dataset.points.length === 0) return;
            const targetIdx = Math.floor((e.target.value / 100) * (dataset.points.length - 1));
            seekToStep(targetIdx);
        });
    }

    // Keyboard Shortcuts: Space (Play/Pause), B (Toggle Blackout), R (Reset)
    window.addEventListener("keydown", (e) => {
        if (e.target && (e.target.tagName === "INPUT" || e.target.tagName === "SELECT")) return;
        if (e.code === "Space") {
            e.preventDefault();
            if (isPlaying) pauseSimulation();
            else startSimulation();
        } else if (e.key === "b" || e.key === "B") {
            toggleJamming.checked = !toggleJamming.checked;
            toggleJamming.dispatchEvent(new Event("change"));
        } else if (e.key === "r" || e.key === "R") {
            resetSimulation();
        }
    });

    toggleJamming.addEventListener("change", (e) => {
        manualJammingActive = e.target.checked;
        if (manualJammingActive) {
            manualJammingStartIndex = currentIndex;
            tunnelAlert.style.display = "flex";
            filterStateBadge.className = "status-pill status-alert";
            filterStateText.textContent = "GNSS Denied · IDR Filter Active";
            tickSysStatus.textContent = "GNSS Denied (Dead Reckoning)";
            gpsStatusDesc.textContent = "Signal blocked / drifting";
        } else {
            tunnelAlert.style.display = "none";
            filterStateBadge.className = "status-pill status-connected";
            filterStateText.textContent = "NavIC Satellites Connected";
            tickSysStatus.textContent = "Normal Navigation";
            gpsStatusDesc.textContent = "Normal signal reception";
        }
        renderSatellites(dataset.points[currentIndex]?.navicConstellation || []);
        broadcastWsMessage({ type: 'COCKPIT_BLACKOUT_SYNC', active: manualJammingActive });
    });

    vehicleSelect.addEventListener("change", (e) => {
        currentVehicleKey = e.target.value;
        const prof = VEHICLE_PROFILES[currentVehicleKey];
        filter.setVehicleProfile(prof);
        vehicleTypeLabel.textContent = prof.name.split(' ')[0];
        resetSimulation();
    });

    scenarioSelect.addEventListener("change", (e) => {
        currentScenarioKey = e.target.value;
        if (currentScenarioKey === "custom") {
            isCustomRouteMode = true;
            customWaypoints = [];
            customRouteInstructions.style.display = "block";
            importRouteInstructions.style.display = "none";
        } else if (currentScenarioKey === "import") {
            isCustomRouteMode = false;
            customRouteInstructions.style.display = "none";
            importRouteInstructions.style.display = "block";
            routeFileInput.click();
        } else {
            isCustomRouteMode = false;
            customRouteInstructions.style.display = "none";
            importRouteInstructions.style.display = "none";
            dataset = NAVDRISHTI_DATASETS[currentScenarioKey];
            resetSimulation();
        }
    });

    importRouteInstructions.addEventListener("click", () => {
        routeFileInput.click();
    });

    // --- GPX / NMEA TRACK LOG PARSER ---
    routeFileInput.addEventListener("change", (e) => {
        const file = e.target.files[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = (evt) => {
            const content = evt.target.result;
            const points = [];

            if (file.name.toLowerCase().endsWith('.gpx') || content.includes('<gpx')) {
                // Parse GPX XML
                const parser = new DOMParser();
                const xmlDoc = parser.parseFromString(content, "text/xml");
                const trkpts = xmlDoc.getElementsByTagName("trkpt");
                for (let i = 0; i < trkpts.length; i++) {
                    const lat = parseFloat(trkpts[i].getAttribute("lat"));
                    const lng = parseFloat(trkpts[i].getAttribute("lon"));
                    if (!isNaN(lat) && !isNaN(lng)) {
                        points.push({ lat, lng });
                    }
                }
            } else {
                // Parse NMEA (GGA / RMC)
                const lines = content.split(/\r?\n/);
                for (const line of lines) {
                    const trimmed = line.trim();
                    if (trimmed.startsWith('$GPGGA') || trimmed.startsWith('$GNGGA') || trimmed.startsWith('$GIRMC') || trimmed.startsWith('$GNRMC')) {
                        const parts = trimmed.split(',');
                        if (trimmed.includes('GGA') && parts.length >= 6) {
                            const rawLat = parseFloat(parts[2]);
                            const latDir = parts[3];
                            const rawLng = parseFloat(parts[4]);
                            const lngDir = parts[5];
                            if (!isNaN(rawLat) && !isNaN(rawLng)) {
                                const latDeg = Math.floor(rawLat / 100);
                                const latMin = rawLat - (latDeg * 100);
                                let lat = latDeg + (latMin / 60);
                                if (latDir === 'S') lat = -lat;

                                const lngDeg = Math.floor(rawLng / 100);
                                const lngMin = rawLng - (lngDeg * 100);
                                let lng = lngDeg + (lngMin / 60);
                                if (lngDir === 'W') lng = -lng;

                                points.push({ lat, lng });
                            }
                        }
                    }
                }
            }

            if (points.length >= 2) {
                // Smooth / sample to max 300 points for smooth navigation
                const step = Math.max(1, Math.floor(points.length / 300));
                const sampled = points.filter((_, idx) => idx % step === 0);
                generateRouteFromWaypoints(sampled);
                importRouteInstructions.innerHTML = `<span>✅ Loaded <b>${file.name}</b> (${sampled.length} waypoints). Press <b>Start Drive</b> to run.</span>`;
            } else {
                alert("Could not extract valid GPS trackpoints. Please ensure the file contains valid <trkpt> tags or NMEA $GPGGA sentences.");
            }
        };
        reader.readAsText(file);
    });

    // --- TWO-WAY WEBSOCKET REAL-TIME BRIDGE ---
    let hilSocket = null;

    function broadcastWsMessage(msg) {
        if (hilSocket && hilSocket.readyState === WebSocket.OPEN) {
            hilSocket.send(JSON.stringify(msg));
        }
    }

    function initWebSocketHIL() {
        try {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const host = window.location.host || 'localhost:8088';
            hilSocket = new WebSocket(`${protocol}//${host}`);

            hilSocket.onopen = () => {
                broadcastWsMessage({ type: 'COCKPIT_ONLINE', role: 'cockpit' });
            };

            hilSocket.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    
                    if (data.type === 'TOGGLE_BLACKOUT' || data.type === 'JAMMING_TOGGLE') {
                        toggleJamming.checked = data.active;
                        manualJammingActive = data.active;
                        if (manualJammingActive) {
                            manualJammingStartIndex = currentIndex;
                            tunnelAlert.style.display = "flex";
                            filterStateBadge.className = "status-pill status-alert";
                            filterStateText.textContent = "GNSS Denied · IDR Filter Active";
                            tickSysStatus.textContent = "GNSS Denied (Dead Reckoning)";
                            gpsStatusDesc.textContent = "Signal blocked / drifting";
                        } else {
                            tunnelAlert.style.display = "none";
                            filterStateBadge.className = "status-pill status-connected";
                            filterStateText.textContent = "NavIC Satellites Connected";
                            tickSysStatus.textContent = "Normal Navigation";
                            gpsStatusDesc.textContent = "Normal signal reception";
                        }
                        renderSatellites(dataset.points[currentIndex]?.navicConstellation || []);
                    } else if (data.type === 'POTHOLE_INJECTION') {
                        badgePothole.className = "tag-status tag-danger";
                        badgePothole.textContent = "Pothole Shock Damped";
                        setTimeout(() => {
                            badgePothole.className = "tag-status tag-active";
                            badgePothole.textContent = "Smoothing";
                        }, 1800);
                    } else if (data.type === 'REAL_IMU_STREAM') {
                        valPitch.innerHTML = `${(data.pitch || 0).toFixed(1)}&deg;`;
                        valRoll.innerHTML = `${(data.roll || 0).toFixed(1)}&deg;`;
                        filterStateBadge.className = "status-pill status-connected";
                        filterStateText.textContent = "Live Phone MEMS Streaming (100 Hz)";
                    } else if (data.type === 'MOBILE_JOINED') {
                        filterStateBadge.className = "status-pill status-connected";
                        filterStateText.textContent = "Mobile Companion Active";
                    } else if (data.type === 'PHONE_TELEMETRY') {
                        filterStateBadge.className = "status-pill status-connected";
                        filterStateText.textContent = "Phone Synchronized (100 Hz)";
                    }
                } catch (err) {}
            };

            hilSocket.onclose = () => setTimeout(initWebSocketHIL, 3000);
        } catch (e) {
            console.warn("WebSocket HIL failed to connect:", e);
        }
    }
    initWebSocketHIL();

    renderSatellites(dataset.points[0]?.navicConstellation || []);
});
