document.addEventListener('DOMContentLoaded', () => {
    const canvas = document.getElementById('map');
    const ctx = canvas.getContext('2d');
    
    // --- DOM Elements ---
    const navActiveEl = document.getElementById('nav-active');
    const poseEl = document.getElementById('pose');
    const editModePanel = document.getElementById('edit-mode-panel');
    const missionModePanel = document.getElementById('mission-mode-panel');
    const onDemandButtons = document.getElementById('on-demand-buttons');
    const wpTableContainer = document.getElementById('wp-table-container');
    const routeTableContainer = document.getElementById('route-table-container');
    const recordedPointsContainer = document.getElementById('recorded-points-container');
    const recordingControls = document.getElementById('recording-controls');
    const consoleEl = document.getElementById('console');
    const btnStartMission = document.getElementById('btn-start-mission');
    const btnStopMission = document.getElementById('btn-stop-mission');
    const btnPause = document.getElementById('btn-pause');
    const btnResume = document.getElementById('btn-resume');
    const btnAddWp = document.getElementById('btn-add-wp');
    const btnAddRoute = document.getElementById('btn-add-route');
    const proximityInput = document.getElementById('setting-proximity');
    const densityInput = document.getElementById('setting-density');
    const recordFromSelect = document.getElementById('record-from');
    const recordToSelect = document.getElementById('record-to');
    const btnStartRecording = document.getElementById('btn-start-recording');
    const btnFinishRecording = document.getElementById('btn-finish-recording');
    const routeEditor = document.getElementById('route-editor');
    const routeEditorHeader = document.getElementById('route-editor-header');
    const routePointsContainer = document.getElementById('route-points-container');
    const btnSaveRoute = document.getElementById('btn-save-route');
    const btnExport = document.getElementById('btn-export');
    const btnImport = document.getElementById('btn-import');
    const fileImport = document.getElementById('file-import');

    // --- State ---
    const defaultSettings = { proximity: 0.2, recordDensity: 0.1 };
    let localRouteData = { waypoints: [], routes: {}, settings: { ...defaultSettings } };
    let nav2Path = [];
    let recordedPath = [];
    let pose = { x: 0, y: 0, yaw: 0, yaw_deg: 0 };
    let isRecording = false;
    let isEditingTables = false;
    let currentEditRouteKey = null;
    
    // --- Map Config & Drawing ---
    const metersPerPixel = 0.05;
    const origin = { x: canvas.width / 4, y: canvas.height / 2 }; 
    function worldToScreen(x, y) { return { x: origin.x + x / metersPerPixel, y: origin.y - y / metersPerPixel }; }
    function drawArrow(x,y,yaw,color='#d73a49',size=12,isNear=false){const p=worldToScreen(x,y);if(isNear){ctx.fillStyle='rgba(40,167,69,0.3)';ctx.beginPath();ctx.arc(p.x,p.y,15,0,2*Math.PI);ctx.fill();}ctx.save();ctx.translate(p.x,p.y);ctx.rotate(-yaw);ctx.fillStyle=color;ctx.beginPath();ctx.moveTo(size,0);ctx.lineTo(-size*0.6,size*0.6);ctx.lineTo(-size*0.6,-size*0.6);ctx.closePath();ctx.fill();ctx.restore();}
    function drawPath(path,color,dashed=false,width=3){if(!path || path.length<2)return;ctx.strokeStyle=color;ctx.lineWidth=width;if(dashed)ctx.setLineDash([8,8]);ctx.beginPath();const p0=worldToScreen(path[0].x,path[0].y);ctx.moveTo(p0.x,p0.y);for(const point of path){const p=worldToScreen(point.x,point.y);ctx.lineTo(p.x,p.y);}ctx.stroke();if(dashed)ctx.setLineDash([]);}

    function render() {
        requestAnimationFrame(render);
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        drawPath(nav2Path, '#2b7cff', true);
        drawPath(recordedPath, 'rgba(227, 0, 145, 0.5)', false, 2.5);
        Object.values(localRouteData.routes).forEach(route => drawPath(route, '#e30091', false, 2.5));
        
        const proximity = (localRouteData.settings && typeof localRouteData.settings.proximity === 'number')
            ? localRouteData.settings.proximity
            : defaultSettings.proximity;
        const proximitySq = proximity * proximity;
        localRouteData.waypoints.forEach((wp, index) => {
            const distSq = (pose.x - wp.x)**2 + (pose.y - wp.y)**2;
            const isNear = distSq < proximitySq;
            drawArrow(wp.x, wp.y, (wp.yaw_deg * Math.PI) / 180, '#ff7b2b', 8, isNear);
            const p = worldToScreen(wp.x, wp.y);
            ctx.fillStyle = isNear ? '#28a745' : '#1b1f23'; ctx.font = '12px system-ui'; ctx.textAlign = 'center';
            ctx.fillText(wp.name || `Waypoint ${index + 1}`, p.x, p.y - 15);
        });
        drawArrow(pose.x, pose.y, pose.yaw);
    }

    function updateUI(data) {
        pose = data.pose;
        nav2Path = data.nav2_path;
        recordedPath = data.recorded_path;
        isRecording = data.is_recording;
        
        if (data.mission_mode === 'idle') {
            if (!localRouteData.waypoints.length && !Object.keys(localRouteData.routes).length) {
                localRouteData = data.route_data || { waypoints: [], routes: {}, settings: { ...defaultSettings } };
                localRouteData.settings = { ...defaultSettings, ...(localRouteData.settings || {}) };
            } else {
                const incomingSettings = (data.route_data && data.route_data.settings) || {};
                localRouteData.settings = { ...defaultSettings, ...incomingSettings };
                // Merge in any new/updated routes saved on the server (e.g., after recording)
                const serverRoutes = (data.route_data && data.route_data.routes) || {};
                for (const [k, v] of Object.entries(serverRoutes)) {
                    localRouteData.routes[k] = v;
                }
            }
            if (proximityInput) proximityInput.value = String(localRouteData.settings.proximity);
            if (densityInput) densityInput.value = String(localRouteData.settings.recordDensity);
            renderWaypointSelects();
            editModePanel.classList.remove('hidden');
            missionModePanel.classList.add('hidden');
            btnStartMission.classList.remove('hidden');
            btnStopMission.classList.add('hidden');
            if (!isEditingTables) renderTables();
            // Show recording controls only while recording, or when explicitly opened by user actions
            if (isRecording && recordingControls) {
                recordingControls.classList.remove('hidden');
                btnStartRecording.disabled = true;
                btnFinishRecording.disabled = false;
            } else if (recordingControls) {
                // leave visibility unchanged if user explicitly opened it
                // only auto-hide when not recording and no pending edit/new route context
                if (!recordingControls.dataset.sticky) recordingControls.classList.add('hidden');
                btnStartRecording.disabled = false;
                btnFinishRecording.disabled = true;
            }
            renderRecordedPoints();
            renderWaypointSelects();
        } else {
            localRouteData = data.route_data || { waypoints: [], routes: {}, settings: { ...defaultSettings } };
            localRouteData.settings = { ...defaultSettings, ...(localRouteData.settings || {}) };
            editModePanel.classList.add('hidden');
            missionModePanel.classList.remove('hidden');
            btnStartMission.classList.add('hidden');
            btnStopMission.classList.remove('hidden');
            
            onDemandButtons.innerHTML = '';
            localRouteData.waypoints.forEach((wp) => {
                const btn = document.createElement('button');
                btn.textContent = wp.name || `Go to WP #${wp.id}`;
                btn.dataset.id = wp.id;
                btn.disabled = data.is_moving;
                onDemandButtons.appendChild(btn);
            });
        }
        
        let statusText = 'Nav2: Idle'; let statusClass = 'badge idle';
        if (isRecording) { statusText = 'Nav2: Recording Route'; statusClass = 'badge paused';
        } else if (data.mission_mode === 'active') {
            if (data.is_moving && data.is_paused) { statusText = 'Nav2: Paused'; statusClass = 'badge paused';
            } else if (data.is_moving) { statusText = 'Nav2: Moving'; statusClass = 'badge active';
            } else { statusText = 'Nav2: Awaiting Command'; statusClass = 'badge waiting'; }
        }
        navActiveEl.textContent = statusText; navActiveEl.className = statusClass;
        poseEl.textContent = `Pose: x=${pose.x.toFixed(2)}, y=${pose.y.toFixed(2)}, yaw=${pose.yaw_deg.toFixed(1)}°`;
        btnPause.disabled = !data.is_moving || data.is_paused;
        btnResume.disabled = !data.is_paused;
        btnStartMission.disabled = data.mission_mode === 'active' || localRouteData.waypoints.length === 0;
    }
    
    function renderWaypointSelects() {
        if (!recordFromSelect || !recordToSelect) return;
        const makeOptions = () => localRouteData.waypoints.map(wp => `<option value="${wp.id}">${wp.name || wp.id}</option>`).join('');
        const fromVal = recordFromSelect.value;
        const toVal = recordToSelect.value;
        recordFromSelect.innerHTML = makeOptions();
        recordToSelect.innerHTML = makeOptions();
        if (fromVal) recordFromSelect.value = fromVal;
        if (toVal) recordToSelect.value = toVal;
    }

    function renderRecordedPoints() {
        if (!recordedPointsContainer) return;
        if (!isRecording || !Array.isArray(recordedPath) || recordedPath.length === 0) {
            recordedPointsContainer.innerHTML = '<div class="mono">No points yet.</div>';
            return;
        }
        let html = '<table><thead><tr><th>#</th><th>X</th><th>Y</th><th>Yaw°</th></tr></thead><tbody>';
        recordedPath.forEach((p, i) => {
            const yaw = (typeof p.yaw_deg === 'number') ? p.yaw_deg : (p.yaw ? (p.yaw * 180/Math.PI) : 0);
            html += `<tr><td>${i}</td><td>${p.x.toFixed(2)}</td><td>${p.y.toFixed(2)}</td><td>${yaw.toFixed(1)}</td></tr>`;
        });
        html += '</tbody></table>';
        recordedPointsContainer.innerHTML = html;
    }
    
    function renderTables() {
        let wpHtml = '<table><thead><tr><th>Name</th><th>X</th><th>Y</th><th>Yaw</th><th></th></tr></thead><tbody>';
        localRouteData.waypoints.forEach((wp, i) => {
            wpHtml += `<tr>
                <td><input type="text" class="wp-input" data-index="${i}" data-field="name" value="${wp.name || ''}"></td>
                <td><input type="number" class="wp-input" data-index="${i}" data-field="x" value="${wp.x.toFixed(2)}"></td>
                <td><input type="number" class="wp-input" data-index="${i}" data-field="y" value="${wp.y.toFixed(2)}"></td>
                <td><input type="number" class="wp-input" data-index="${i}" data-field="yaw_deg" value="${wp.yaw_deg.toFixed(1)}"></td>
                <td><button class="btn-delete-wp" data-index="${i}">X</button></td>
            </tr>`;
        });
        wpTableContainer.innerHTML = wpHtml + '</tbody></table>';

        let routeHtml = '<table><thead><tr><th>From</th><th>To</th><th>Points</th><th></th></tr></thead><tbody>';
        for (const [key, path] of Object.entries(localRouteData.routes)) {
            const [from, to] = key.split('-');
            const fromName = localRouteData.waypoints.find(w=>w.id===from)?.name || from;
            const toName = localRouteData.waypoints.find(w=>w.id===to)?.name || to;
            routeHtml += `<tr>
                <td>${fromName}</td><td>${toName}</td><td>${path.length}</td>
                <td>
                    <button class="btn-record-route" data-from="${from}" data-to="${to}">Record</button>
                    <button class="btn-edit-route" data-key="${key}">Edit</button>
                    <button class="btn-delete-route" data-key="${key}">X</button>
                </td>
            </tr>`;
        }
        routeTableContainer.innerHTML = routeHtml + '</tbody></table>';
    }

    btnAddWp.onclick = () => {
        localRouteData.waypoints.push({ id: `wp${Date.now()}`, name: '', x: pose.x, y: pose.y, yaw_deg: pose.yaw_deg });
        renderTables();
        // Focus the name field of the newly added waypoint for quick editing
        const lastIndex = localRouteData.waypoints.length - 1;
        const input = wpTableContainer.querySelector(`input.wp-input[data-index="${lastIndex}"][data-field="name"]`);
        if (input) { input.focus(); input.select(); }
    };
    btnAddRoute.onclick = () => {
        // Open recording controls to create a new route
        if (recordingControls) {
            recordingControls.classList.remove('hidden');
            recordingControls.dataset.sticky = '1';
        }
        renderWaypointSelects();
    };
    wpTableContainer.addEventListener('input', (e) => {
        if (e.target.classList.contains('wp-input')) {
            const { index, field } = e.target.dataset;
            const value = e.target.type === 'number' ? parseFloat(e.target.value) : e.target.value;
            localRouteData.waypoints[index][field] = value;
        }
    });
    // Avoid re-rendering while the user types
    wpTableContainer.addEventListener('focusin', () => { isEditingTables = true; });
    wpTableContainer.addEventListener('focusout', () => { isEditingTables = false; });
    wpTableContainer.addEventListener('click', (e) => {
        if (e.target.classList.contains('btn-delete-wp')) {
            localRouteData.waypoints.splice(e.target.dataset.index, 1);
            renderTables();
        }
    });
    routeTableContainer.addEventListener('click', (e) => {
        if (e.target.classList.contains('btn-delete-route')) {
            const key = e.target.dataset.key;
            delete localRouteData.routes[key];
            renderTables();
            // Persist deletion so it doesn't reappear on next poll
            fetch('/api/set_route_data', {
                method:'POST',
                headers:{'Content-Type':'application/json'},
                body: JSON.stringify(localRouteData)
            });
        } else if (e.target.classList.contains('btn-record-route')) {
            const { from, to } = e.target.dataset;
            if (recordingControls) {
                recordingControls.classList.remove('hidden');
                recordingControls.dataset.sticky = '1';
            }
            if (recordFromSelect && recordToSelect) {
                recordFromSelect.value = from;
                recordToSelect.value = to;
                recordFromSelect.scrollIntoView({ behavior: 'smooth', block: 'center' });
            }
        } else if (e.target.classList.contains('btn-edit-route')) {
            const key = e.target.dataset.key;
            showRouteEditor(key);
        }
    });
    
    proximityInput.onchange = () => { localRouteData.settings.proximity = parseFloat(proximityInput.value); };
    densityInput.onchange = () => { localRouteData.settings.recordDensity = parseFloat(densityInput.value); };

    btnStartRecording.onclick = async () => {
        const from = recordFromSelect?.value;
        const to = recordToSelect?.value;
        if (!from || !to) { alert('Select both From and To waypoints'); return; }
        // Ensure a route entry exists client-side so the table reflects recording intent
        if (!localRouteData.routes[`${from}-${to}`]) {
            localRouteData.routes[`${from}-${to}`] = [];
            renderTables();
        }
        // Sync latest waypoints/routes to the server so it can resolve IDs
        await fetch('/api/set_route_data', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(localRouteData)});
        await fetch('/api/start_recording', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({ from_wp_id: from })});
        routeTableContainer.dataset.recordingTo = to;
        btnStartRecording.disabled = true;
        btnFinishRecording.disabled = false;
    };
    btnFinishRecording.onclick = async () => {
        const to = recordToSelect?.value || routeTableContainer.dataset.recordingTo;
        if (!to) { alert('No destination selected'); return; }
        await fetch('/api/stop_recording', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({ to_wp_id: to })});
        btnStartRecording.disabled = false;
        btnFinishRecording.disabled = true;
        // After finishing, un-sticky so auto-hide can occur
        if (recordingControls) delete recordingControls.dataset.sticky;
    };

    btnStartMission.onclick = async () => { await fetch('/api/set_route_data', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(localRouteData)}); await fetch('/api/start_mission', {method:'POST'}); };
    btnStopMission.onclick = () => fetch('/api/stop_mission', { method: 'POST' });
    btnPause.onclick = () => fetch('/api/pause', { method: 'POST' });
    btnResume.onclick = () => fetch('/api/resume', { method: 'POST' });
    onDemandButtons.addEventListener('click', (e) => { if (e.target.tagName === 'BUTTON' && e.target.dataset.id) { fetch('/api/go_to_waypoint', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({target_wp_id: e.target.dataset.id})});}});
    
    const MAX_LINEAR_SPEED = 0.22; const MAX_ANGULAR_SPEED = 2.84; let joystickData = { linear: { x: 0 }, angular: { z: 0 } };
    const joyManager = nipplejs.create({ zone: document.getElementById('joystick-container'), mode: 'static', position: { right: '80px', bottom: '80px' }, color: 'rgba(0, 122, 255, 0.5)', size: 120, });
    joyManager.on('move', (evt, data) => { const angle = data.angle.radian; const force = data.force; joystickData.linear.x = Math.sin(angle) * force * MAX_LINEAR_SPEED; joystickData.angular.z = -Math.cos(angle) * force * MAX_ANGULAR_SPEED; });
    joyManager.on('end', () => { joystickData.linear.x = 0; joystickData.angular.z = 0; sendTeleop(true); });
    async function sendTeleop(force = false) { if (force || joystickData.linear.x !== 0 || joystickData.angular.z !== 0) { await fetch('/api/teleop', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(joystickData)}); } }
    setInterval(() => sendTeleop(), 100);
    async function poll(){ let lastLogCount = 0; let pollError = false; async function p() { try { const resp = await fetch('/api/status'); if(!resp.ok) throw new Error(`API error: ${resp.status}`); const data = await resp.json(); updateUI(data); pollError = false; if (data.logs.length !== lastLogCount) { consoleEl.innerHTML = data.logs.map(log => `<div>${log}</div>`).join(''); consoleEl.scrollTop = consoleEl.scrollHeight; lastLogCount = data.logs.length; } } catch(e) { console.error("Poll failed:", e); pollError = true; navActiveEl.textContent = 'Nav2: Connection Failed'; navActiveEl.className = 'badge error'; poseEl.textContent = 'Check browser console (F12) for errors.'; } finally { setTimeout(p, pollError ? 2000 : 200); } } p(); }

    function showRouteEditor(key) {
        currentEditRouteKey = key;
        if (!routeEditor) return;
        routeEditor.classList.remove('hidden');
        routeEditorHeader.textContent = `Editing route: ${key}`;
        if (recordingControls) {
            // Also show recording controls during edit for quick re-record
            recordingControls.classList.remove('hidden');
            recordingControls.dataset.sticky = '1';
            const [from, to] = key.split('-');
            if (recordFromSelect && recordToSelect) { recordFromSelect.value = from; recordToSelect.value = to; }
        }
        renderRoutePointsEditor();
    }

    function renderRoutePointsEditor() {
        if (!currentEditRouteKey) return;
        const path = localRouteData.routes[currentEditRouteKey] || [];
        let html = '<table><thead><tr><th>#</th><th>X</th><th>Y</th><th>Yaw°</th><th></th></tr></thead><tbody>';
        path.forEach((p, i) => {
            const yaw = (typeof p.yaw_deg === 'number') ? p.yaw_deg : (p.yaw ? (p.yaw * 180/Math.PI) : 0);
            html += `<tr>
                <td>${i}</td>
                <td>${Number(p.x).toFixed(2)}</td>
                <td>${Number(p.y).toFixed(2)}</td>
                <td>${Number(yaw).toFixed(1)}</td>
                <td><button class="btn-del-pt" data-index="${i}">Delete</button></td>
            </tr>`;
        });
        html += '</tbody></table>';
        routePointsContainer.innerHTML = html;
    }

    routePointsContainer.addEventListener('click', (e) => {
        if (e.target.classList.contains('btn-del-pt')) {
            const idx = parseInt(e.target.dataset.index, 10);
            if (!Number.isNaN(idx)) {
                const path = localRouteData.routes[currentEditRouteKey] || [];
                path.splice(idx, 1);
                localRouteData.routes[currentEditRouteKey] = path;
                renderRoutePointsEditor();
                renderTables();
            }
        }
    });

    btnSaveRoute.onclick = async () => {
        await fetch('/api/set_route_data', { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify(localRouteData) });
    };

    // --- Import/Export ---
    function downloadJSON(filename, dataObj) {
        const blob = new Blob([JSON.stringify(dataObj, null, 2)], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url; a.download = filename; document.body.appendChild(a); a.click();
        setTimeout(() => { document.body.removeChild(a); URL.revokeObjectURL(url); }, 0);
    }
    if (btnExport) {
        btnExport.onclick = () => {
            // Export current local edits, not just server state
            const data = { ...localRouteData, settings: { ...defaultSettings, ...(localRouteData.settings || {}) } };
            downloadJSON('routes.json', data);
        };
    }
    if (btnImport && fileImport) {
        btnImport.onclick = () => fileImport.click();
        fileImport.onchange = async () => {
            const file = fileImport.files && fileImport.files[0];
            if (!file) return;
            try {
                const text = await file.text();
                const json = JSON.parse(text);
                const waypoints = Array.isArray(json.waypoints) ? json.waypoints : [];
                const routes = (json.routes && typeof json.routes === 'object') ? json.routes : {};
                const settings = { ...defaultSettings, ...(json.settings || {}) };
                localRouteData = { waypoints, routes, settings };
                renderTables();
                renderWaypointSelects();
                await fetch('/api/set_route_data', { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify(localRouteData) });
            } catch (e) {
                alert('Failed to import JSON: ' + e.message);
                console.error('Import error', e);
            } finally {
                fileImport.value = '';
            }
        };
    }
    render(); poll();
});