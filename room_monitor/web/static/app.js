let currentMode = "IDLE";
let loopTimer = null;
let mapSeq = -1, mapImg = null, mapInfo = null, robotState = null, robotPath = [];
let currentMapName = "";

let mappingMode = "IDLE";

let spMode = "NONE";
let spWaypoints = [];
let isRecording = false;
let isRunning = false;
let isPaused = false;

let isEditMode = false;
let isCameraOn = false;

let zoneData = null;
let zoneColors = {};

// 作法 A：地圖影像 ready 後才觸發分區
let autoSegRequested = false;
let mapFrameReady = false;

// ====== 編輯工具（RENAME / SPLIT / MERGE）======
let editTool = 'RENAME';
let splitTarget = null;
let mergeSelected = [];
let editStartPos = null; // ratio
let editLine = null;     // canvas line for split

const mapCvs = document.getElementById('cvs-map');
const mapCtx = mapCvs.getContext('2d');
const mapLayer = document.getElementById('map-interaction-layer');

const spCvs = document.getElementById('sp-canvas');
const spCtx = spCvs.getContext('2d');
const spLayer = document.getElementById('sp-interaction-layer');
const spStatus = document.getElementById('sp-status-bar');
const spMapName = document.getElementById('sp-map-name');

let mapTrans = { x:0, y:0, scale:1, rotate:0 };
let spTrans = { x:0, y:0, scale:1, rotate:0 };

function switchView(id) {
    document.querySelectorAll('.view').forEach(v => v.classList.remove('active'));
    document.getElementById(id).classList.add('active');
}
function goHome() { switchView('view-home'); }

function initViewControl(layer, transformObj, sceneId, onInteract) {
    let mode = 'NONE';
    let lastX=0, lastY=0;
    let startX=0, startY=0;

    layer.addEventListener('contextmenu', e => e.preventDefault());

    layer.addEventListener('pointerdown', e => {
        e.preventDefault();
        startX = e.clientX; startY = e.clientY;

        // 如果 onInteract('DOWN') 回傳 true，就不要進入 ROTATE/PAN
        if (e.button === 0 && onInteract && onInteract('DOWN', e)) {}
        else if (e.button === 0) mode = 'ROTATE';
        else if (e.button === 1 || e.button === 2) mode = 'PAN';

        lastX = e.clientX; lastY = e.clientY;
    });

    layer.addEventListener('pointermove', e => {
        e.preventDefault();
        if (mode === 'ROTATE') {
            transformObj.rotate += (e.clientX - lastX) * 0.5;
            applyTransform(sceneId, transformObj);
        } else if (mode === 'PAN') {
            transformObj.x += e.clientX - lastX;
            transformObj.y += e.clientY - lastY;
            applyTransform(sceneId, transformObj);
        } else {
            if (onInteract) onInteract('MOVE', e);
        }
        lastX = e.clientX; lastY = e.clientY;
    });

    layer.addEventListener('pointerup', e => {
        e.preventDefault();
        const dist = Math.sqrt(Math.pow(e.clientX - startX, 2) + Math.pow(e.clientY - startY, 2));
        const isClick = dist < 5;

        // 如果剛剛有進入 ROTATE/PAN，這次 UP 不轉交給 onInteract
        if (mode !== 'NONE') mode = 'NONE';
        else if (onInteract) onInteract('UP', e, isClick);
    });

    layer.addEventListener('wheel', e => {
        e.preventDefault();
        const delta = e.deltaY > 0 ? 0.9 : 1.1;
        transformObj.scale = Math.min(Math.max(0.1, transformObj.scale * delta), 10);
        applyTransform(sceneId, transformObj);
    }, {passive: false});
}

function applyTransform(elemId, t) {
    const el = document.getElementById(elemId);
    if(el) el.style.transform = `translate(${t.x}px, ${t.y}px) scale(${t.scale}) rotate(${t.rotate}deg)`;
}

function getPreciseRatio(e, canvas) {
    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    return { rx: Math.max(0, Math.min(1, x / rect.width)), ry: Math.max(0, Math.min(1, y / rect.height)) };
}

// --- 1. 建圖模式互動 ---
let mapTempLine=null;
let mapStartPos = {rx:0, ry:0};
initViewControl(mapLayer, mapTrans, 'map-scene', (type, e, isClick) => {
    if (mappingMode === 'NAV') {
        const pos = getPreciseRatio(e, mapCvs);
        if (type === 'DOWN') {
            mapStartPos = pos;
            mapTempLine = { x1: e.offsetX, y1: e.offsetY, x2: e.offsetX, y2: e.offsetY };
            return true;
        } else if (type === 'MOVE') {
            if (mapTempLine) { mapTempLine.x2 = e.offsetX; mapTempLine.y2 = e.offsetY; drawMapping(); }
        } else if (type === 'UP') {
            if (mapTempLine) {
                const dx = mapTempLine.x2 - mapTempLine.x1;
                const dy = mapTempLine.y2 - mapTempLine.y1;
                const dist = Math.sqrt(dx*dx + dy*dy);
                let yaw = (dist > 10) ? Math.atan2(-dy, dx) : 0.0;
                api('/ctrl/navigate_to_pose', {rx: mapStartPos.rx, ry: mapStartPos.ry, yaw: yaw});
                mapTempLine = null;
                setMappingMode('idle');
                drawMapping();
            }
        }
        return true;
    }
    return false;
});

// --- 2. 巡邏模式互動 ---
let spTempLine=null;
let spStartPos = {rx:0, ry:0};

function isPointInPolygon(point, vs) {
    const x = point[0], y = point[1];
    let inside = false;
    for (let i = 0, j = vs.length - 1; i < vs.length; j = i++) {
        const xi = vs[i][0], yi = vs[i][1];
        const xj = vs[j][0], yj = vs[j][1];
        const intersect = ((yi > y) !== (yj > y)) &&
                          (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi);
        if (intersect) inside = !inside;
    }
    return inside;
}

function randomZoneColor() {
    return `rgba(${rn(255)}, ${rn(255)}, ${rn(255)}, 0.4)`;
}

function setEditToolsOpen(open) {
    const panel = document.getElementById('edit-tools-panel');
    if (!panel) return;
    panel.classList.toggle('hidden', !open);

    const btnEdit = document.getElementById('btn-edit');
    if (btnEdit) btnEdit.innerText = open ? "✏️ 編輯 ▴" : "✏️ 編輯 ▾";
}

function setToolActiveBtn(tool) {
    const ids = ['tool-rename','tool-split','tool-merge'];
    ids.forEach(id => document.getElementById(id)?.classList.remove('tool-active'));
    if (tool === 'RENAME') document.getElementById('tool-rename')?.classList.add('tool-active');
    if (tool === 'SPLIT')  document.getElementById('tool-split')?.classList.add('tool-active');
    if (tool === 'MERGE')  document.getElementById('tool-merge')?.classList.add('tool-active');
}

function setEditTool(tool) {
    if(!isEditMode) isEditMode = true;

    editTool = tool || 'RENAME';
    splitTarget = null;
    mergeSelected = [];
    editStartPos = null;
    editLine = null;

    updateSpBtn();
    renderAll();
}

function canvasRatioToWorld(rx, ry) {
    if (!zoneData) return null;

    const pixelX = rx * spCvs.width;
    const pixelY = ry * spCvs.height;

    const mapScale = spCvs.width / zoneData.info.width;
    const mapPx = pixelX / mapScale;
    const mapPy = pixelY / mapScale;

    const res = zoneData.info.resolution;
    const ox  = zoneData.info.origin[0];
    const oy  = zoneData.info.origin[1];
    const h   = zoneData.info.height;

    const wx = mapPx * res + ox;
    const wy = (h - mapPy) * res + oy;
    return {wx, wy};
}

function getZoneAtRatio(rx, ry) {
    if (!zoneData) return null;
    const w = canvasRatioToWorld(rx, ry);
    if (!w) return null;

    for (const [name, pts] of Object.entries(zoneData.zones)) {
        if (isPointInPolygon([w.wx, w.wy], pts)) return name;
    }
    return null;
}

function handleZoneClick(e) {
    if (!zoneData) return;

    const rect = spCvs.getBoundingClientRect();
    const clickX = e.clientX - rect.left;
    const clickY = e.clientY - rect.top;

    const scaleRatioX = spCvs.width / rect.width;
    const scaleRatioY = spCvs.height / rect.height;
    const pixelX = clickX * scaleRatioX;
    const pixelY = clickY * scaleRatioY;

    const mapScale = spCvs.width / zoneData.info.width;
    const mapPx = pixelX / mapScale;
    const mapPy = pixelY / mapScale;

    const res = zoneData.info.resolution;
    const ox  = zoneData.info.origin[0];
    const oy  = zoneData.info.origin[1];
    const h   = zoneData.info.height;

    const wx = mapPx * res + ox;
    const wy = (h - mapPy) * res + oy;

    for (const [name, pts] of Object.entries(zoneData.zones)) {
        if (isPointInPolygon([wx, wy], pts)) {
            if (isEditMode) {
                const newName = prompt(`重新命名 "${name}" 為:`, name);
                if (newName && newName.trim() && newName.trim() !== name) {
                    renameZoneRequest(name, newName.trim());
                }
            }
            return;
        }
    }
}

function renameZoneRequest(oldName, newName) {
    const nn = (newName || "").trim();
    if(!nn) { alert("分區名稱不可為空"); return; }
    if(nn === oldName) return;

    if(zoneData && zoneData.zones && zoneData.zones[nn]) {
        alert("已有同名分區，請換一個名稱");
        return;
    }

    fetch('/map/rename_zone', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ map_name: currentMapName, old_name: oldName, new_name: nn })
    })
    .then(r => r.json())
    .then(d => {
        if(d.success) {
            zoneData = d.data;
            if(zoneColors[oldName]) {
                zoneColors[nn] = zoneColors[oldName];
                delete zoneColors[oldName];
            } else {
                zoneColors[nn] = randomZoneColor();
            }
            renderAll();
        } else {
            alert("更名失敗: " + (d.error || "unknown"));
        }
    });
}

// ✅ 編輯模式：RENAME / SPLIT / MERGE（會吃掉 DOWN，避免旋轉）
initViewControl(spLayer, spTrans, 'sp-scene', (type, e, isClick) => {
    if (zoneData && isEditMode) {

        if (type === 'DOWN') {
            if (editTool === 'SPLIT' && splitTarget) {
                editStartPos = getPreciseRatio(e, spCvs);
                editLine = { x1: e.offsetX, y1: e.offsetY, x2: e.offsetX, y2: e.offsetY };
            }
            return true;
        }

        if (type === 'MOVE') {
            if (editTool === 'SPLIT' && editLine) {
                editLine.x2 = e.offsetX;
                editLine.y2 = e.offsetY;
                renderAll();
                drawArrow(spCtx, editLine.x1, editLine.y1, editLine.x2, editLine.y2);
            }
            return true;
        }

        if (type === 'UP') {
            const pos = getPreciseRatio(e, spCvs);

            // RENAME
            if (editTool === 'RENAME') {
                if (isClick) handleZoneClick(e);
                return true;
            }

            // MERGE：點兩個 zone
            if (editTool === 'MERGE') {
                if (!isClick) return true;
                const z = getZoneAtRatio(pos.rx, pos.ry);
                if (!z) return true;

                if (mergeSelected.includes(z)) mergeSelected = mergeSelected.filter(x => x !== z);
                else {
                    if (mergeSelected.length < 2) mergeSelected.push(z);
                }

                if (mergeSelected.length === 2) {
                    const [z1, z2] = mergeSelected;
                    const defName = `${z1}_${z2}`;
                    const newName = prompt(`合併 "${z1}" + "${z2}" → 新名稱：`, defName);
                    if (newName && newName.trim()) {
                        const nn = newName.trim();
                        fetch('/map/merge_zones', {
                            method:'POST',
                            headers:{'Content-Type':'application/json'},
                            body: JSON.stringify({
                                map_name: currentMapName,
                                zones: [z1, z2],
                                new_name: nn
                            })
                        })
                        .then(r=>r.json())
                        .then(d=>{
                            if(d.success){
                                zoneData = d.data;
                                const c = zoneColors[z1] || randomZoneColor();
                                delete zoneColors[z1];
                                delete zoneColors[z2];
                                zoneColors[nn] = c;

                                mergeSelected = [];
                                renderAll();
                                spStatus.innerText = "🔗 合併完成";
                            } else {
                                alert("合併失敗: " + (d.error || "unknown"));
                            }
                            updateSpBtn();
                        });
                    }
                    mergeSelected = [];
                }

                updateSpBtn();
                renderAll();
                return true;
            }

            // SPLIT：先點選目標，再拖曳畫切割線
            if (editTool === 'SPLIT') {
                if (isClick) {
                    const z = getZoneAtRatio(pos.rx, pos.ry);
                    if (z) {
                        splitTarget = z;
                        updateSpBtn();
                        renderAll();
                    }
                    return true;
                }

                if (editLine && editStartPos && splitTarget) {
                    const w1 = canvasRatioToWorld(editStartPos.rx, editStartPos.ry);
                    const w2 = canvasRatioToWorld(pos.rx, pos.ry);

                    const def1 = `${splitTarget}_1`;
                    const def2 = `${splitTarget}_2`;
                    const n1 = prompt(`分割後第一塊名稱：`, def1);
                    if (!n1 || !n1.trim()) { editLine=null; editStartPos=null; renderAll(); return true; }
                    const n2 = prompt(`分割後第二塊名稱：`, def2);
                    if (!n2 || !n2.trim()) { editLine=null; editStartPos=null; renderAll(); return true; }

                    const nn1 = n1.trim();
                    const nn2 = n2.trim();

                    fetch('/map/split_zone', {
                        method:'POST',
                        headers:{'Content-Type':'application/json'},
                        body: JSON.stringify({
                            map_name: currentMapName,
                            zone_name: splitTarget,
                            line: [[w1.wx, w1.wy], [w2.wx, w2.wy]],
                            new_names: [nn1, nn2]
                        })
                    })
                    .then(r=>r.json())
                    .then(d=>{
                        if(d.success){
                            const old = splitTarget;
                            zoneData = d.data;

                            const oldColor = zoneColors[old] || randomZoneColor();
                            delete zoneColors[old];

                            zoneColors[nn1] = oldColor;
                            zoneColors[nn2] = randomZoneColor();

                            splitTarget = null;
                            renderAll();
                            spStatus.innerText = "✂️ 分割完成";
                        } else {
                            alert("分割失敗: " + (d.error || "unknown"));
                        }
                        updateSpBtn();
                    });

                    editLine = null;
                    editStartPos = null;
                    renderAll();
                }
                return true;
            }

            return true;
        }

        return true;
    }

    // ---- 非編輯模式才走原本邏輯（定位/導航/錄製） ----
    if (spMode === 'NONE') return false;

    const pos = getPreciseRatio(e, spCvs);
    if (type === 'DOWN') {
        spStartPos = pos;
        spTempLine = { x1: e.offsetX, y1: e.offsetY, x2: e.offsetX, y2: e.offsetY };
        return true;
    } else if (type === 'MOVE') {
        if (spTempLine) { spTempLine.x2 = e.offsetX; spTempLine.y2 = e.offsetY; drawSmartPatrol(); }
    } else if (type === 'UP') {
        if (spTempLine) {
            const dx = spTempLine.x2 - spTempLine.x1;
            const dy = spTempLine.y2 - spTempLine.y1;
            const dist = Math.sqrt(dx*dx + dy*dy);
            let yaw = (dist > 10) ? Math.atan2(-dy, dx) : 0.0;

            if (spMode === 'SET_POSE') {
                api('/ctrl/set_pose', {rx: spStartPos.rx, ry: spStartPos.ry, yaw: yaw}, "定位設定中...");
                togglePoseMode();
            } else if (spMode === 'SET_GOAL') {
                api('/ctrl/navigate_to_pose', {rx: spStartPos.rx, ry: spStartPos.ry, yaw: yaw}, "導航中...");
                toggleNavMode();
            } else if (spMode === 'RECORD') {
                spWaypoints.push({rx: spStartPos.rx, ry: spStartPos.ry, yaw: yaw});
                renderSpMarkers();
                spStatus.innerText = `已錄製第 ${spWaypoints.length} 點`;
                updateSpBtn();
            }
            spTempLine = null;
            drawSmartPatrol();
        }
        return true;
    }
    return false;
});

function startMappingSystem() {
    document.getElementById('sys-status').innerText = "啟動建圖引擎...";
    fetch('/sys/start_mapping', {method:'POST'}).then(r => {
        if(r.ok) {
            currentMode = "MAPPING";
            switchView('view-mapping');

            // ✅ 切換模式時重置狀態（避免沿用上一張地圖尺寸/seq）
            mapSeq = -1;
            mapImg = null;
            mapInfo = null;

            mapTrans = {x:0, y:0, scale:1, rotate:0};
            applyTransform('map-scene', mapTrans);
            setMappingMode('manual');
            startLoop();
        }
    });
}

function setMappingMode(mode) {
    ['btn-auto', 'btn-man', 'btn-nav'].forEach(id => document.getElementById(id).classList.remove('active'));
    document.getElementById('dpad-zone').style.display = 'none';
    document.getElementById('map-interaction-layer').style.cursor = 'default';
    mappingMode = mode.toUpperCase();

    if(mode === 'auto') {
        document.getElementById('btn-auto').classList.add('active');
        document.getElementById('map-mode-text').innerText = "建圖：自動探索";
    } else if(mode === 'nav') {
        document.getElementById('btn-nav').classList.add('active');
        document.getElementById('map-mode-text').innerText = "建圖：拖曳設定目標與方向";
        document.getElementById('map-interaction-layer').style.cursor = 'crosshair';
    } else if(mode === 'manual') {
        document.getElementById('btn-man').classList.add('active');
        document.getElementById('dpad-zone').style.display = 'grid';
        document.getElementById('map-mode-text').innerText = "建圖：手動遙控";
    } else if(mode === 'idle') {
        document.getElementById('map-mode-text').innerText = "建圖模式：請選擇功能";
    }
    fetch('/sys/mapping_toggle', {
        method:'POST',
        headers:{'Content-Type':'application/json'},
        body:JSON.stringify({mode: mode})
    });
}

function saveMapPrompt() {
    const name = prompt("請輸入地圖名稱:", "map_" + Math.floor(Date.now()/1000));
    if(name) fetch('/map/save', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({name: name})})
        .then(async r=>alert(await r.text()));
}

// ✅ 地圖列表：新增「預覽」「更改地圖」按鈕（載入後不允許改名）
function showMapList() {
    fetch('/map/list').then(r=>r.json()).then(files => {
        const con = document.getElementById('map-list-container');
        con.innerHTML = "";
        if(!files || files.length === 0) {
            con.innerHTML = "<p>無地圖</p>";
            switchView('view-map-list');
            return;
        }

        files.forEach(mapName => {
            const row = document.createElement('div');
            row.className = 'map-item';

            const left = document.createElement('div');
            left.className = 'name';
            left.innerHTML = `<span>🗺️</span><span title="${escapeHtml(mapName)}">${escapeHtml(mapName)}</span>`;

            const actions = document.createElement('div');
            actions.className = 'map-actions';

            const btnPreview = document.createElement('button');
            btnPreview.className = 'btn-ghost';
            btnPreview.textContent = '預覽';
            btnPreview.onclick = () => previewMap(mapName);

            const btnRename = document.createElement('button');
            btnRename.className = 'btn-warn';
            btnRename.textContent = '更改地圖';
            btnRename.onclick = () => renameMapFromList(mapName);

            const btnLoad = document.createElement('button');
            btnLoad.className = 'btn-load';
            btnLoad.textContent = '載入';
            btnLoad.onclick = () => startPatrol(mapName);

            actions.appendChild(btnPreview);
            actions.appendChild(btnRename);
            actions.appendChild(btnLoad);

            row.appendChild(left);
            row.appendChild(actions);
            con.appendChild(row);
        });

        switchView('view-map-list');
    });
}

function escapeHtml(s){
    return String(s).replace(/[&<>"']/g, m => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[m]));
}

// ✅ 預覽：嘗試呼叫 /map/preview（若你後端沒有，就會提示失敗）
async function previewMap(mapName) {
    const modal = document.getElementById('preview-modal');
    const title = document.getElementById('preview-title');
    const img = document.getElementById('preview-img');

    title.textContent = `預覽：${mapName}`;
    img.src = "";
    modal.style.display = "flex";

    try {
        const r = await fetch('/map/preview', {
            method:'POST',
            headers:{'Content-Type':'application/json'},
            body: JSON.stringify({ name: mapName })
        });
        if(!r.ok) throw new Error(`HTTP ${r.status}`);

        const ct = (r.headers.get('content-type') || '').toLowerCase();

        if(ct.includes('application/json')) {
            const d = await r.json();
            if(d && d.success && (d.img || d.image || d.data)) {
                const b64 = d.img || d.image || d.data;
                img.src = "data:image/jpeg;base64," + b64;
            } else {
                throw new Error((d && d.error) ? d.error : "no image");
            }
        } else if(ct.startsWith('image/')) {
            const blob = await r.blob();
            img.src = URL.createObjectURL(blob);
        } else {
            const t = await r.text();
            throw new Error(t.slice(0,200));
        }
    } catch(e) {
        alert("預覽失敗（後端可能尚未提供 /map/preview）：\n" + e.message);
        document.getElementById('preview-modal').style.display = "none";
    }
}

function closePreview(e){
    if(e && e.target && e.target.id !== 'preview-modal') return;
    const modal = document.getElementById('preview-modal');
    const img = document.getElementById('preview-img');
    if(img && img.src && img.src.startsWith('blob:')) URL.revokeObjectURL(img.src);
    if(img) img.src = "";
    modal.style.display = "none";
}

// ✅ 更改地圖名稱：只在「選擇地圖」畫面提供（載入後不能改）
function renameMapFromList(oldName){
    const newName = prompt(`更改地圖名稱：\n${oldName} →`, oldName);
    if(!newName) return;
    const nn = newName.trim();
    if(!nn) { alert("名稱不可為空"); return; }
    if(nn === oldName) return;

    fetch('/map/rename_map', {
        method:'POST',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify({ old_name: oldName, new_name: nn })
    })
    .then(r=>r.json())
    .then(d=>{
        if(d.success){
            alert(`已更名：${oldName} → ${d.new_name || nn}`);
            showMapList(); // 重新整理列表
        } else {
            alert("改名失敗: " + (d.error || "unknown"));
        }
    });
}

// ✅ 作法 A：載入後等第一張地圖影像 ready 才跑分區
function startPatrol(mapName) {
    if(!confirm(`載入 ${mapName}？\n（載入後將無法在此模式改名）`)) return;
    currentMapName = mapName;
    spMapName.innerText = mapName;

    // ✅ 切換地圖時先重置狀態（避免沿用上一張地圖的 canvas 尺寸/seq）
    mapSeq = -1;
    mapImg = null;
    mapInfo = null;
    mapFrameReady = false;

    fetch('/sys/start_patrol', {
        method:'POST',
        headers:{'Content-Type':'application/json'},
        body:JSON.stringify({map: mapName})
    })
    .then(r => r.json())
    .then(d => {
        if(d.status === "OK") {
            currentMode = "PATROL";
            switchView('view-patrol');
            resetSpView();

            isRecording=false; isRunning=false; isPaused=false; spWaypoints=[];
            isEditMode = false;

            editTool = 'RENAME';
            splitTarget = null;
            mergeSelected = [];
            editStartPos = null;
            editLine = null;

            autoSegRequested = false;
            mapFrameReady = false;

            if (d.zones) {
                zoneData = d.zones;
                spStatus.innerText = "已載入分區";
                zoneColors = {};
                Object.keys(zoneData.zones).forEach(k => zoneColors[k] = randomZoneColor());
            } else {
                zoneData = null;
                spStatus.innerText = "載入地圖中…取得影像後將自動分區";
            }

            updateSpBtn();
            startLoop();
        } else {
            alert("載入失敗");
        }
    });
}

function resetSpView() {
    spTrans = {x:0, y:0, scale:1, rotate:0};
    applyTransform('sp-scene', spTrans);
}

function updateSpBtn() {
    const btnAction = document.getElementById('btn-action');
    const btnRec = document.getElementById('btn-rec');
    const btnTrash = document.getElementById('btn-trash');
    const btnEdit = document.getElementById('btn-edit');

    // ✅ 編輯模式：按鈕外觀 + 工具列折疊/展開
    if (isEditMode) btnEdit.classList.add('edit-active');
    else btnEdit.classList.remove('edit-active');

    setEditToolsOpen(isEditMode);
    setToolActiveBtn(editTool);

    // ✅ 編輯模式提示
    if (isEditMode) {
        if (editTool === 'RENAME') spStatus.innerText = "✏️ 編輯：點房間改名";
        if (editTool === 'SPLIT')  spStatus.innerText = splitTarget ? `✂️ 分割：已選 ${splitTarget}，拖曳畫切割線` : "✂️ 分割：先點選要分割的房間";
        if (editTool === 'MERGE')  spStatus.innerText = `🔗 合併：已選 ${mergeSelected.length}/2（點兩個房間）`;
    } else {
        editTool = 'RENAME';
        splitTarget = null;
        mergeSelected = [];
        editStartPos = null;
        editLine = null;
    }

    // ✅ 錄製 / 巡邏狀態（原本邏輯保留）
    if (isRecording) {
        btnRec.className = "sp-btn btn-red sp-recording";
        btnRec.innerText = "■";

        btnAction.className = "sp-btn btn-grey";
        btnAction.innerText = "錄製中...";

        btnTrash.classList.add('hidden');

        if (!isEditMode) spStatus.innerText = "錄製中：請拖曳設定點與方向";
    } else {
        btnRec.className = "sp-btn btn-grey";
        btnRec.innerText = "●";

        if (spWaypoints.length === 0) {
            btnAction.className = "sp-btn btn-grey";
            btnAction.innerText = "▶ 巡邏";
            btnTrash.classList.add('hidden');
        } else {
            btnTrash.classList.remove('hidden');

            if (isRunning) {
                btnAction.className = "sp-btn btn-red";
                btnAction.innerText = "❚❚ 暫停";
                if (!isEditMode) spStatus.innerText = "巡邏執行中...";
            } else {
                btnAction.className = "sp-btn btn-green";
                btnAction.innerText = isPaused ? "▶ 繼續" : "▶ 執行";
                if (!isEditMode) spStatus.innerText = isPaused ? "已暫停" : "就緒：可執行巡邏";
            }
        }
    }
}

function toggleEditMode() {
    if(isRunning || isRecording) { alert("請先停止巡邏或錄製"); return; }
    isEditMode = !isEditMode;

    if(isEditMode) {
        editTool = 'RENAME';
        splitTarget = null;
        mergeSelected = [];
        editStartPos = null;
        editLine = null;
    } else {
        setEditToolsOpen(false);
    }

    updateSpBtn();
    renderAll();
}

function toggleSpRecord() {
    if(isRunning) return;
    if(isEditMode) toggleEditMode();

    if(spMode === 'SET_POSE' || spMode === 'SET_GOAL') {
        document.getElementById('btn-pose').classList.remove('sp-active');
        document.getElementById('btn-goal').classList.remove('sp-active');
    }

    isRecording = !isRecording;
    spMode = isRecording ? 'RECORD' : 'NONE';
    document.getElementById('sp-interaction-layer').style.cursor = isRecording ? 'crosshair' : 'default';
    updateSpBtn();
}

function handlePatrolAction() {
    if(isRecording || spWaypoints.length === 0) return;
    if(isEditMode) toggleEditMode();

    if(isRunning) {
        api('/ctrl/patrol_pause', {}, "暫停中...");
        isRunning = false; isPaused = true;
    } else {
        if (isPaused) api('/ctrl/patrol_resume', {}, "繼續巡邏...");
        else api('/ctrl/patrol_start', {pts: spWaypoints}, "開始巡邏...");
        isRunning = true; isPaused = false;
    }
    updateSpBtn();
}

function clearSpPoints() {
    if(!confirm("確定清除所有巡邏點？")) return;
    spWaypoints = [];
    isRunning = false; isPaused = false;
    api('/ctrl/patrol_stop', {}, "已清除");
    renderSpMarkers();
    updateSpBtn();
}

function togglePoseMode() {
    if(isRecording || isRunning) return;
    if(isEditMode) toggleEditMode();

    if(spMode === 'SET_POSE') {
        spMode='NONE';
        document.getElementById('btn-pose').classList.remove('sp-active');
    } else {
        spMode='SET_POSE';
        document.getElementById('btn-pose').classList.add('sp-active');
        document.getElementById('btn-goal').classList.remove('sp-active');
    }
    document.getElementById('sp-interaction-layer').style.cursor = (spMode==='SET_POSE')?'crosshair':'default';
}

function toggleNavMode() {
    if(isRecording || isRunning) return;
    if(isEditMode) toggleEditMode();

    if(spMode === 'SET_GOAL') {
        spMode='NONE';
        document.getElementById('btn-goal').classList.remove('sp-active');
    } else {
        spMode='SET_GOAL';
        document.getElementById('btn-goal').classList.add('sp-active');
        document.getElementById('btn-pose').classList.remove('sp-active');
    }
    document.getElementById('sp-interaction-layer').style.cursor = (spMode==='SET_GOAL')?'crosshair':'default';
}

function renderSpMarkers() {
    const layer = document.getElementById('sp-marker-layer');
    layer.innerHTML = '';
    spWaypoints.forEach((p, i) => {
        const m = document.createElement('div');
        m.className = 'marker';
        m.innerText = i+1;
        m.style.left = (p.rx * 100) + '%';
        m.style.top = (p.ry * 100) + '%';
        layer.appendChild(m);
    });
}

function toggleCamera() {
    const win = document.getElementById('camera-window');
    if(!isCameraOn) {
        fetch('/cam/on', {method:'POST'});
        win.style.display='flex';
        document.getElementById('camera-feed').src = '/cam/feed?' + Date.now();
        isCameraOn = true;
    } else {
        fetch('/cam/off', {method:'POST'});
        win.style.display='none';
        document.getElementById('camera-feed').src = '';
        isCameraOn = false;
    }
}

function api(url, data, msg) {
    if(msg) spStatus.innerText = msg;
    fetch(url, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(data)});
}

function stopSystem(force=false) {
    if(!force && !confirm("確定退出？")) return;

    fetch('/sys/stop', {method:'POST'});
    clearInterval(loopTimer);

    mapImg=null;
    spWaypoints=[];
    isRecording=false; isRunning=false; isPaused=false;

    zoneData = null;
    autoSegRequested = false;
    mapFrameReady = false;

    isEditMode = false;
    editTool = 'RENAME';
    splitTarget = null;
    mergeSelected = [];
    editStartPos = null;
    editLine = null;

    currentMapName = "";
    mapSeq = -1;
    robotState = null;
    robotPath = [];

    updateSpBtn();
    goHome();
}

function triggerAutoSegment(silent = false) {
    if (!currentMapName) return;
    if (!silent) spStatus.innerText = "運算中...";

    fetch('/map/auto_segment', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ map_name: currentMapName })
    })
    .then(r => r.json())
    .then(d => {
        if(d.success) {
            zoneData = d.data;
            spStatus.innerText = "分割完成！";
            zoneColors = {};
            Object.keys(zoneData.zones).forEach(k => zoneColors[k] = randomZoneColor());
            if (mapFrameReady) renderAll();
        } else {
            if(!silent) alert("分割失敗: " + (d.error || "unknown"));
        }
    });
}

function rn(max){ return Math.floor(Math.random()*max); }

function drawZones(ctx) {
    if (!zoneData) return;

    const scale = spCvs.width / zoneData.info.width;
    const res = zoneData.info.resolution;
    const ox  = zoneData.info.origin[0];
    const oy  = zoneData.info.origin[1];
    const h   = zoneData.info.height;

    ctx.save();
    for (const [name, pts] of Object.entries(zoneData.zones)) {
        ctx.beginPath();
        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity;

        pts.forEach((pt, i) => {
            let px = (pt[0] - ox) / res;
            let py = (pt[1] - oy) / res;
            py = h - py;

            let dx = px * scale;
            let dy = py * scale;

            if(dx < minX) minX = dx; if(dx > maxX) maxX = dx;
            if(dy < minY) minY = dy; if(dy > maxY) maxY = dy;

            if(i===0) ctx.moveTo(dx, dy);
            else ctx.lineTo(dx, dy);
        });

        ctx.closePath();
        ctx.fillStyle = zoneColors[name] || "rgba(255,255,255,0.15)";
        ctx.fill();

        const isSel = (name === splitTarget) || (mergeSelected.includes(name));
        ctx.strokeStyle = isSel ? "rgba(255,215,0,0.95)" : "rgba(255,255,255,0.8)";
        ctx.lineWidth = isSel ? 3 : 1;
        ctx.stroke();

        if(pts.length > 0) {
            let centerX = (minX + maxX) / 2;
            let centerY = (minY + maxY) / 2;
            ctx.textAlign = "center";
            ctx.textBaseline = "middle";
            ctx.font = "bold 14px Arial";
            ctx.strokeStyle = "black";
            ctx.lineWidth = 3;
            ctx.strokeText(name, centerX, centerY);
            ctx.fillStyle = "white";
            ctx.fillText(name, centerX, centerY);
        }
    }
    ctx.restore();
}

function startLoop() {
    if(loopTimer) clearInterval(loopTimer);
    loopTimer = setInterval(async ()=>{
        try {
            const res = await fetch(`/data?seq=${mapSeq}`);
            const d = await res.json();

            robotState = d.rob;
            robotPath = d.path || [];

            if(d.upd) {
                const img = new Image();
                img.onload = ()=>{
                    mapImg=img;
                    mapInfo=d.info;
                    mapSeq=d.info.seq;

                    // ✅ 修正：寬或高任一變化都要更新 Canvas 尺寸（避免黑邊/裁切）
                    if (mapCvs.width !== img.width || mapCvs.height !== img.height) {
                        mapCvs.width = img.width;
                        mapCvs.height = img.height;
                    }
                    if (spCvs.width !== img.width || spCvs.height !== img.height) {
                        spCvs.width = img.width;
                        spCvs.height = img.height;
                    }

                    renderAll();

                    // ✅ 作法 A：第一張影像 ready → 立刻跑分區（只跑一次）
                    mapFrameReady = true;
                    if (currentMode === "PATROL" && !zoneData && !autoSegRequested) {
                        autoSegRequested = true;
                        spStatus.innerText = "自動分區中…";
                        triggerAutoSegment(true);
                    }
                };
                img.src = "data:image/jpeg;base64,"+d.img;
            } else if(mapImg) {
                renderAll();
            }
        } catch(e) {}
    }, 500);
}

function renderAll() {
    if(currentMode==="MAPPING") drawMapping();
    else if(currentMode==="PATROL") {
        drawSmartPatrol();
        if(zoneData) drawZones(spCtx);

        // SPLIT 進行中：畫切割線
        if (isEditMode && editTool === 'SPLIT' && editLine) {
            drawArrow(spCtx, editLine.x1, editLine.y1, editLine.x2, editLine.y2);
        }
    }
}

function drawPath(ctx) {
    if(robotPath.length>0){
        ctx.beginPath();
        ctx.strokeStyle='#00e676';
        ctx.lineWidth=2;
        ctx.moveTo(robotPath[0][0],robotPath[0][1]);
        for(let p of robotPath) ctx.lineTo(p[0],p[1]);
        ctx.stroke();
    }
}

function drawMapping() {
    mapCtx.clearRect(0,0,mapCvs.width,mapCvs.height);
    if(mapImg) mapCtx.drawImage(mapImg,0,0);
    drawPath(mapCtx);
    drawRobot(mapCtx);
    if(mapTempLine) drawArrow(mapCtx,mapTempLine.x1,mapTempLine.y1,mapTempLine.x2,mapTempLine.y2);
}

function drawSmartPatrol() {
    spCtx.clearRect(0,0,spCvs.width,spCvs.height);
    if(mapImg) spCtx.drawImage(mapImg,0,0);
    drawPath(spCtx);
    drawRobot(spCtx);
    if(spTempLine)drawArrow(spCtx,spTempLine.x1,spTempLine.y1,spTempLine.x2,spTempLine.y2);
}

function drawRobot(ctx) {
    if(robotState && mapInfo){
        const i=mapInfo;
        const px=(robotState.x-i.ox)/i.res;
        const py=i.h-1-(robotState.y-i.oy)/i.res;
        ctx.save();
        ctx.translate(px,py);
        ctx.rotate(-robotState.yaw);
        ctx.beginPath();
        ctx.arc(0,0,8,0,Math.PI*2);
        ctx.fillStyle='#2979ff';
        ctx.fill();
        ctx.strokeStyle='white';
        ctx.lineWidth=2;
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(0,0);
        ctx.lineTo(15,0);
        ctx.strokeStyle='#ffeb3b';
        ctx.lineWidth=3;
        ctx.stroke();
        ctx.restore();
    }
}

function drawArrow(ctx,x1,y1,x2,y2){
    ctx.beginPath();
    ctx.lineWidth=4;
    ctx.strokeStyle='#ff5252';
    const head=15;
    const angle=Math.atan2(y2-y1,x2-x1);
    ctx.moveTo(x1,y1);
    ctx.lineTo(x2,y2);
    ctx.lineTo(x2-head*Math.cos(angle-Math.PI/6),y2-head*Math.sin(angle-Math.PI/6));
    ctx.moveTo(x2,y2);
    ctx.lineTo(x2-head*Math.cos(angle+Math.PI/6),y2-head*Math.sin(angle+Math.PI/6));
    ctx.stroke();
}

function mv(l,a){
    fetch('/ctrl/move',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({l:l,a:a})});
}
function stop(){mv(0,0);}
