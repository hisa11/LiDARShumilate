/**
 * Simulation Worker Thread - Lightweight Version
 * 
 * 各クライアント接続に対して専用のWorkerスレッドで
 * シミュレーション処理を実行する
 * 
 * 最適化: 10Hz更新、軽量パーティクルフィルタ
 */
const { parentPort, workerData } = require('worker_threads');
const { SimulationEngine, METERS_TO_PIXELS, ROBOT_CONF, setScale } = require('./simulation-engine');

// Worker初期化データ
const { socketId, mapData, canvasWidth, canvasHeight } = workerData;

// マップデータを復元
let imageData = null;
if (mapData) {
    imageData = {
        width: mapData.width,
        height: mapData.height,
        data: new Uint8Array(mapData.data)
    };
    
    // スケール設定: 画像サイズからメートル換算
    const fieldWidthM = mapData.fieldWidthM || (mapData.width / 100);
    const fieldHeightM = mapData.fieldHeightM || (mapData.height / 100);
    setScale(mapData.width, mapData.height, fieldWidthM, fieldHeightM);
}

// シミュレーションエンジン初期化
let sim = new SimulationEngine(canvasWidth, canvasHeight, imageData);
let isRunning = true;
let updateIntervalId = null;

// ログ送信
function log(message) {
    parentPort.postMessage({ type: 'log', data: message });
}

// 初期状態を送信
function sendInitialState() {
    const state = sim.getState();
    const initialPoseInfo = sim.getInitialPoseInfo();
    
    log(`Initial pose: x=${initialPoseInfo.x_m.toFixed(2)}m, y=${initialPoseInfo.y_m.toFixed(2)}m, θ=${initialPoseInfo.theta_deg.toFixed(1)}°`);
    
    parentPort.postMessage({
        type: 'init',
        data: {
            ...state,
            initialPose: initialPoseInfo
        }
    });
}

// 状態を送信
function sendState() {
    if (!isRunning) return;
    
    parentPort.postMessage({
        type: 'state',
        data: sim.getState()
    });
}

// シミュレーションループ開始
function startSimulationLoop() {
    // 10Hz (100ms間隔) でシミュレーション更新 - 軽量化
    updateIntervalId = setInterval(() => {
        if (!isRunning) return;
        
        try {
            sim.update(0.1); // 100ms = 0.1s
            sendState();
        } catch (error) {
            log(`Update error: ${error.message}`);
        }
    }, 100);
}

// メインスレッドからのメッセージを処理
parentPort.on('message', (message) => {
    switch (message.type) {
        case 'setGoal':
            handleSetGoal(message.data);
            break;
            
        case 'setInitPose':
            handleSetInitPose(message.data);
            break;
            
        case 'kidnap':
            handleKidnap();
            break;
            
        case 'globalLocalization':
            handleGlobalLocalization();
            break;
            
        case 'loadMap':
            handleLoadMap(message.data);
            break;
            
        case 'reset':
            handleReset();
            break;
            
        case 'shutdown':
            handleShutdown();
            break;
    }
});

// ゴール設定処理
function handleSetGoal(data) {
    log(`Planning path to (${data.x}, ${data.y})`);
    
    const startTime = Date.now();
    const result = sim.setGoal(data.x, data.y);
    const elapsed = Date.now() - startTime;
    
    log(`Path planning completed in ${elapsed}ms`);
    
    parentPort.postMessage({
        type: 'goalResult',
        data: result
    });
}

// 初期位置設定処理
function handleSetInitPose(data) {
    log(`Setting initial pose: (${data.x}, ${data.y}, ${data.theta})`);
    
    const result = sim.setInitialPose(data.x, data.y, data.theta);
    
    parentPort.postMessage({
        type: 'initPoseResult',
        data: result
    });
}

// キドナップ処理（ロボットをランダム位置に移動）
function handleKidnap() {
    log('Kidnapping robot...');
    
    const result = sim.kidnap();
    
    parentPort.postMessage({
        type: 'kidnapResult',
        data: result
    });
}

// グローバルローカライゼーション処理
function handleGlobalLocalization() {
    log('Performing global localization...');
    
    const result = sim.globalLocalization();
    
    parentPort.postMessage({
        type: 'globalLocalizationResult',
        data: result
    });
}

// マップ読み込み処理
function handleLoadMap(data) {
    log(`Loading new map: ${data.width}x${data.height}`);
    
    try {
        // 新しいマップデータを構築
        const newImageData = {
            width: data.width,
            height: data.height,
            data: new Uint8Array(data.data)
        };
        
        // シミュレーションを再初期化
        sim = new SimulationEngine(canvasWidth, canvasHeight, newImageData);
        
        // 初期状態を送信
        sendInitialState();
        
        parentPort.postMessage({
            type: 'mapLoaded',
            data: { success: true }
        });
    } catch (error) {
        log(`Error loading map: ${error.message}`);
        parentPort.postMessage({
            type: 'mapLoaded',
            data: { success: false, message: error.message }
        });
    }
}

// リセット処理
function handleReset() {
    log('Resetting simulation...');
    
    // 現在のマップデータで再初期化
    sim = new SimulationEngine(canvasWidth, canvasHeight, imageData);
    
    // 初期状態を送信
    sendInitialState();
}

// シャットダウン処理
function handleShutdown() {
    log('Shutting down worker...');
    isRunning = false;
    
    if (updateIntervalId) {
        clearInterval(updateIntervalId);
        updateIntervalId = null;
    }
}

// Worker起動
log('Worker started');
sendInitialState();
startSimulationLoop();
