/**
 * LiDAR Simulator Server - Multi-threaded with Worker Threads
 * 
 * 改善点:
 * - Worker Threadsによるシミュレーション処理のマルチスレッド化
 * - メインスレッドはSocket.IO通信のみ担当
 * - 各クライアント用に専用Workerスレッドを生成
 * - sample-map.jpg をデフォルトマップとして使用
 */
const express = require('express');
const path = require('path');
const fs = require('fs');
const http = require('http');
const { Server } = require('socket.io');
const multer = require('multer');
const sharp = require('sharp');
const { Worker } = require('worker_threads');
const os = require('os');

// CPU情報
const NUM_CPUS = os.cpus().length;

// デフォルトマップデータをグローバルに保持
let defaultMapData = null;

// デフォルトマップを読み込む関数
async function loadDefaultMap() {
    try {
        // sample-map.jpg を優先的に探す
        const jpgPath = path.join(__dirname, 'sample-map.jpg');
        const pngPath = path.join(__dirname, 'sample-map.png');
        
        let mapPath = null;
        if (fs.existsSync(jpgPath)) {
            mapPath = jpgPath;
        } else if (fs.existsSync(pngPath)) {
            mapPath = pngPath;
        }
        
        if (!mapPath) {
            console.log('⚠️  Default map (sample-map.jpg/png) not found, using procedural map');
            return null;
        }
        
        console.log(`📂 Loading default map from: ${mapPath}`);
        
        const image = sharp(mapPath);
        const metadata = await image.metadata();
        
        let width = metadata.width;
        let height = metadata.height;
        
        // キャンバスサイズに合わせてスケーリング
        const canvasWidth = 1200;
        const canvasHeight = 800;
        
        // アスペクト比を維持してリサイズ
        const scale = Math.min(canvasWidth / width, canvasHeight / height);
        width = Math.floor(width * scale);
        height = Math.floor(height * scale);
        
        // グレースケールに変換してrawデータを取得
        const { data } = await image
            .greyscale()
            .resize(width, height)
            .raw()
            .toBuffer({ resolveWithObject: true });
        
        // グレースケール値を抽出
        const grayscaleData = new Uint8Array(width * height);
        for (let i = 0; i < width * height; i++) {
            grayscaleData[i] = data[i];
        }
        
        console.log(`✅ Default map loaded: ${width}x${height}`);
        
        return {
            width,
            height,
            data: grayscaleData
        };
    } catch (error) {
        console.error('❌ Error loading default map:', error);
        return null;
    }
}

// ===============================
// Express & Socket.IO Setup
// ===============================
const app = express();
const server = http.createServer(app);
const io = new Server(server, {
    maxHttpBufferSize: 10 * 1024 * 1024, // 10MB
    cors: { origin: "*" }
});
const PORT = 3000;

// ファイルアップロード設定
const upload = multer({ 
    storage: multer.memoryStorage(),
    limits: { fileSize: 10 * 1024 * 1024 }
});

// 静的ファイルを提供
app.use(express.static(__dirname));

// ルートパスでclient.htmlを提供
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'client.html'));
});

// robot-config.jsonを取得するエンドポイント
app.get('/api/config', (req, res) => {
    try {
        const configPath = path.join(__dirname, 'robot-config.json');
        if (fs.existsSync(configPath)) {
            const config = JSON.parse(fs.readFileSync(configPath, 'utf8'));
            res.json(config);
        } else {
            res.status(404).json({ error: 'Config not found' });
        }
    } catch (error) {
        res.status(500).json({ error: error.message });
    }
});

// robot-config.jsonを更新するエンドポイント
app.post('/api/config', express.json(), (req, res) => {
    try {
        const configPath = path.join(__dirname, 'robot-config.json');
        fs.writeFileSync(configPath, JSON.stringify(req.body, null, 2));
        res.json({ success: true });
    } catch (error) {
        res.status(500).json({ error: error.message });
    }
});

// 画像アップロードエンドポイント
app.post('/upload-map', upload.single('mapImage'), async (req, res) => {
    try {
        if (!req.file) {
            return res.status(400).json({ success: false, message: 'No file uploaded' });
        }
        
        console.log(`📂 Processing uploaded image: ${req.file.originalname}`);
        
        const image = sharp(req.file.buffer);
        const metadata = await image.metadata();
        
        let width = metadata.width;
        let height = metadata.height;
        
        // キャンバスサイズに合わせてリサイズ
        const canvasWidth = 1200;
        const canvasHeight = 800;
        const scale = Math.min(canvasWidth / width, canvasHeight / height);
        width = Math.floor(width * scale);
        height = Math.floor(height * scale);
        
        const { data } = await image
            .greyscale()
            .resize(width, height)
            .raw()
            .toBuffer({ resolveWithObject: true });
        
        const grayscaleData = new Uint8Array(width * height);
        for (let i = 0; i < width * height; i++) {
            grayscaleData[i] = data[i];
        }
        
        console.log(`✅ Image processed: ${width}x${height}`);
        
        res.json({ 
            success: true, 
            imageData: { width, height, data: Array.from(grayscaleData) }
        });
    } catch (error) {
        console.error('❌ Error processing map image:', error);
        res.status(500).json({ success: false, message: error.message });
    }
});

// ===============================
// Worker Thread Management
// ===============================
const workers = new Map();

function createWorkerForClient(socketId, mapData) {
    const worker = new Worker('./simulation-worker.js', {
        workerData: {
            socketId,
            mapData: mapData ? {
                width: mapData.width,
                height: mapData.height,
                data: Array.from(mapData.data)
            } : null,
            canvasWidth: 1200,
            canvasHeight: 800
        }
    });
    
    // Workerからのメッセージを処理
    worker.on('message', (message) => {
        const socket = io.sockets.sockets.get(socketId);
        if (!socket) return;
        
        switch (message.type) {
            case 'init':
                socket.emit('init', message.data);
                break;
            case 'state':
                socket.emit('state', message.data);
                break;
            case 'goalResult':
                socket.emit('goalResult', message.data);
                break;
            case 'initPoseResult':
                socket.emit('initPoseResult', message.data);
                break;
            case 'kidnapResult':
                socket.emit('kidnapResult', message.data);
                break;
            case 'globalLocalizationResult':
                socket.emit('globalLocalizationResult', message.data);
                break;
            case 'mapLoaded':
                socket.emit('mapLoaded', message.data);
                break;
            case 'log':
                console.log(`[Worker ${socketId.substring(0, 8)}] ${message.data}`);
                break;
        }
    });
    
    worker.on('error', (error) => {
        console.error(`Worker error for ${socketId}:`, error);
    });
    
    worker.on('exit', (code) => {
        if (code !== 0) {
            console.log(`Worker for ${socketId} exited with code ${code}`);
        }
        workers.delete(socketId);
    });
    
    workers.set(socketId, worker);
    return worker;
}

// ===============================
// Socket.IO Connection Handling
// ===============================
io.on('connection', (socket) => {
    console.log(`🔌 Client connected: ${socket.id}`);
    
    // このクライアント用のWorkerスレッドを作成
    const worker = createWorkerForClient(socket.id, defaultMapData);
    
    // クライアントからのコマンドをWorkerに転送
    socket.on('setGoal', (data) => {
        worker.postMessage({ type: 'setGoal', data });
    });
    
    socket.on('setInitPose', (data) => {
        worker.postMessage({ type: 'setInitPose', data });
    });
    
    socket.on('kidnap', () => {
        worker.postMessage({ type: 'kidnap' });
    });
    
    socket.on('globalLocalization', () => {
        worker.postMessage({ type: 'globalLocalization' });
    });
    
    socket.on('loadMap', (data) => {
        worker.postMessage({ type: 'loadMap', data });
    });
    
    socket.on('reset', () => {
        worker.postMessage({ type: 'reset' });
    });
    
    socket.on('disconnect', () => {
        console.log(`🔌 Client disconnected: ${socket.id}`);
        const worker = workers.get(socket.id);
        if (worker) {
            worker.postMessage({ type: 'shutdown' });
            worker.terminate();
            workers.delete(socket.id);
        }
    });
});

// ===============================
// Server Startup
// ===============================
async function startServer() {
    console.log('\n🚀 Initializing LiDAR Simulator (Multi-threaded)...');
    console.log(`   Node.js version: ${process.version}`);
    console.log(`   Platform: ${process.platform}`);
    console.log(`   CPU cores: ${NUM_CPUS}`);
    
    // デフォルトマップを読み込み
    defaultMapData = await loadDefaultMap();
    
    server.listen(PORT, () => {
        console.log(`\n🌐 LiDAR Simulator is running on http://localhost:${PORT}`);
        
        if (defaultMapData) {
            console.log(`📍 Using default map: sample-map.jpg (${defaultMapData.width}x${defaultMapData.height})`);
        } else {
            console.log(`📍 Using procedural map (no default map found)`);
        }
        
        console.log(`\n✨ Multi-threading enabled:`);
        console.log(`   ✓ Each client gets a dedicated Worker thread`);
        console.log(`   ✓ Simulation runs in parallel`);
        console.log(`   ✓ Main thread handles only Socket.IO`);
        console.log(`\nPress Ctrl+C to stop the server\n`);
    });
}

startServer().catch(console.error);
