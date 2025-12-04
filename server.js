const express = require('express');
const path = require('path');
const fs = require('fs');
const http = require('http');
const { Server } = require('socket.io');
const multer = require('multer');
const sharp = require('sharp');
const { SimulationEngine } = require('./simulation-engine');

// デフォルトマップの画像パス
const DEFAULT_MAP_PATH = path.join(__dirname, 'sample-map.png');

// デフォルトマップデータをグローバルに保持
let defaultMapData = null;

// デフォルトマップを読み込む関数
async function loadDefaultMap() {
    try {
        if (!fs.existsSync(DEFAULT_MAP_PATH)) {
            console.log('Default map not found, using procedural map');
            return null;
        }
        
        console.log(`Loading default map from: ${DEFAULT_MAP_PATH}`);
        
        const image = sharp(DEFAULT_MAP_PATH);
        const metadata = await image.metadata();
        
        let width = metadata.width;
        let height = metadata.height;
        
        // 画像が大きすぎる場合は縮小
        const maxWidth = 800;
        const maxHeight = 600;
        let resizeNeeded = false;
        
        if (width > maxWidth || height > maxHeight) {
            const scale = Math.min(maxWidth / width, maxHeight / height);
            width = Math.floor(width * scale);
            height = Math.floor(height * scale);
            resizeNeeded = true;
        }
        
        // グレースケールに変換してrawデータを取得
        let processor = image.greyscale();
        
        if (resizeNeeded) {
            processor = processor.resize(width, height);
        }
        
        const { data, info } = await processor
            .raw()
            .toBuffer({ resolveWithObject: true });
        
        // グレースケール値を抽出
        const grayscaleData = new Uint8Array(width * height);
        for (let i = 0; i < width * height; i++) {
            grayscaleData[i] = data[i];
        }
        
        console.log(`Default map loaded: ${width}x${height}`);
        
        return {
            width,
            height,
            data: grayscaleData
        };
    } catch (error) {
        console.error('Error loading default map:', error);
        return null;
    }
}

const app = express();
const server = http.createServer(app);
const io = new Server(server, {
    maxHttpBufferSize: 10 * 1024 * 1024, // 10MB
    cors: {
        origin: "*"
    }
});
const PORT = 3000;

// ファイルアップロード設定
const upload = multer({ 
    storage: multer.memoryStorage(),
    limits: { fileSize: 10 * 1024 * 1024 } // 10MB
});

// 静的ファイルを提供
app.use(express.static(__dirname));

// ルートパスでclient.htmlを提供
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'client.html'));
});

// 画像アップロードエンドポイント
app.post('/upload-map', upload.single('mapImage'), async (req, res) => {
    try {
        if (!req.file) {
            return res.status(400).json({ success: false, message: 'No file uploaded' });
        }
        
        console.log(`Processing uploaded image: ${req.file.originalname} (${req.file.mimetype})`);
        
        // Sharpで画像を読み込み、グレースケールに変換
        // 画像が大きすぎる場合は縮小（最大800x600）
        const image = sharp(req.file.buffer);
        const metadata = await image.metadata();
        
        let width = metadata.width;
        let height = metadata.height;
        
        // 画像が大きすぎる場合は縮小
        const maxWidth = 800;
        const maxHeight = 600;
        let resizeNeeded = false;
        
        if (width > maxWidth || height > maxHeight) {
            const scale = Math.min(maxWidth / width, maxHeight / height);
            width = Math.floor(width * scale);
            height = Math.floor(height * scale);
            resizeNeeded = true;
        }
        
        // グレースケールに変換してrawデータを取得
        let processor = image.greyscale();
        
        if (resizeNeeded) {
            processor = processor.resize(width, height);
        }
        
        const { data, info } = await processor
            .raw()
            .toBuffer({ resolveWithObject: true });
        
        // グレースケール値を抽出（1チャンネル）
        const grayscaleData = new Uint8Array(width * height);
        for (let i = 0; i < width * height; i++) {
            grayscaleData[i] = data[i];
        }
        
        console.log(`Image processed: ${width}x${height}`);
        
        res.json({ 
            success: true, 
            imageData: {
                width,
                height,
                data: Array.from(grayscaleData)
            }
        });
    } catch (error) {
        console.error('Error processing map image:', error);
        res.status(500).json({ success: false, message: error.message });
    }
});

// クライアント接続管理
const simulations = new Map();
const updateIntervals = new Map();

io.on('connection', (socket) => {
    console.log('Client connected:', socket.id);
    
    // 新しいシミュレーションエンジンを作成（デフォルトマップを使用）
    const sim = new SimulationEngine(1200, 800, defaultMapData);
    simulations.set(socket.id, sim);
    
    // 初期状態を送信
    socket.emit('init', sim.getState());
    
    // シミュレーションループ (50ms = 20Hz)
    const updateInterval = setInterval(() => {
        const sim = simulations.get(socket.id);
        if (sim) {
            sim.update(0.05); // dt = 50ms
            socket.emit('state', sim.getState());
        }
    }, 50);
    updateIntervals.set(socket.id, updateInterval);
    
    // クライアントからのコマンド処理
    socket.on('setGoal', async (data) => {
        const sim = simulations.get(socket.id);
        if (sim) {
            console.log(`Planning path to (${data.x}, ${data.y}) for client ${socket.id}`);
            // 非同期で実行してメインループをブロックしない
            setImmediate(() => {
                try {
                    const result = sim.setGoal(data.x, data.y);
                    socket.emit('goalResult', result);
                } catch (error) {
                    console.error('Path planning error:', error);
                    socket.emit('goalResult', { success: false, message: error.message });
                }
            });
        }
    });
    
    socket.on('setInitPose', (data) => {
        const sim = simulations.get(socket.id);
        if (sim) {
            const result = sim.setInitialPose(data.x, data.y, data.theta);
            socket.emit('initPoseResult', result);
        }
    });
    
    socket.on('kidnap', () => {
        const sim = simulations.get(socket.id);
        if (sim) {
            const result = sim.kidnap();
            socket.emit('kidnapResult', result);
        }
    });
    
    // グローバルローカライゼーション（マップ全体で自己位置を探索）
    socket.on('globalLocalization', () => {
        const sim = simulations.get(socket.id);
        if (sim) {
            console.log(`Performing global localization for client ${socket.id}`);
            const result = sim.globalLocalization();
            socket.emit('globalLocalizationResult', result);
        }
    });
    
    socket.on('reset', (data) => {
        const sim = simulations.get(socket.id);
        if (sim) {
            // リセット時もデフォルトマップを使用
            const newSim = new SimulationEngine(data.width || 1200, data.height || 800, defaultMapData);
            simulations.set(socket.id, newSim);
            socket.emit('init', newSim.getState());
        }
    });
    
    socket.on('loadMapImage', (imageData) => {
        console.log(`Loading map from image (${imageData.width}x${imageData.height}) for client ${socket.id}`);
        console.log(`Image data array length: ${imageData.data.length}`);
        try {
            // 新しいシミュレーションを画像データから作成
            const canvasWidth = 1200;
            const canvasHeight = 800;
            
            console.log('Creating new simulation engine with image data...');
            const newSim = new SimulationEngine(canvasWidth, canvasHeight, {
                width: imageData.width,
                height: imageData.height,
                data: new Uint8Array(imageData.data)
            });
            
            console.log(`Simulation created with ${newSim.world.walls.length} walls`);
            simulations.set(socket.id, newSim);
            socket.emit('init', newSim.getState());
            socket.emit('mapLoadResult', { success: true });
            
            console.log(`Map loaded successfully for client ${socket.id}`);
        } catch (error) {
            console.error('Error loading map image:', error);
            console.error('Error stack:', error.stack);
            socket.emit('mapLoadResult', { success: false, message: error.message });
        }
    });
    
    // 切断処理
    socket.on('disconnect', () => {
        console.log('Client disconnected:', socket.id);
        const interval = updateIntervals.get(socket.id);
        if (interval) {
            clearInterval(interval);
            updateIntervals.delete(socket.id);
        }
        simulations.delete(socket.id);
    });
});

// サーバー起動前にデフォルトマップを読み込む
(async () => {
    console.log('Initializing LiDAR Simulator...');
    defaultMapData = await loadDefaultMap();
    
    server.listen(PORT, () => {
        console.log(`🚀 LiDAR Simulator is running on http://localhost:${PORT}`);
        if (defaultMapData) {
            console.log(`📍 Using default map from: ${DEFAULT_MAP_PATH}`);
        } else {
            console.log('📍 Using procedural map (no default map found)');
        }
        console.log('Press Ctrl+C to stop the server');
    });
})();
