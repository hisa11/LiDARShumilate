/**
 * LiDAR Simulation Engine (Omni-wheel Ready) - Optimized Version
 * 
 * 修正内容:
 * - 自己位置推定のバグ修正
 * - A*経路計画の最適化（フリーズ防止）
 * - パーティクルフィルタの軽量化
 * - 4輪オムニホイール対応の実用化準備
 */
const fs = require('fs');
const path = require('path');

// 設定読み込み
const CONFIG_PATH = path.join(__dirname, 'robot-config.json');
let ROBOT_CONF = {
    robot: { radiusM: 0.15 },
    lidar: { offsetX_M: 0, offsetY_M: 0, offsetTheta_Rad: 0, numRays: 450, maxRangeM: 12.0, minRangeM: 0.02 },
    kinematics: { maxSpeedMps: 1.0, maxRotationRadps: 2.0 },
    safety: { wallMarginM: 0.30, minPassageWidthM: 0.45 },
    localization: { 
        particleCount: 300, 
        particleLidarRays: 36,
        initialPoseUncertaintyM: 0.15,
        initialPoseUncertaintyRad: 0.1,
        motionNoiseXY: 0.03,
        motionNoiseTheta: 0.02,
        sensorNoisePx: 0.8
    },
    control: {
        lookAheadDistM: 0.4,
        goalToleranceM: 0.05,
        linearPGain: 2.0,
        angularPGain: 3.0
    }
};

try {
    if (fs.existsSync(CONFIG_PATH)) {
        const loadedConf = JSON.parse(fs.readFileSync(CONFIG_PATH, 'utf8'));
        // 新しい設定形式に対応しつつ、古い形式もサポート
        if (loadedConf.robot) {
            ROBOT_CONF = loadedConf;
        } else {
            // 旧形式から変換
            ROBOT_CONF.robot = { radiusM: loadedConf.radiusM || 0.15 };
            ROBOT_CONF.lidar = loadedConf.lidar || ROBOT_CONF.lidar;
            ROBOT_CONF.kinematics = loadedConf.kinematics || ROBOT_CONF.kinematics;
            ROBOT_CONF.safety = loadedConf.safety || ROBOT_CONF.safety;
        }
        console.log('Loaded robot config:', JSON.stringify(ROBOT_CONF, null, 2));
    }
} catch (e) {
    console.error('Failed to load robot config, using defaults.', e);
}

const METERS_TO_PIXELS = 40;
const GRID_RES = 0.10; // 10cm grid
const PIXELS_PER_GRID = GRID_RES * METERS_TO_PIXELS;

// シミュレーション設定（robot-config.jsonから上書き）
const SIM_CONF = {
    particleCount: ROBOT_CONF.localization?.particleCount || 300,
    particleLidarRays: ROBOT_CONF.localization?.particleLidarRays || 36,
    dt: 0.05,
    noise: {
        odom_xy: ROBOT_CONF.localization?.motionNoiseXY || 0.03,
        odom_theta: ROBOT_CONF.localization?.motionNoiseTheta || 0.02,
        lidar: (ROBOT_CONF.localization?.sensorNoisePx || 0.8)
    }
};

// --- Math Utilities ---
class MathUtils {
    static normalizeAngle(angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    static gaussianRandom(mean, variance) {
        if (variance <= 0) return mean;
        let u = 0, v = 0;
        while(u === 0) u = Math.random();
        while(v === 0) v = Math.random();
        const num = Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v);
        return num * variance + mean;
    }

    static dist(p1, p2) {
        return Math.hypot(p2.x - p1.x, p2.y - p1.y);
    }

    // 座標変換: ローカル(ロボット座標) -> グローバル
    static localToGlobal(lx, ly, poseX, poseY, poseTheta) {
        const cos = Math.cos(poseTheta);
        const sin = Math.sin(poseTheta);
        return {
            x: poseX + (lx * cos - ly * sin),
            y: poseY + (lx * sin + ly * cos)
        };
    }

    static intersect(x1, y1, x2, y2, x3, y3, x4, y4) {
        const den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (Math.abs(den) < 1e-10) return null;
        const t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
        const u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
            return { x: x1 + t * (x2 - x1), y: y1 + t * (y2 - y1) };
        }
        return null;
    }
}

// --- Priority Queue for A* (Binary Heap) ---
class PriorityQueue {
    constructor() {
        this.heap = [];
        this.indexMap = new Map(); // index -> heap position
    }
    
    isEmpty() {
        return this.heap.length === 0;
    }
    
    push(index, priority) {
        const node = { index, priority };
        this.heap.push(node);
        this.indexMap.set(index, this.heap.length - 1);
        this._bubbleUp(this.heap.length - 1);
    }
    
    pop() {
        if (this.heap.length === 0) return null;
        const result = this.heap[0];
        this.indexMap.delete(result.index);
        
        if (this.heap.length > 1) {
            this.heap[0] = this.heap.pop();
            this.indexMap.set(this.heap[0].index, 0);
            this._bubbleDown(0);
        } else {
            this.heap.pop();
        }
        return result;
    }
    
    has(index) {
        return this.indexMap.has(index);
    }
    
    updatePriority(index, newPriority) {
        const pos = this.indexMap.get(index);
        if (pos === undefined) return false;
        
        const oldPriority = this.heap[pos].priority;
        this.heap[pos].priority = newPriority;
        
        if (newPriority < oldPriority) {
            this._bubbleUp(pos);
        } else {
            this._bubbleDown(pos);
        }
        return true;
    }
    
    _bubbleUp(pos) {
        while (pos > 0) {
            const parent = Math.floor((pos - 1) / 2);
            if (this.heap[parent].priority <= this.heap[pos].priority) break;
            this._swap(parent, pos);
            pos = parent;
        }
    }
    
    _bubbleDown(pos) {
        const len = this.heap.length;
        while (true) {
            const left = 2 * pos + 1;
            const right = 2 * pos + 2;
            let smallest = pos;
            
            if (left < len && this.heap[left].priority < this.heap[smallest].priority) {
                smallest = left;
            }
            if (right < len && this.heap[right].priority < this.heap[smallest].priority) {
                smallest = right;
            }
            if (smallest === pos) break;
            this._swap(pos, smallest);
            pos = smallest;
        }
    }
    
    _swap(i, j) {
        const temp = this.heap[i];
        this.heap[i] = this.heap[j];
        this.heap[j] = temp;
        this.indexMap.set(this.heap[i].index, i);
        this.indexMap.set(this.heap[j].index, j);
    }
}

// --- Costmap & Planner (最適化版) ---
class Costmap {
    constructor(widthPx, heightPx, walls) {
        this.widthPx = widthPx;
        this.heightPx = heightPx;
        this.cols = Math.ceil(widthPx / PIXELS_PER_GRID);
        this.rows = Math.ceil(heightPx / PIXELS_PER_GRID);
        this.distMap = new Float32Array(this.cols * this.rows).fill(999.0);
        this.isWall = new Uint8Array(this.cols * this.rows).fill(0);
        this.buildMap(walls);
    }
    
    buildMap(walls) {
        // 壁をラスタライズ
        for (let w of walls) this.rasterizeLine(w.p1, w.p2);
        
        // 距離マップを構築 (BFS)
        const totalPixels = this.cols * this.rows;
        const queue = new Int32Array(totalPixels);
        let head = 0, tail = 0;
        
        for (let i = 0; i < totalPixels; i++) {
            if (this.isWall[i]) { 
                this.distMap[i] = 0; 
                queue[tail++] = i; 
            }
        }
        
        const offsets = [-1, 1, -this.cols, this.cols];
        
        while(head < tail) {
            const currIdx = queue[head++];
            const cx = currIdx % this.cols;
            const cy = Math.floor(currIdx / this.cols);
            const currentDist = this.distMap[currIdx];
            
            for(const offset of offsets) {
                const neighborIdx = currIdx + offset;
                if (neighborIdx < 0 || neighborIdx >= totalPixels) continue;
                
                const nx = neighborIdx % this.cols;
                const ny = Math.floor(neighborIdx / this.cols);
                
                // 行の境界チェック
                if (Math.abs(nx - cx) > 1) continue;
                if (ny < 0 || ny >= this.rows) continue;
                
                if (this.distMap[neighborIdx] > currentDist + GRID_RES) {
                    this.distMap[neighborIdx] = currentDist + GRID_RES;
                    queue[tail++] = neighborIdx;
                }
            }
        }
    }
    
    rasterizeLine(p1, p2) {
        let x0 = Math.floor(p1.x / PIXELS_PER_GRID), y0 = Math.floor(p1.y / PIXELS_PER_GRID);
        let x1 = Math.floor(p2.x / PIXELS_PER_GRID), y1 = Math.floor(p2.y / PIXELS_PER_GRID);
        
        // クランプ
        x0 = Math.max(0, Math.min(this.cols - 1, x0));
        y0 = Math.max(0, Math.min(this.rows - 1, y0));
        x1 = Math.max(0, Math.min(this.cols - 1, x1));
        y1 = Math.max(0, Math.min(this.rows - 1, y1));
        
        const dx = Math.abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        const dy = -Math.abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        let err = dx + dy;
        
        while (true) {
            if (x0 >= 0 && x0 < this.cols && y0 >= 0 && y0 < this.rows) {
                this.isWall[y0 * this.cols + x0] = 1;
            }
            if (x0 === x1 && y0 === y1) break;
            const e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }
    
    getCost(idx) {
        if (idx < 0 || idx >= this.distMap.length) return 255;
        const dist = this.distMap[idx];
        const robotRadius = (ROBOT_CONF.robot?.radiusM || 0.15);
        
        // 壁セルは通行不可
        if (this.isWall[idx]) return 255;
        
        // ロボット半径より近いセルは通行不可（少しマージンを緩める）
        if (dist < robotRadius * 0.6) return 255;
        
        // コスト計算（壁に近いほど高コスト）
        const safeDist = 0.5; // 安全距離 (m) - 少し緩める
        if (dist > safeDist) return 1;
        
        const norm = Math.max(0, (safeDist - dist) / safeDist);
        return Math.floor(norm * 50) + 1; // コストの上限を下げる
    }
    
    isValid(idx) {
        return idx >= 0 && idx < this.distMap.length && this.getCost(idx) < 255;
    }
}

class GlobalPlanner {
    constructor(costmap) { 
        this.cm = costmap;
        this.maxIterations = 100000; // 十分な探索を許可
    }
    
    plan(startPx, goalPx) {
        const sx = Math.floor(startPx.x / PIXELS_PER_GRID);
        const sy = Math.floor(startPx.y / PIXELS_PER_GRID);
        const gx = Math.floor(goalPx.x / PIXELS_PER_GRID);
        const gy = Math.floor(goalPx.y / PIXELS_PER_GRID);
        
        // 境界チェック
        if (sx < 0 || sx >= this.cm.cols || sy < 0 || sy >= this.cm.rows) {
            console.log('Start position out of bounds');
            return null;
        }
        if (gx < 0 || gx >= this.cm.cols || gy < 0 || gy >= this.cm.rows) {
            console.log('Goal position out of bounds');
            return null;
        }
        
        const startIdx = sy * this.cm.cols + sx;
        const goalIdx = gy * this.cm.cols + gx;
        
        if (!this.cm.isValid(goalIdx)) {
            console.log('Goal is in obstacle');
            return null;
        }
        if (!this.cm.isValid(startIdx)) {
            console.log('Start is in obstacle');
            return null;
        }
        
        // A* with Priority Queue
        const openSet = new PriorityQueue();
        const closedSet = new Set();
        const cameFrom = new Map();
        const gScore = new Map();
        
        gScore.set(startIdx, 0);
        openSet.push(startIdx, this.heuristic(sx, sy, gx, gy));
        
        // 8方向移動
        const neighbors = [
            { dx: -1, dy: 0, cost: 1.0 },
            { dx: 1, dy: 0, cost: 1.0 },
            { dx: 0, dy: -1, cost: 1.0 },
            { dx: 0, dy: 1, cost: 1.0 },
            { dx: -1, dy: -1, cost: 1.414 },
            { dx: 1, dy: -1, cost: 1.414 },
            { dx: -1, dy: 1, cost: 1.414 },
            { dx: 1, dy: 1, cost: 1.414 }
        ];
        
        let iterations = 0;
        
        while (!openSet.isEmpty() && iterations++ < this.maxIterations) {
            const current = openSet.pop();
            if (!current) break;
            
            const currentIdx = current.index;
            
            if (currentIdx === goalIdx) {
                console.log(`Path found in ${iterations} iterations`);
                return this.reconstructPath(cameFrom, currentIdx);
            }
            
            if (closedSet.has(currentIdx)) continue;
            closedSet.add(currentIdx);
            
            const cx = currentIdx % this.cm.cols;
            const cy = Math.floor(currentIdx / this.cm.cols);
            const currentG = gScore.get(currentIdx) || Infinity;
            
            for (const n of neighbors) {
                const nx = cx + n.dx;
                const ny = cy + n.dy;
                
                if (nx < 0 || nx >= this.cm.cols || ny < 0 || ny >= this.cm.rows) continue;
                
                const neighborIdx = ny * this.cm.cols + nx;
                
                if (closedSet.has(neighborIdx)) continue;
                
                const cellCost = this.cm.getCost(neighborIdx);
                if (cellCost >= 255) continue;
                
                const tentativeG = currentG + n.cost + (cellCost * 0.02);
                const existingG = gScore.get(neighborIdx);
                
                if (existingG === undefined || tentativeG < existingG) {
                    cameFrom.set(neighborIdx, currentIdx);
                    gScore.set(neighborIdx, tentativeG);
                    
                    const fScore = tentativeG + this.heuristic(nx, ny, gx, gy);
                    
                    if (openSet.has(neighborIdx)) {
                        openSet.updatePriority(neighborIdx, fScore);
                    } else {
                        openSet.push(neighborIdx, fScore);
                    }
                }
            }
        }
        
        console.log(`Path not found after ${iterations} iterations`);
        return null;
    }
    
    heuristic(x1, y1, x2, y2) { 
        // オクタイル距離（8方向移動に最適）
        const dx = Math.abs(x2 - x1);
        const dy = Math.abs(y2 - y1);
        return Math.max(dx, dy) + 0.414 * Math.min(dx, dy);
    }
    
    reconstructPath(cameFrom, current) {
        const totalPath = [this.idxToPoint(current)];
        while (cameFrom.has(current)) {
            current = cameFrom.get(current);
            totalPath.unshift(this.idxToPoint(current));
        }
        
        // パスの平滑化（ジグザグ解消）
        return this.smoothPath(totalPath);
    }
    
    /**
     * パス平滑化 - Line of Sight (LOS) アルゴリズム
     * 直線で結べる点をスキップして滑らかなパスを生成
     */
    smoothPath(path) {
        if (path.length < 3) return path;
        
        const smoothed = [path[0]];
        let i = 0;
        
        while (i < path.length - 1) {
            // できるだけ遠くの点まで直線で見通せるか確認
            let furthest = i + 1;
            
            for (let j = path.length - 1; j > i + 1; j--) {
                if (this.hasLineOfSight(path[i], path[j])) {
                    furthest = j;
                    break;
                }
            }
            
            smoothed.push(path[furthest]);
            i = furthest;
        }
        
        return smoothed;
    }
    
    /**
     * 2点間に障害物がないか確認（Line of Sight）
     */
    hasLineOfSight(p1, p2) {
        const dx = p2.x - p1.x;
        const dy = p2.y - p1.y;
        const dist = Math.hypot(dx, dy);
        const steps = Math.ceil(dist / (PIXELS_PER_GRID * 0.5));
        
        if (steps === 0) return true;
        
        const stepX = dx / steps;
        const stepY = dy / steps;
        
        // ロボット半径分のマージンを考慮
        const robotRadius = (ROBOT_CONF.robot?.radiusM || 0.15);
        
        for (let i = 1; i < steps; i++) {
            const x = p1.x + stepX * i;
            const y = p1.y + stepY * i;
            
            const gx = Math.floor(x / PIXELS_PER_GRID);
            const gy = Math.floor(y / PIXELS_PER_GRID);
            
            if (gx < 0 || gx >= this.cm.cols || gy < 0 || gy >= this.cm.rows) {
                return false;
            }
            
            const idx = gy * this.cm.cols + gx;
            const dist = this.cm.distMap[idx];
            
            // 壁からの距離がロボット半径より小さい場合は通過不可
            if (dist < robotRadius) {
                return false;
            }
        }
        
        return true;
    }
    
    // 旧 simplifyPath は削除（smoothPath に置き換え）
    
    idxToPoint(idx) {
        return {
            x: (idx % this.cm.cols) * PIXELS_PER_GRID + (PIXELS_PER_GRID/2),
            y: Math.floor(idx / this.cm.cols) * PIXELS_PER_GRID + (PIXELS_PER_GRID/2)
        };
    }
}

// --- World Map ---
class WorldMap {
    constructor(width, height, imageData = null) {
        this.width = width;
        this.height = height;
        this.walls = [];
        if (imageData) {
            this.createWallsFromImage(imageData);
        } else {
            this.createWalls();
        }
    }
    
    createWallsFromImage(imageData) {
        const { width: imgW, height: imgH, data } = imageData;
        
        // 画像座標からキャンバス座標へのスケール
        const scaleX = this.width / imgW;
        const scaleY = this.height / imgH;
        
        // 外壁を追加
        this.walls.push({p1: {x: 0, y: 0}, p2: {x: this.width, y: 0}});
        this.walls.push({p1: {x: this.width, y: 0}, p2: {x: this.width, y: this.height}});
        this.walls.push({p1: {x: this.width, y: this.height}, p2: {x: 0, y: this.height}});
        this.walls.push({p1: {x: 0, y: this.height}, p2: {x: 0, y: 0}});
        
        // 黒いピクセルを壁として検出
        const threshold = 128;
        
        // エッジ検出による壁生成
        for (let y = 0; y < imgH - 1; y++) {
            for (let x = 0; x < imgW - 1; x++) {
                const idx = y * imgW + x;
                const current = data[idx];
                const right = data[idx + 1];
                const down = data[idx + imgW];
                
                // 水平エッジ
                if ((current < threshold) !== (right < threshold)) {
                    const px = (x + 1) * scaleX;
                    const py1 = y * scaleY;
                    const py2 = (y + 1) * scaleY;
                    this.walls.push({p1: {x: px, y: py1}, p2: {x: px, y: py2}});
                }
                
                // 垂直エッジ
                if ((current < threshold) !== (down < threshold)) {
                    const px1 = x * scaleX;
                    const px2 = (x + 1) * scaleX;
                    const py = (y + 1) * scaleY;
                    this.walls.push({p1: {x: px1, y: py}, p2: {x: px2, y: py}});
                }
            }
        }
        
        console.log(`Created ${this.walls.length} wall segments from image`);
    }
    
    createWalls() {
        const w = this.width, h = this.height;
        // 外壁
        this.walls.push({p1: {x: 0, y: 0}, p2: {x: w, y: 0}});
        this.walls.push({p1: {x: w, y: 0}, p2: {x: w, y: h}});
        this.walls.push({p1: {x: w, y: h}, p2: {x: 0, y: h}});
        this.walls.push({p1: {x: 0, y: h}, p2: {x: 0, y: 0}});
        // サンプル障害物
        this.walls.push({p1: {x: w*0.2, y: h*0.2}, p2: {x: w*0.8, y: h*0.2}});
        this.walls.push({p1: {x: w*0.5, y: h*0.4}, p2: {x: w*0.5, y: h*0.8}});
    }
}

// --- Lidar (オフセット対応・最適化版) ---
class Lidar {
    constructor(numRays, maxRange) {
        this.numRays = numRays;
        this.maxRange = maxRange;
        this.fov = Math.PI * 2;
        
        // Configからオフセット取得 (メートル -> ピクセル)
        this.offsetX = (ROBOT_CONF.lidar?.offsetX_M || 0) * METERS_TO_PIXELS;
        this.offsetY = (ROBOT_CONF.lidar?.offsetY_M || 0) * METERS_TO_PIXELS;
        this.offsetTheta = ROBOT_CONF.lidar?.offsetTheta_Rad || 0;
        
        // 事前計算: 角度テーブル
        this.angleStep = this.fov / this.numRays;
        this.cosTable = new Float32Array(this.numRays);
        this.sinTable = new Float32Array(this.numRays);
        
        for (let i = 0; i < this.numRays; i++) {
            const angle = i * this.angleStep;
            this.cosTable[i] = Math.cos(angle);
            this.sinTable[i] = Math.sin(angle);
        }
    }
    
    // pose: ロボットの中心座標と向き
    scan(pose, walls, noiseStdDev = 0) {
        const ranges = new Float32Array(this.numRays);
        
        // LiDARの実際の位置を計算 (ロボット座標系 -> グローバル座標系)
        const cos0 = Math.cos(pose.theta);
        const sin0 = Math.sin(pose.theta);
        
        const lidarX = pose.x + (this.offsetX * cos0 - this.offsetY * sin0);
        const lidarY = pose.y + (this.offsetX * sin0 + this.offsetY * cos0);
        const lidarTheta = pose.theta + this.offsetTheta;
        
        // 基準角度の三角関数
        const cosBase = Math.cos(lidarTheta);
        const sinBase = Math.sin(lidarTheta);
        
        for (let i = 0; i < this.numRays; i++) {
            // 事前計算した角度テーブルを使用
            const localCos = this.cosTable[i];
            const localSin = this.sinTable[i];
            
            // グローバル方向ベクトル（回転行列の合成）
            const dirX = localCos * cosBase - localSin * sinBase;
            const dirY = localCos * sinBase + localSin * cosBase;
            
            const rayEndX = lidarX + dirX * this.maxRange;
            const rayEndY = lidarY + dirY * this.maxRange;

            let minDist = this.maxRange;
            
            for (let j = 0, len = walls.length; j < len; j++) {
                const w = walls[j];
                const hit = MathUtils.intersect(
                    w.p1.x, w.p1.y, w.p2.x, w.p2.y, 
                    lidarX, lidarY, rayEndX, rayEndY
                );
                if (hit) {
                    const d = Math.hypot(hit.x - lidarX, hit.y - lidarY);
                    if (d < minDist) minDist = d;
                }
            }
            
            if (noiseStdDev > 0 && minDist < this.maxRange) {
                minDist += MathUtils.gaussianRandom(0, noiseStdDev);
                const minRange = (ROBOT_CONF.lidar?.minRangeM || 0.02) * METERS_TO_PIXELS;
                if (minDist < minRange) minDist = minRange;
            }
            ranges[i] = minDist;
        }
        return ranges;
    }
}

// --- Robot (オムニホイール対応) ---
class Robot {
    constructor(x, y, theta, canvasWidth, canvasHeight, useImageMap = false) {
        this.pose = { x, y, theta };
        this.radius = (ROBOT_CONF.robot?.radiusM || 0.15) * METERS_TO_PIXELS;
        
        // LiDAR設定
        const lidarRays = useImageMap ? 180 : (ROBOT_CONF.lidar?.numRays || 450);
        this.lidar = new Lidar(lidarRays, (ROBOT_CONF.lidar?.maxRangeM || 12.0) * METERS_TO_PIXELS);
        
        this.path = null;
        this.pathIndex = 0;
        this.canvasWidth = canvasWidth;
        this.canvasHeight = canvasHeight;
        
        // 速度状態（実用化用）
        this.velocity = { vx: 0, vy: 0, omega: 0 };
    }

    /**
     * オムニホイール移動
     * @param {number} vx - ロボット前方方向の速度 (m/s)
     * @param {number} vy - ロボット左方向の速度 (m/s)
     * @param {number} omega - 角速度 (rad/s)
     * @param {number} dt - 時間刻み
     * @param {Array} walls - 衝突判定用の壁
     */
    move(vx, vy, omega, dt, walls = []) {
        // 現在の速度を保存
        this.velocity = { vx, vy, omega };
        
        // ロボット座標系での移動量 (m)
        const dx_local = vx * dt;
        const dy_local = vy * dt;
        const dtheta = omega * dt;

        // グローバル座標系への変換
        const cos = Math.cos(this.pose.theta);
        const sin = Math.sin(this.pose.theta);
        
        const globalDx = dx_local * cos - dy_local * sin;
        const globalDy = dx_local * sin + dy_local * cos;

        // 次の位置を計算
        const nextX = this.pose.x + globalDx * METERS_TO_PIXELS;
        const nextY = this.pose.y + globalDy * METERS_TO_PIXELS;
        const nextTheta = MathUtils.normalizeAngle(this.pose.theta + dtheta);

        // 簡易衝突判定
        let collision = false;
        if (nextX < this.radius || nextX > this.canvasWidth - this.radius ||
            nextY < this.radius || nextY > this.canvasHeight - this.radius) {
            collision = true;
        }
        
        if (!collision) {
            this.pose.x = nextX;
            this.pose.y = nextY;
            this.pose.theta = nextTheta;
        }

        // オドメトリ（真の移動量を返す）
        // 注意: ノイズはパーティクルフィルタの予測ステップで追加する
        // ここでノイズを追加するとパーティクルフィルタで二重にノイズが加わる
        return {
            dx: dx_local,       // ローカル座標系での移動量 (m)
            dy: dy_local,       // ローカル座標系での移動量 (m)
            dtheta: dtheta      // 角度変化 (rad)
        };
    }
}

// --- Particle Filter (オムニ対応・最適化版) ---
class ParticleFilter {
    constructor(count, mapWidth, mapHeight, knownMap, useImageMap = false) {
        this.count = count;
        this.width = mapWidth;
        this.height = mapHeight;
        this.map = knownMap;
        
        const particleLidarRays = useImageMap ? 24 : SIM_CONF.particleLidarRays;
        this.simLidar = new Lidar(particleLidarRays, (ROBOT_CONF.lidar?.maxRangeM || 12.0) * METERS_TO_PIXELS);
        this.particles = [];
        
        // 初期位置が設定されるまでは初期化しない
        this.initialized = false;
    }
    
    initGlobal() {
        this.particles = [];
        for (let i = 0; i < this.count; i++) {
            this.particles.push({
                x: Math.random() * this.width,
                y: Math.random() * this.height,
                theta: Math.random() * Math.PI * 2,
                weight: 1.0 / this.count
            });
        }
        this.initialized = true;
    }
    
    /**
     * 初期位置を設定（重要: 自己位置推定バグの修正ポイント）
     */
    setEstimatedPose(x, y, theta) {
        this.particles = [];
        
        // 設定ファイルから初期分散を取得
        const xyStdDev = (ROBOT_CONF.localization?.initialPoseUncertaintyM || 0.15) * METERS_TO_PIXELS;
        const thetaStdDev = ROBOT_CONF.localization?.initialPoseUncertaintyRad || 0.1;
        
        for (let i = 0; i < this.count; i++) {
            this.particles.push({
                x: x + MathUtils.gaussianRandom(0, xyStdDev),
                y: y + MathUtils.gaussianRandom(0, xyStdDev),
                theta: MathUtils.normalizeAngle(theta + MathUtils.gaussianRandom(0, thetaStdDev)),
                weight: 1.0 / this.count
            });
        }
        this.initialized = true;
    }
    
    /**
     * 予測ステップ (Motion Model) - バグ修正版
     * @param {Object} odomDelta - {dx, dy, dtheta} ローカル座標系での移動量 (m, rad)
     */
    predict(odomDelta) {
        if (!this.initialized) return;
        
        const dDist = Math.hypot(odomDelta.dx, odomDelta.dy);
        
        // オドメトリの不確かさモデル:
        // - 移動量に比例するノイズ（スリップなど）
        // - 最小ノイズ（センサー精度の限界）
        const linearNoiseRatio = SIM_CONF.noise.odom_xy;  // 移動量に対する割合
        const angularNoiseRatio = SIM_CONF.noise.odom_theta;
        
        // ノイズの標準偏差（ピクセル単位に変換）
        const distStdDev = Math.max(dDist * linearNoiseRatio * METERS_TO_PIXELS, 0.3);
        const angStdDev = Math.max(Math.abs(odomDelta.dtheta) * angularNoiseRatio, 0.005);

        for (let p of this.particles) {
            // ローカル座標での移動量にノイズを追加
            const noisyLx = odomDelta.dx * METERS_TO_PIXELS + MathUtils.gaussianRandom(0, distStdDev);
            const noisyLy = odomDelta.dy * METERS_TO_PIXELS + MathUtils.gaussianRandom(0, distStdDev);
            const noisyTh = odomDelta.dtheta + MathUtils.gaussianRandom(0, angStdDev);

            // パーティクルの現在の向きでグローバル座標系へ変換
            const cos = Math.cos(p.theta);
            const sin = Math.sin(p.theta);

            p.x += (noisyLx * cos - noisyLy * sin);
            p.y += (noisyLx * sin + noisyLy * cos);
            p.theta = MathUtils.normalizeAngle(p.theta + noisyTh);
        }
    }
    
    /**
     * 更新ステップ (Sensor Model) - バグ修正版
     * @param {Array} realScan - 実際のLiDARスキャンデータ（ピクセル単位）
     */
    update(realScan) {
        if (!this.initialized) return;
        
        // 実スキャンとシミュレーションスキャンの角度対応を正確に
        const realNumRays = realScan.length;
        const simNumRays = this.simLidar.numRays;
        
        // センサーモデルのパラメータ
        // sigma: スキャンの許容誤差（ピクセル単位）
        const sigma = 15.0; // 標準偏差 15ピクセル ≈ 0.375m（少し緩めに）
        const sigma2 = 2.0 * sigma * sigma;

        let totalWeight = 0;
        let maxWeight = 0;
        
        for (let p of this.particles) {
            // マップ外判定
            if (p.x < 10 || p.x > this.width - 10 || p.y < 10 || p.y > this.height - 10) {
                p.weight = 1e-10;
                totalWeight += p.weight;
                continue;
            }

            // このパーティクル位置・向きでのLiDARスキャンをシミュレート
            // パーティクルをposeオブジェクトとして渡す
            const hypoScan = this.simLidar.scan({ x: p.x, y: p.y, theta: p.theta }, this.map, 0);
            
            let logLikelihood = 0;
            let validPoints = 0;
            
            // 各シミュレーションレイに対応する実スキャンの角度を計算
            for (let i = 0; i < simNumRays; i++) {
                // シミュレーションレイの角度インデックスに対応する実スキャンのインデックス
                const realIdx = Math.round(i * realNumRays / simNumRays) % realNumRays;
                const realDist = realScan[realIdx];
                const hypoDist = hypoScan[i];
                
                // 最大距離付近はスキップ（信頼性が低い）
                if (realDist >= this.simLidar.maxRange * 0.90) continue;
                if (hypoDist >= this.simLidar.maxRange * 0.90) continue;
                
                // 短距離（ノイズが多い）もスキップ
                const minRange = 20; // 0.5m
                if (realDist < minRange || hypoDist < minRange) continue;
                
                const diff = realDist - hypoDist;
                logLikelihood += -(diff * diff) / sigma2;
                validPoints++;
            }
            
            if (validPoints >= 5) {
                // 平均対数尤度を使用（レイ数に依存しない）
                p.weight = Math.exp(logLikelihood / validPoints);
            } else {
                p.weight = 1e-10;
            }
            
            totalWeight += p.weight;
            if (p.weight > maxWeight) maxWeight = p.weight;
        }
        
        // 正規化
        if (totalWeight > 0) {
            for (let p of this.particles) {
                p.weight /= totalWeight;
            }
        } else {
            // 全滅した場合 - 推定位置周辺に再配置
            console.warn('All particles have zero weight, redistributing around estimate...');
            const est = this.getEstimate();
            for (let i = 0; i < this.count; i++) {
                this.particles[i] = {
                    x: est.x + MathUtils.gaussianRandom(0, 50),
                    y: est.y + MathUtils.gaussianRandom(0, 50),
                    theta: est.theta + MathUtils.gaussianRandom(0, 0.3),
                    weight: 1.0 / this.count
                };
            }
        }
    }
    
    /**
     * リサンプリング - 低分散リサンプリング
     */
    resample() {
        if (!this.initialized) return;
        
        const newParticles = [];
        
        // ESS (Effective Sample Size) の計算
        let sumSqWeight = 0;
        for (let p of this.particles) {
            sumSqWeight += p.weight * p.weight;
        }
        const ess = 1.0 / (sumSqWeight * this.count);
        
        // ESSが閾値以上ならリサンプリングをスキップ
        if (ess > 0.5) {
            return;
        }
        
        // 低分散リサンプリング
        const r = Math.random() / this.count;
        let c = this.particles[0].weight;
        let i = 0;
        
        // ランダムパーティクル注入数
        const randomCount = Math.floor(this.count * 0.03);
        const mainCount = this.count - randomCount;
        
        for (let m = 0; m < mainCount; m++) {
            const u = r + m / this.count;
            while (c < u && i < this.count - 1) {
                i++;
                c += this.particles[i].weight;
            }
            
            const p = this.particles[i];
            // 少しジッターを加えて多様性を維持
            newParticles.push({
                x: p.x + MathUtils.gaussianRandom(0, 0.5),
                y: p.y + MathUtils.gaussianRandom(0, 0.5),
                theta: MathUtils.normalizeAngle(p.theta + MathUtils.gaussianRandom(0, 0.01)),
                weight: 1.0 / this.count
            });
        }
        
        // ランダムパーティクル注入（位置ロスト回復用）
        for (let j = 0; j < randomCount; j++) {
            const est = this.getEstimate();
            newParticles.push({
                x: est.x + MathUtils.gaussianRandom(0, 2 * METERS_TO_PIXELS),
                y: est.y + MathUtils.gaussianRandom(0, 2 * METERS_TO_PIXELS),
                theta: Math.random() * Math.PI * 2,
                weight: 1.0 / this.count
            });
        }
        
        this.particles = newParticles;
    }
    
    getEstimate() {
        if (!this.initialized || this.particles.length === 0) {
            return { x: this.width / 2, y: this.height / 2, theta: 0 };
        }
        
        let x = 0, y = 0, sinSum = 0, cosSum = 0, totalW = 0;
        
        for (let p of this.particles) {
            x += p.x * p.weight;
            y += p.y * p.weight;
            sinSum += Math.sin(p.theta) * p.weight;
            cosSum += Math.cos(p.theta) * p.weight;
            totalW += p.weight;
        }
        
        if (totalW === 0) {
            return this.particles[0];
        }
        
        return {
            x: x / totalW,
            y: y / totalW,
            theta: Math.atan2(sinSum / totalW, cosSum / totalW)
        };
    }
}

// --- Simulation State Manager ---
class SimulationEngine {
    constructor(canvasWidth, canvasHeight, imageData = null, initialPose = null) {
        this.canvasWidth = canvasWidth;
        this.canvasHeight = canvasHeight;
        this.useImageMap = imageData !== null;
        
        this.world = new WorldMap(canvasWidth, canvasHeight, imageData);
        this.costmap = new Costmap(canvasWidth, canvasHeight, this.world.walls);
        this.planner = new GlobalPlanner(this.costmap);
        
        // 初期位置の設定（安全な位置を自動的に見つける）
        let startX, startY, startTheta;
        if (initialPose) {
            startX = initialPose.x;
            startY = initialPose.y;
            startTheta = initialPose.theta || 0;
        } else {
            // 安全な初期位置を自動で見つける
            const safePos = this.findSafeInitialPosition();
            startX = safePos.x;
            startY = safePos.y;
            startTheta = safePos.theta;
        }
        
        // 初期位置情報を保存（クライアントに通知用）
        this.initialPose = { x: startX, y: startY, theta: startTheta };
        
        this.robot = new Robot(startX, startY, startTheta, canvasWidth, canvasHeight, this.useImageMap);
        this.particleFilter = new ParticleFilter(
            SIM_CONF.particleCount, 
            canvasWidth, 
            canvasHeight, 
            this.world.walls, 
            this.useImageMap
        );
        
        // 初期位置をセットして即座にローカライズ
        this.particleFilter.setEstimatedPose(startX, startY, startTheta);
        this.performInitialLocalization();
        
        // 制御用
        this.lastCommand = { vx: 0, vy: 0, omega: 0 };
        
        // ロータリーエンコーダシミュレーション用（累積オドメトリ）
        this.encoderOdom = { x: 0, y: 0, theta: 0 };
        
        console.log(`🤖 Initial pose set: x=${(startX/METERS_TO_PIXELS).toFixed(2)}m, y=${(startY/METERS_TO_PIXELS).toFixed(2)}m, theta=${(startTheta*180/Math.PI).toFixed(1)}°`);
    }
    
    /**
     * 安全な初期位置を自動で見つける
     */
    findSafeInitialPosition() {
        const robotRadius = (ROBOT_CONF.robot?.radiusM || 0.15);
        const minDist = robotRadius + 0.3; // 壁から30cm以上離れた位置
        
        // コストマップから安全な位置を探索
        let bestX = this.canvasWidth * 0.2;
        let bestY = this.canvasHeight * 0.5;
        let bestDist = 0;
        
        // グリッドをスキャンして最も壁から離れた位置を探す
        const stepPx = PIXELS_PER_GRID * 5; // 大きめのステップで高速化
        
        for (let y = stepPx * 2; y < this.canvasHeight - stepPx * 2; y += stepPx) {
            for (let x = stepPx * 2; x < this.canvasWidth - stepPx * 2; x += stepPx) {
                const gx = Math.floor(x / PIXELS_PER_GRID);
                const gy = Math.floor(y / PIXELS_PER_GRID);
                const idx = gy * this.costmap.cols + gx;
                
                if (idx >= 0 && idx < this.costmap.distMap.length) {
                    const dist = this.costmap.distMap[idx];
                    if (dist > bestDist && dist > minDist) {
                        bestDist = dist;
                        bestX = x;
                        bestY = y;
                    }
                }
            }
        }
        
        return { x: bestX, y: bestY, theta: 0 };
    }
    
    /**
     * 初期位置情報を取得（クライアントに通知用）
     */
    getInitialPoseInfo() {
        const pose = this.initialPose;
        return {
            x_m: pose.x / METERS_TO_PIXELS,
            y_m: pose.y / METERS_TO_PIXELS,
            theta_deg: pose.theta * 180 / Math.PI,
            x_px: pose.x,
            y_px: pose.y,
            theta_rad: pose.theta
        };
    }
    
    performInitialLocalization() {
        // 静止状態で複数回更新し、パーティクルを収束させる
        for (let i = 0; i < 10; i++) {
            const realScan = this.robot.lidar.scan(this.robot.pose, this.world.walls, SIM_CONF.noise.lidar);
            this.particleFilter.update(realScan);
            this.particleFilter.resample();
        }
    }
    
    setGoal(goalX, goalY) {
        const est = this.particleFilter.getEstimate();
        
        console.log(`Planning path from (${est.x.toFixed(0)}, ${est.y.toFixed(0)}) to (${goalX.toFixed(0)}, ${goalY.toFixed(0)})`);
        
        const path = this.planner.plan({x: est.x, y: est.y}, {x: goalX, y: goalY});
        
        if (path) {
            this.robot.path = path;
            this.robot.pathIndex = 0;
            return { success: true, pathLength: path.length };
        }
        return { success: false, message: "No path found" };
    }
    
    setInitialPose(x, y, theta) {
        this.robot.pose = { x, y, theta };
        this.particleFilter.setEstimatedPose(x, y, theta);
        this.performInitialLocalization();
        return { success: true };
    }
    
    kidnap() {
        // ロボットをランダムな位置に移動（パーティクルはそのまま）
        const newX = Math.random() * this.canvasWidth * 0.8 + this.canvasWidth * 0.1;
        const newY = Math.random() * this.canvasHeight * 0.8 + this.canvasHeight * 0.1;
        const newTheta = Math.random() * Math.PI * 2;
        
        this.robot.pose.x = newX;
        this.robot.pose.y = newY;
        this.robot.pose.theta = newTheta;
        
        return { success: true };
    }
    
    globalLocalization() {
        this.particleFilter.initGlobal();
        
        // 少し収束させる
        for (let i = 0; i < 5; i++) {
            const realScan = this.robot.lidar.scan(this.robot.pose, this.world.walls, SIM_CONF.noise.lidar);
            this.particleFilter.update(realScan);
            this.particleFilter.resample();
        }
        
        const est = this.particleFilter.getEstimate();
        const error = MathUtils.dist(this.robot.pose, est) / METERS_TO_PIXELS;
        
        return { success: true, error };
    }
    
    /**
     * オムニホイール用制御指令値計算 (Holonomic Controller)
     * マイコンへ送信する形式: { vx: m/s, vy: m/s, omega: rad/s }
     */
    computeControl() {
        if (!this.robot.path || this.robot.path.length === 0) {
            this.lastCommand = { vx: 0, vy: 0, omega: 0 };
            return this.lastCommand;
        }

        const est = this.particleFilter.getEstimate();
        
        // Look ahead
        const lookAheadDist = (ROBOT_CONF.control?.lookAheadDistM || 0.4) * METERS_TO_PIXELS;
        let targetIdx = this.robot.pathIndex;
        
        for (let i = this.robot.pathIndex; i < this.robot.path.length; i++) {
            const d = MathUtils.dist(est, this.robot.path[i]);
            if (d > lookAheadDist) {
                targetIdx = i;
                break;
            }
            if (i === this.robot.path.length - 1) targetIdx = i;
        }
        this.robot.pathIndex = targetIdx;
        
        const target = this.robot.path[targetIdx];
        
        // グローバル座標系での偏差
        const dx_global = target.x - est.x;
        const dy_global = target.y - est.y;
        const dist = Math.hypot(dx_global, dy_global);
        
        // ゴール到達判定
        const goalTolerance = (ROBOT_CONF.control?.goalToleranceM || 0.05) * METERS_TO_PIXELS;
        if (this.robot.pathIndex >= this.robot.path.length - 1 && dist < goalTolerance) {
            this.robot.path = null;
            this.lastCommand = { vx: 0, vy: 0, omega: 0 };
            return this.lastCommand;
        }
        
        // グローバル座標系での速度ベクトル
        const maxSpeed = ROBOT_CONF.kinematics?.maxSpeedMps || 1.0;
        const linearGain = ROBOT_CONF.control?.linearPGain || 2.0;
        const angularGain = ROBOT_CONF.control?.angularPGain || 3.0;
        
        let speed = Math.min(maxSpeed, linearGain * (dist / METERS_TO_PIXELS));
        
        const v_global_x = (dx_global / dist) * speed * METERS_TO_PIXELS;
        const v_global_y = (dy_global / dist) * speed * METERS_TO_PIXELS;
        
        // ロボット座標系へ変換
        const cos = Math.cos(est.theta);
        const sin = Math.sin(est.theta);
        
        let vx = (v_global_x * cos + v_global_y * sin) / METERS_TO_PIXELS;
        let vy = (-v_global_x * sin + v_global_y * cos) / METERS_TO_PIXELS;
        
        // 向き制御
        const targetGlobalAngle = Math.atan2(dy_global, dx_global);
        const angleDiff = MathUtils.normalizeAngle(targetGlobalAngle - est.theta);
        let omega = angleDiff * angularGain;
        
        // 角速度制限
        const maxOmega = ROBOT_CONF.kinematics?.maxRotationRadps || 2.0;
        omega = Math.max(-maxOmega, Math.min(maxOmega, omega));
        
        // 速度制限
        vx = Math.max(-maxSpeed, Math.min(maxSpeed, vx));
        vy = Math.max(-maxSpeed, Math.min(maxSpeed, vy));
        
        this.lastCommand = { vx, vy, omega };
        return this.lastCommand;
    }
    
    update(dt) {
        // 制御ループ
        const ctrl = this.computeControl();
        
        // ロボットを移動させる & オドメトリ取得
        const odomDelta = this.robot.move(ctrl.vx, ctrl.vy, ctrl.omega, dt, this.world.walls);
        
        // パーティクルフィルタ更新
        const moveThreshold = 0.001; // 1mm
        const rotThreshold = 0.001; // ~0.05度
        
        if (Math.abs(odomDelta.dx) > moveThreshold || 
            Math.abs(odomDelta.dy) > moveThreshold || 
            Math.abs(odomDelta.dtheta) > rotThreshold) {
            this.particleFilter.predict(odomDelta);
        }
        
        // センサ更新
        const realScan = this.robot.lidar.scan(this.robot.pose, this.world.walls, SIM_CONF.noise.lidar);
        this.particleFilter.update(realScan);
        this.particleFilter.resample();
    }
    
    getState() {
        const est = this.particleFilter.getEstimate();
        const scan = this.robot.lidar.scan(this.robot.pose, this.world.walls, SIM_CONF.noise.lidar);
        
        return {
            robot: { pose: this.robot.pose, radius: this.robot.radius },
            estimate: est,
            particles: this.particleFilter.particles,
            scan: Array.from(scan),
            path: this.robot.path,
            walls: this.world.walls,
            costmap: { cols: this.costmap.cols, rows: this.costmap.rows },
            command: this.lastCommand, // マイコンへの指令値
            stats: {
                posError: MathUtils.dist(this.robot.pose, est) / METERS_TO_PIXELS,
                odomError: 0,
                pathLength: this.robot.path ? this.robot.path.length : 0,
                status: this.robot.path ? "MOVING (OMNI)" : "IDLE"
            }
        };
    }
}

module.exports = { SimulationEngine, METERS_TO_PIXELS, ROBOT_CONF };
