/**
 * LiDAR Simulation Engine (Omni-wheel Ready)
 */
const fs = require('fs');
const path = require('path');

// 設定読み込み
const CONFIG_PATH = path.join(__dirname, 'robot-config.json');
let ROBOT_CONF = {
    radiusM: 0.15,
    lidar: { offsetX_M: 0, offsetY_M: 0, numRays: 450, maxRangeM: 12.0, minRangeM: 0.02 },
    kinematics: { maxSpeedMps: 1.0, maxRotationRadps: 2.0 },
    safety: { wallMarginM: 0.30, minPassageWidthM: 0.45 }
};

try {
    if (fs.existsSync(CONFIG_PATH)) {
        ROBOT_CONF = JSON.parse(fs.readFileSync(CONFIG_PATH, 'utf8'));
        console.log('Loaded robot config:', ROBOT_CONF);
    }
} catch (e) {
    console.error('Failed to load robot config, using defaults.', e);
}

const METERS_TO_PIXELS = 40;
const GRID_RES = 0.10; // 10cm grid
const PIXELS_PER_GRID = GRID_RES * METERS_TO_PIXELS;

const SIM_CONF = {
    particleCount: 600,
    particleLidarRays: 45, // パーティクル評価用は軽量化
    dt: 0.05,
    noise: {
        // オムニ用ノイズモデル (x, y, thetaそれぞれにノイズが乗る)
        odom_xy: 0.05,   // 移動距離に対する分散係数
        odom_theta: 0.03, // 回転に対する分散係数
        lidar: 0.020 * METERS_TO_PIXELS
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
        if (den === 0) return null;
        const t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
        const u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
            return { x: x1 + t * (x2 - x1), y: y1 + t * (y2 - y1) };
        }
        return null;
    }
}

// --- Costmap & Planner (変更なし - 省略可能だが依存関係のため記載) ---
// ※ 既存のA*ロジックはそのまま使用可能。ただしオムニ移動なので経路追従側で工夫する。
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
        for (let w of walls) this.rasterizeLine(w.p1, w.p2);
        const totalPixels = this.cols * this.rows;
        const queue = new Int32Array(totalPixels);
        let head = 0, tail = 0;
        for (let i = 0; i < totalPixels; i++) {
            if (this.isWall[i]) { this.distMap[i] = 0; queue[tail++] = i; }
        }
        const offsets = [-1, 1, -this.cols, this.cols];
        while(head < tail) {
            let currIdx = queue[head++];
            let cx = currIdx % this.cols;
            let currentDist = this.distMap[currIdx];
            for(let offset of offsets) {
                let neighborIdx = currIdx + offset;
                let nx = neighborIdx % this.cols;
                let ny = Math.floor(neighborIdx / this.cols);
                if (nx < 0 || nx >= this.cols || ny < 0 || ny >= this.rows) continue;
                if (Math.abs(nx - cx) > 1) continue;
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
        let dx = Math.abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        let dy = -Math.abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        let err = dx + dy;
        while (true) {
            if (x0 >= 0 && x0 < this.cols && y0 >= 0 && y0 < this.rows) this.isWall[y0 * this.cols + x0] = 1;
            if (x0 === x1 && y0 === y1) break;
            let e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }
    getCost(idx) {
        const dist = this.distMap[idx];
        if (dist < ROBOT_CONF.safety.minPassageWidthM / 2) return 255;
        const safeDist = 1.0;
        if (dist > safeDist) return 1;
        const norm = (dist - ROBOT_CONF.safety.minPassageWidthM/2) / (safeDist - ROBOT_CONF.safety.minPassageWidthM/2);
        return Math.floor((1.0 - norm) * 50) + 1;
    }
}

class GlobalPlanner {
    constructor(costmap) { this.cm = costmap; }
    plan(startPx, goalPx) {
        const sx = Math.floor(startPx.x / PIXELS_PER_GRID), sy = Math.floor(startPx.y / PIXELS_PER_GRID);
        const gx = Math.floor(goalPx.x / PIXELS_PER_GRID), gy = Math.floor(goalPx.y / PIXELS_PER_GRID);
        const startIdx = sy * this.cm.cols + sx, goalIdx = gy * this.cm.cols + gx;
        if (sx < 0 || sx >= this.cm.cols || sy < 0 || sy >= this.cm.rows) return null;
        if (gx < 0 || gx >= this.cm.cols || gy < 0 || gy >= this.cm.rows) return null;
        if (this.cm.getCost(goalIdx) === 255) return null;
        
        const openSet = new Set([startIdx]);
        const cameFrom = new Map();
        const gScore = new Map(); gScore.set(startIdx, 0);
        const fScore = new Map(); fScore.set(startIdx, this.heuristic(sx, sy, gx, gy));
        const neighbors = [-1, 1, -this.cm.cols, this.cm.cols, -this.cm.cols-1, -this.cm.cols+1, this.cm.cols-1, this.cm.cols+1];
        
        let iterations = 0;
        while (openSet.size > 0 && iterations++ < 50000) {
            let current = null, lowestF = Infinity;
            for (let node of openSet) {
                const f = fScore.get(node) || Infinity;
                if (f < lowestF) { lowestF = f; current = node; }
            }
            if (current === goalIdx) return this.reconstructPath(cameFrom, current);
            openSet.delete(current);
            
            const cx = current % this.cm.cols, cy = Math.floor(current / this.cm.cols);
            for (let offset of neighbors) {
                const neighbor = current + offset;
                const nx = neighbor % this.cm.cols, ny = Math.floor(neighbor / this.cm.cols);
                if (nx < 0 || nx >= this.cm.cols || ny < 0 || ny >= this.cm.rows) continue;
                if (Math.abs(nx - cx) > 1 || Math.abs(ny - cy) > 1) continue;
                
                const cellCost = this.cm.getCost(neighbor);
                if (cellCost === 255) continue;
                
                const distCost = (Math.abs(nx-cx)+Math.abs(ny-cy) === 2) ? 1.414 : 1.0;
                const tentative_gScore = gScore.get(current) + distCost + (cellCost * 0.05);
                if (tentative_gScore < (gScore.get(neighbor) || Infinity)) {
                    cameFrom.set(neighbor, current);
                    gScore.set(neighbor, tentative_gScore);
                    fScore.set(neighbor, tentative_gScore + this.heuristic(nx, ny, gx, gy));
                    if (!openSet.has(neighbor)) openSet.add(neighbor);
                }
            }
        }
        return null;
    }
    heuristic(x1, y1, x2, y2) { return Math.hypot(x2 - x1, y2 - y1); }
    reconstructPath(cameFrom, current) {
        const totalPath = [this.idxToPoint(current)];
        while (cameFrom.has(current)) {
            current = cameFrom.get(current);
            totalPath.unshift(this.idxToPoint(current));
        }
        return totalPath;
    }
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
        if (imageData) this.createWallsFromImage(imageData);
        else this.createWalls();
    }
    createWallsFromImage(imageData) {
        // (省略: 元のコードと同じ内容を使用)
        // ここは変更ありませんが、長くなるので省略します。元のコードをそのまま使ってください。
        // ※実際の実装では元の `createWallsFromImage` の中身をここに貼ってください。
        this.walls.push({p1: {x:0, y:0}, p2: {x:this.width, y:0}});
        this.walls.push({p1: {x:this.width, y:0}, p2: {x:this.width, y:this.height}});
        this.walls.push({p1: {x:this.width, y:this.height}, p2: {x:0, y:this.height}});
        this.walls.push({p1: {x:0, y:this.height}, p2: {x:0, y:0}});
        // 簡易的に外壁のみ追加（実際は元の画像解析ロジックを入れてください）
    }
    createWalls() {
        const w = this.width, h = this.height;
        this.walls.push({p1: {x:0, y:0}, p2: {x:w, y:0}});
        this.walls.push({p1: {x:w, y:0}, p2: {x:w, y:h}});
        this.walls.push({p1: {x:w, y:h}, p2: {x:0, y:h}});
        this.walls.push({p1: {x:0, y:h}, p2: {x:0, y:0}});
        this.walls.push({p1: {x:w*0.2, y:h*0.2}, p2: {x:w*0.8, y:h*0.2}}); // Sample obstacle
    }
}

// --- Lidar (オフセット対応) ---
class Lidar {
    constructor(numRays, maxRange) {
        this.numRays = numRays;
        this.maxRange = maxRange;
        this.fov = Math.PI * 2;
        // Configからオフセット取得 (メートル -> ピクセル)
        this.offsetX = ROBOT_CONF.lidar.offsetX_M * METERS_TO_PIXELS;
        this.offsetY = ROBOT_CONF.lidar.offsetY_M * METERS_TO_PIXELS;
    }
    
    // pose: ロボットの中心座標と向き
    scan(pose, walls, noiseStdDev = 0) {
        const ranges = new Float32Array(this.numRays);
        const angleStep = this.fov / this.numRays;
        
        // LiDARの実際の位置を計算 (ロボット座標系 -> グローバル座標系)
        // ロボットの向き(theta)に応じてオフセットを回転させる
        const lidarGlobal = MathUtils.localToGlobal(this.offsetX, this.offsetY, pose.x, pose.y, pose.theta);
        
        for (let i = 0; i < this.numRays; i++) {
            // LiDAR自体の向きも考慮する場合はここにオフセット角度を加えるが、通常LiDARはロボット正面を0度とする
            const angle = pose.theta + i * angleStep;
            
            const dirX = Math.cos(angle);
            const dirY = Math.sin(angle);
            
            // レイの発射起点はLiDARの位置
            const rayEndX = lidarGlobal.x + dirX * this.maxRange;
            const rayEndY = lidarGlobal.y + dirY * this.maxRange;

            let minDist = this.maxRange;
            for (let j = 0, len = walls.length; j < len; j++) {
                const w = walls[j];
                const hit = MathUtils.intersect(w.p1.x, w.p1.y, w.p2.x, w.p2.y, lidarGlobal.x, lidarGlobal.y, rayEndX, rayEndY);
                if (hit) {
                    const d = Math.hypot(hit.x - lidarGlobal.x, hit.y - lidarGlobal.y);
                    if (d < minDist) minDist = d;
                }
            }
            if (noiseStdDev > 0 && minDist < this.maxRange) {
                minDist += MathUtils.gaussianRandom(0, noiseStdDev);
                if(minDist < ROBOT_CONF.lidar.minRangeM * METERS_TO_PIXELS) 
                    minDist = ROBOT_CONF.lidar.minRangeM * METERS_TO_PIXELS;
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
        this.radius = ROBOT_CONF.radiusM * METERS_TO_PIXELS;
        // LiDAR設定
        const lidarRays = useImageMap ? 180 : ROBOT_CONF.lidar.numRays;
        this.lidar = new Lidar(lidarRays, ROBOT_CONF.lidar.maxRangeM * METERS_TO_PIXELS);
        this.path = null;
        this.pathIndex = 0;
        this.canvasWidth = canvasWidth;
        this.canvasHeight = canvasHeight;
        
        // マイコンへの送信値をシミュレートするためのオドメトリ累積値
        this.odomAccum = { x: 0, y: 0, theta: 0 };
    }

    /**
     * オムニホイール移動
     * @param {number} vx - ロボット前方方向の速度 (m/s)
     * @param {number} vy - ロボット左方向の速度 (m/s)
     * @param {number} omega - 角速度 (rad/s)
     * @param {number} dt - 時間刻み
     */
    move(vx, vy, omega, dt) {
        // 1. ノイズを含まない真の移動量を計算
        
        // ロボット座標系での移動量 (m)
        const dx_local = vx * dt;
        const dy_local = vy * dt;
        const dtheta = omega * dt;

        // グローバル座標系への変換 (現在の向き theta を使用)
        // グローバルでの変位
        const globalDx = dx_local * Math.cos(this.pose.theta) - dy_local * Math.sin(this.pose.theta);
        const globalDy = dx_local * Math.sin(this.pose.theta) + dy_local * Math.cos(this.pose.theta);

        // ピクセル単位に変換して更新
        const trueNextX = this.pose.x + globalDx * METERS_TO_PIXELS;
        const trueNextY = this.pose.y + globalDy * METERS_TO_PIXELS;
        const trueNextTheta = MathUtils.normalizeAngle(this.pose.theta + dtheta);

        // 衝突判定 (簡易的: キャンバス境界のみ)
        let collision = false;
        if (trueNextX < 0 || trueNextX > this.canvasWidth || trueNextY < 0 || trueNextY > this.canvasHeight) {
            collision = true;
        }
        
        if (!collision) {
            this.pose.x = trueNextX;
            this.pose.y = trueNextY;
            this.pose.theta = trueNextTheta;
        }

        // 2. オドメトリの計算 (マイコンが計算する自己位置推定値のシミュレーション)
        // エンコーダノイズ等をシミュレート
        // 速度に比例したノイズ + 定常ノイズ
        const speed = Math.hypot(vx, vy);
        const noiseX = MathUtils.gaussianRandom(0, (speed * SIM_CONF.noise.odom_xy + 0.001) * dt);
        const noiseY = MathUtils.gaussianRandom(0, (speed * SIM_CONF.noise.odom_xy + 0.001) * dt);
        const noiseTh = MathUtils.gaussianRandom(0, (Math.abs(omega) * SIM_CONF.noise.odom_theta + 0.001) * dt);

        const noisyVx = vx + noiseX;
        const noisyVy = vy + noiseY;
        const noisyOmega = omega + noiseTh;

        // オドメトリ情報の差分を返す (m, rad単位)
        // パーティクルフィルタはこの「ノイズを含んだローカル移動量」を受け取る
        return {
            dx: noisyVx * dt,
            dy: noisyVy * dt,
            dtheta: noisyOmega * dt
        };
    }
}

// --- Particle Filter (オムニ対応) ---
class ParticleFilter {
    constructor(count, mapWidth, mapHeight, knownMap, useImageMap = false) {
        this.count = count;
        this.width = mapWidth;
        this.height = mapHeight;
        this.map = knownMap;
        const particleLidarRays = useImageMap ? 20 : SIM_CONF.particleLidarRays;
        this.simLidar = new Lidar(particleLidarRays, ROBOT_CONF.lidar.maxRangeM * METERS_TO_PIXELS);
        this.particles = [];
        this.initGlobal();
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
    }
    
    setEstimatedPose(x, y, theta) {
        this.particles = [];
        const xyStdDev = 0.2 * METERS_TO_PIXELS; // 初期分散を少し小さく
        const thetaStdDev = 0.1;
        for (let i = 0; i < this.count; i++) {
            this.particles.push({
                x: x + MathUtils.gaussianRandom(0, xyStdDev),
                y: y + MathUtils.gaussianRandom(0, xyStdDev),
                theta: MathUtils.normalizeAngle(theta + MathUtils.gaussianRandom(0, thetaStdDev)),
                weight: 1.0 / this.count
            });
        }
    }
    
    /**
     * 予測ステップ (Motion Model)
     * オムニホイール対応: ローカル座標系での移動量(dx, dy, dtheta)を受け取り、各パーティクルに適用
     */
    predict(odomDelta) {
        // odomDelta: { dx, dy, dtheta } (meters, radians)
        const dDist = Math.hypot(odomDelta.dx, odomDelta.dy);
        
        // 移動量に応じた分散
        const distVar = dDist * SIM_CONF.noise.odom_xy * METERS_TO_PIXELS; 
        const angVar = Math.abs(odomDelta.dtheta) * SIM_CONF.noise.odom_theta;

        for (let p of this.particles) {
            // ローカル移動量にノイズを加える
            const noisyLx = odomDelta.dx * METERS_TO_PIXELS + MathUtils.gaussianRandom(0, distVar + 0.1);
            const noisyLy = odomDelta.dy * METERS_TO_PIXELS + MathUtils.gaussianRandom(0, distVar + 0.1);
            const noisyTh = odomDelta.dtheta + MathUtils.gaussianRandom(0, angVar + 0.005);

            // ローカル -> グローバル変換してパーティクル位置更新
            // パーティクルの現在の向き p.theta を基準にする
            const cos = Math.cos(p.theta);
            const sin = Math.sin(p.theta);

            p.x += (noisyLx * cos - noisyLy * sin);
            p.y += (noisyLx * sin + noisyLy * cos);
            p.theta = MathUtils.normalizeAngle(p.theta + noisyTh);
        }
    }
    
    update(realScan) {
        let maxWeight = 0;
        const stepRatio = Math.floor(realScan.length / this.simLidar.numRays);
        // 尤度計算の厳しさを調整 (分散)
        const sigma2 = 100.0; 

        for (let p of this.particles) {
            // マップ外判定
            if (p.x < 0 || p.x > this.width || p.y < 0 || p.y > this.height) {
                p.weight = 0;
                continue;
            }

            // このパーティクル位置・向きでのLiDARスキャンをシミュレート
            // ※ Lidarクラス内でオフセット計算が含まれているため、ここではpを渡すだけでOK
            const hypoScan = this.simLidar.scan(p, this.map, 0);
            
            let logLikelihood = 0;
            let validPoints = 0;
            
            for (let i = 0; i < hypoScan.length; i++) {
                const realDist = realScan[i * stepRatio];
                const hypoDist = hypoScan[i];
                
                // 最大距離の場合は情報量が少ないのでスキップ
                if (realDist >= this.simLidar.maxRange * 0.95 && hypoDist >= this.simLidar.maxRange * 0.95) continue;
                
                const diff = realDist - hypoDist;
                logLikelihood += -(diff * diff) / sigma2;
                validPoints++;
            }
            
            if (validPoints > 0) {
                // 重みの計算 (対数尤度から戻す)
                p.weight = Math.exp(logLikelihood / validPoints);
                // 重みが極端に小さくなりすぎないようにクリップ
                // if (p.weight < 0.0001) p.weight = 0.0001; 
            } else {
                p.weight = 0.00001;
            }
            
            if (p.weight > maxWeight) maxWeight = p.weight;
        }
        
        // 正規化
        let totalWeight = 0;
        for (let p of this.particles) totalWeight += p.weight;
        
        if (totalWeight > 0) {
            for (let p of this.particles) p.weight /= totalWeight;
            // Effective Sample Size (ESS) 計算などをここに入れてリサンプリング閾値を設けるのが本来だが、簡易的に毎回リサンプリング
        } else {
            // 全滅した場合はリセット（あるいはランダム注入）
            this.initGlobal();
        }
    }
    
    resample() {
        const newParticles = [];
        let index = Math.floor(Math.random() * this.count);
        let beta = 0;
        let maxWeight = 0;
        for (let p of this.particles) if(p.weight > maxWeight) maxWeight = p.weight;
        
        // 低分散リサンプリングなどのアルゴリズムもあるが、ここではホイールリサンプリング
        // わずかなランダムパーティクルを混ぜる（位置ロスト回復用）
        const randomCount = Math.floor(this.count * 0.02); 
        
        for (let i = 0; i < this.count - randomCount; i++) {
            beta += Math.random() * 2 * maxWeight;
            while (beta > this.particles[index].weight) {
                beta -= this.particles[index].weight;
                index = (index + 1) % this.count;
            }
            const p = this.particles[index];
            // コピーを作成 (ジッターを加えることで粒子の枯渇を防ぐ)
            newParticles.push({ 
                x: p.x + MathUtils.gaussianRandom(0, 1), 
                y: p.y + MathUtils.gaussianRandom(0, 1), 
                theta: p.theta + MathUtils.gaussianRandom(0, 0.01), 
                weight: p.weight 
            });
        }
        
        // ランダムパーティクル注入
        for(let i=0; i<randomCount; i++) {
            newParticles.push({
                x: Math.random() * this.width,
                y: Math.random() * this.height,
                theta: Math.random() * Math.PI * 2,
                weight: 0
            });
        }
        
        this.particles = newParticles;
    }
    
    getEstimate() {
        let x = 0, y = 0, sinSum = 0, cosSum = 0, totalW = 0;
        // 重み付き平均
        for (let p of this.particles) {
            x += p.x * p.weight;
            y += p.y * p.weight;
            sinSum += Math.sin(p.theta) * p.weight;
            cosSum += Math.cos(p.theta) * p.weight;
            totalW += p.weight;
        }
        if (totalW === 0) return this.particles[0];
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
        
        let startX, startY, startTheta;
        if (initialPose) {
            startX = initialPose.x; startY = initialPose.y; startTheta = initialPose.theta || 0;
        } else {
            startX = canvasWidth * 0.2; startY = canvasHeight * 0.5; startTheta = 0;
        }
        
        this.robot = new Robot(startX, startY, startTheta, canvasWidth, canvasHeight, this.useImageMap);
        this.particleFilter = new ParticleFilter(SIM_CONF.particleCount, canvasWidth, canvasHeight, this.world.walls, this.useImageMap);
        
        // 初期位置をセットして即座にローカライズ
        this.particleFilter.setEstimatedPose(startX, startY, startTheta);
        this.performInitialLocalization();
    }
    
    performInitialLocalization() {
        // 静止状態で複数回更新し、パーティクルを収束させる
        for (let i = 0; i < 20; i++) {
            const realScan = this.robot.lidar.scan(this.robot.pose, this.world.walls, SIM_CONF.noise.lidar);
            this.particleFilter.update(realScan);
            this.particleFilter.resample();
        }
    }
    
    setGoal(goalX, goalY) {
        const est = this.particleFilter.getEstimate();
        const path = this.planner.plan({x: est.x, y: est.y}, {x: goalX, y: goalY});
        if (path) {
            this.robot.path = path;
            this.robot.pathIndex = 0;
            return { success: true, pathLength: path.length };
        }
        return { success: false, message: "No path found" };
    }
    
    setInitialPose(x, y, theta) {
        this.robot.pose = { x, y, theta }; // ロボット真値を強制移動
        this.particleFilter.setEstimatedPose(x, y, theta);
        this.performInitialLocalization();
        return { success: true };
    }
    
    kidnap() {
        this.robot.pose.x = Math.random() * this.canvasWidth;
        this.robot.pose.y = Math.random() * this.canvasHeight;
        this.robot.pose.theta = Math.random() * Math.PI * 2;
        return { success: true };
    }
    
    globalLocalization() {
        this.particleFilter.initGlobal();
        // 散らした直後は収束していないので何もしない、updateループで収束させる
        return { success: true };
    }
    
    /**
     * オムニホイール用制御指令値計算 (Holonomic Controller)
     * ロボット座標系での vx, vy, omega を返す
     */
    computeControl() {
        if (!this.robot.path || this.robot.path.length === 0) {
            return { vx: 0, vy: 0, omega: 0 };
        }

        const est = this.particleFilter.getEstimate();
        
        // Look ahead
        let targetIdx = this.robot.pathIndex;
        const lookAheadDist = 0.6 * METERS_TO_PIXELS;
        
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
        if (this.robot.pathIndex >= this.robot.path.length - 1 && dist < 0.1 * METERS_TO_PIXELS) {
            this.robot.path = null;
            return { vx: 0, vy: 0, omega: 0 };
        }
        
        // グローバル座標系での速度ベクトル
        const speed = ROBOT_CONF.kinematics.maxSpeedMps * METERS_TO_PIXELS;
        const v_global_x = (dx_global / dist) * speed;
        const v_global_y = (dy_global / dist) * speed;
        
        // ロボット座標系へ変換 (オムニホイールへの指令値)
        // Rotation Matrix R(-theta)
        const cos = Math.cos(est.theta);
        const sin = Math.sin(est.theta);
        
        // ローカル速度 (ロボットから見た前方・左方)
        let vx = v_global_x * cos + v_global_y * sin;
        let vy = -v_global_x * sin + v_global_y * cos;
        
        // 向き制御: 進行方向を向くようにするか、ゴール方向を向くか。
        // ここでは「常にパスの進行方向を向く」ように制御
        const targetGlobalAngle = Math.atan2(dy_global, dx_global);
        const angleDiff = MathUtils.normalizeAngle(targetGlobalAngle - est.theta);
        let omega = angleDiff * 4.0;
        
        // 減速処理
        if (dist < lookAheadDist) {
            vx *= (dist / lookAheadDist);
            vy *= (dist / lookAheadDist);
        }
        
        // マイコンへの指令値形式に合わせる (m/s)
        return { 
            vx: vx / METERS_TO_PIXELS, 
            vy: vy / METERS_TO_PIXELS, 
            omega: omega 
        };
    }
    
    update(dt) {
        // 制御ループ (m/s)
        const ctrl = this.computeControl();
        
        // ロボットを移動させる & オドメトリ取得 (ローカル変化量 dx, dy, dtheta)
        const odomDelta = this.robot.move(ctrl.vx, ctrl.vy, ctrl.omega, dt);
        
        // パーティクルフィルタ更新
        // 1. 予測 (Motion Update)
        if (Math.abs(odomDelta.dx) > 1e-4 || Math.abs(odomDelta.dy) > 1e-4 || Math.abs(odomDelta.dtheta) > 1e-4) {
            this.particleFilter.predict(odomDelta);
        }
        
        // 2. 計測更新 (Sensor Update) - 常に実行することで静止時も位置を補正
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
            stats: {
                posError: MathUtils.dist(this.robot.pose, est) / METERS_TO_PIXELS,
                odomError: 0, // オムニの場合累積誤差の定義が複雑になるため一旦0
                pathLength: this.robot.path ? this.robot.path.length : 0,
                status: this.robot.path ? "MOVING (OMNI)" : "IDLE"
            }
        };
    }
}

module.exports = { SimulationEngine, METERS_TO_PIXELS, ROBOT_CONF };