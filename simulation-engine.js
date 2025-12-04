/**
 * LiDAR Simulation Engine (Server-side)
 * All computation logic runs here
 */

const METERS_TO_PIXELS = 40;
const GRID_RES = 0.10;
const PIXELS_PER_GRID = GRID_RES * METERS_TO_PIXELS;

const CONF = {
    maxLidarRange: 12.0 * METERS_TO_PIXELS,
    minLidarRange: 0.02 * METERS_TO_PIXELS,
    lidarRays: 450,
    particleCount: 600,
    particleLidarRays: 45,
    dt: 0.05,
    robotRadiusM: 0.15,
    safetyDistM: 0.30,
    minClearanceM: 0.225,
    noise: {
        odom_dist: 0.05,
        odom_angle: 0.03,
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

// --- Costmap ---
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
        for (let w of walls) {
            this.rasterizeLine(w.p1, w.p2);
        }
        
        const totalPixels = this.cols * this.rows;
        const queue = new Int32Array(totalPixels);
        let head = 0;
        let tail = 0;
        
        for (let i = 0; i < totalPixels; i++) {
            if (this.isWall[i]) {
                this.distMap[i] = 0;
                queue[tail++] = i;
            }
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
        let x0 = Math.floor(p1.x / PIXELS_PER_GRID);
        let y0 = Math.floor(p1.y / PIXELS_PER_GRID);
        let x1 = Math.floor(p2.x / PIXELS_PER_GRID);
        let y1 = Math.floor(p2.y / PIXELS_PER_GRID);
        
        let dx = Math.abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        let dy = -Math.abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        let err = dx + dy;
        
        while (true) {
            if (x0 >= 0 && x0 < this.cols && y0 >= 0 && y0 < this.rows) {
                this.isWall[y0 * this.cols + x0] = 1;
            }
            if (x0 === x1 && y0 === y1) break;
            let e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }
    
    getCost(idx) {
        const dist = this.distMap[idx];
        if (dist < CONF.minClearanceM) return 255;
        const safeDist = 1.0;
        if (dist > safeDist) return 1;
        const norm = (dist - CONF.minClearanceM) / (safeDist - CONF.minClearanceM);
        const cost = Math.floor((1.0 - norm) * 50) + 1;
        return cost;
    }
}

// --- Global Planner (A*) ---
class GlobalPlanner {
    constructor(costmap) {
        this.cm = costmap;
    }
    
    plan(startPx, goalPx) {
        const sx = Math.floor(startPx.x / PIXELS_PER_GRID);
        const sy = Math.floor(startPx.y / PIXELS_PER_GRID);
        const gx = Math.floor(goalPx.x / PIXELS_PER_GRID);
        const gy = Math.floor(goalPx.y / PIXELS_PER_GRID);
        
        const startIdx = sy * this.cm.cols + sx;
        const goalIdx = gy * this.cm.cols + gx;
        
        if (sx < 0 || sx >= this.cm.cols || sy < 0 || sy >= this.cm.rows) return null;
        if (gx < 0 || gx >= this.cm.cols || gy < 0 || gy >= this.cm.rows) return null;
        if (this.cm.getCost(goalIdx) === 255) {
            console.warn("Goal is in lethal obstacle/too close to wall");
            return null;
        }
        
        // Use Set for faster lookup
        const openSet = new Set([startIdx]);
        const closedSet = new Set();
        const cameFrom = new Map();
        const gScore = new Map();
        const fScore = new Map();
        
        gScore.set(startIdx, 0);
        fScore.set(startIdx, this.heuristic(sx, sy, gx, gy));
        
        const neighbors = [-1, 1, -this.cm.cols, this.cm.cols, 
                          -this.cm.cols-1, -this.cm.cols+1, this.cm.cols-1, this.cm.cols+1];

        let iterations = 0;
        const maxIterations = 100000; // Safety limit

        while (openSet.size > 0 && iterations < maxIterations) {
            iterations++;
            
            // Find node with lowest fScore
            let current = null;
            let lowestF = Infinity;
            for (let node of openSet) {
                const f = fScore.get(node) || Infinity;
                if (f < lowestF) {
                    lowestF = f;
                    current = node;
                }
            }
            
            if (current === null) break;
            
            if (current === goalIdx) {
                console.log(`Path found in ${iterations} iterations`);
                return this.reconstructPath(cameFrom, current);
            }
            
            openSet.delete(current);
            closedSet.add(current);
            
            const cx = current % this.cm.cols;
            const cy = Math.floor(current / this.cm.cols);
            
            for (let offset of neighbors) {
                const neighbor = current + offset;
                const nx = neighbor % this.cm.cols;
                const ny = Math.floor(neighbor / this.cm.cols);
                
                if (nx < 0 || nx >= this.cm.cols || ny < 0 || ny >= this.cm.rows) continue;
                if (Math.abs(nx - cx) > 1 || Math.abs(ny - cy) > 1) continue;
                if (closedSet.has(neighbor)) continue;
                
                const cellCost = this.cm.getCost(neighbor);
                if (cellCost === 255) continue;
                
                const distCost = (offset === -1 || offset === 1 || offset === -this.cm.cols || offset === this.cm.cols) ? 1.0 : 1.414;
                const tentative_gScore = gScore.get(current) + distCost + (cellCost * 0.05);
                
                if (tentative_gScore < (gScore.get(neighbor) || Infinity)) {
                    cameFrom.set(neighbor, current);
                    gScore.set(neighbor, tentative_gScore);
                    fScore.set(neighbor, tentative_gScore + this.heuristic(nx, ny, gx, gy));
                    
                    if (!openSet.has(neighbor)) {
                        openSet.add(neighbor);
                    }
                }
            }
        }
        
        if (iterations >= maxIterations) {
            console.log("Path planning timeout - max iterations reached");
        } else {
            console.log("No path found");
        }
        return null;
    }
    
    heuristic(x1, y1, x2, y2) {
        return Math.hypot(x2 - x1, y2 - y1);
    }
    
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
        
        if (imageData) {
            this.createWallsFromImage(imageData);
        } else {
            this.createWalls();
        }
    }
    
    createWallsFromImage(imageData) {
        // imageData = { width, height, data: Uint8Array (grayscale values) }
        // 黒(0)が壁、白(255)が空きスペース
        const threshold = 128; // この値以下を壁とみなす
        const imgW = imageData.width;
        const imgH = imageData.height;
        const scaleX = this.width / imgW;
        const scaleY = this.height / imgH;
        
        // 外壁
        this.walls.push({p1: {x:0, y:0}, p2: {x:this.width, y:0}});
        this.walls.push({p1: {x:this.width, y:0}, p2: {x:this.width, y:this.height}});
        this.walls.push({p1: {x:this.width, y:this.height}, p2: {x:0, y:this.height}});
        this.walls.push({p1: {x:0, y:this.height}, p2: {x:0, y:0}});
        
        const isWall = (x, y) => {
            if (x < 0 || x >= imgW || y < 0 || y >= imgH) return false;
            const idx = y * imgW + x;
            return imageData.data[idx] < threshold;
        };
        
        // 壁のセグメントを結合して最適化
        const mergeSegments = (segments) => {
            if (segments.length === 0) return [];
            
            // ソート
            segments.sort((a, b) => a.start - b.start);
            
            const merged = [];
            let current = segments[0];
            
            for (let i = 1; i < segments.length; i++) {
                const next = segments[i];
                // 連続または重複している場合は結合
                if (current.end >= next.start - 1) {
                    current.end = Math.max(current.end, next.end);
                } else {
                    merged.push(current);
                    current = next;
                }
            }
            merged.push(current);
            return merged;
        };
        
        // 水平方向の壁を検出（上下両方のエッジ）
        for (let y = 0; y < imgH; y++) {
            const topEdges = [];
            const bottomEdges = [];
            
            for (let x = 0; x < imgW; x++) {
                const curr = isWall(x, y);
                const above = isWall(x, y - 1);
                const below = isWall(x, y + 1);
                
                // 上エッジ（壁の上側の境界）
                if (curr && !above) {
                    const start = x;
                    while (x < imgW && isWall(x, y) && !isWall(x, y - 1)) x++;
                    topEdges.push({ start, end: x });
                    x--;
                }
                
                // 下エッジ（壁の下側の境界）
                if (curr && !below) {
                    const start = x;
                    while (x < imgW && isWall(x, y) && !isWall(x, y + 1)) x++;
                    bottomEdges.push({ start, end: x });
                    x--;
                }
            }
            
            // 上エッジの壁を追加
            const mergedTop = mergeSegments(topEdges);
            for (const seg of mergedTop) {
                if (seg.end - seg.start > 2) {
                    this.walls.push({
                        p1: {x: seg.start * scaleX, y: y * scaleY},
                        p2: {x: seg.end * scaleX, y: y * scaleY}
                    });
                }
            }
            
            // 下エッジの壁を追加
            const mergedBottom = mergeSegments(bottomEdges);
            for (const seg of mergedBottom) {
                if (seg.end - seg.start > 2) {
                    this.walls.push({
                        p1: {x: seg.start * scaleX, y: (y + 1) * scaleY},
                        p2: {x: seg.end * scaleX, y: (y + 1) * scaleY}
                    });
                }
            }
        }
        
        // 垂直方向の壁を検出（左右両方のエッジ）
        for (let x = 0; x < imgW; x++) {
            const leftEdges = [];
            const rightEdges = [];
            
            for (let y = 0; y < imgH; y++) {
                const curr = isWall(x, y);
                const left = isWall(x - 1, y);
                const right = isWall(x + 1, y);
                
                // 左エッジ（壁の左側の境界）
                if (curr && !left) {
                    const start = y;
                    while (y < imgH && isWall(x, y) && !isWall(x - 1, y)) y++;
                    leftEdges.push({ start, end: y });
                    y--;
                }
                
                // 右エッジ（壁の右側の境界）
                if (curr && !right) {
                    const start = y;
                    while (y < imgH && isWall(x, y) && !isWall(x + 1, y)) y++;
                    rightEdges.push({ start, end: y });
                    y--;
                }
            }
            
            // 左エッジの壁を追加
            const mergedLeft = mergeSegments(leftEdges);
            for (const seg of mergedLeft) {
                if (seg.end - seg.start > 2) {
                    this.walls.push({
                        p1: {x: x * scaleX, y: seg.start * scaleY},
                        p2: {x: x * scaleX, y: seg.end * scaleY}
                    });
                }
            }
            
            // 右エッジの壁を追加
            const mergedRight = mergeSegments(rightEdges);
            for (const seg of mergedRight) {
                if (seg.end - seg.start > 2) {
                    this.walls.push({
                        p1: {x: (x + 1) * scaleX, y: seg.start * scaleY},
                        p2: {x: (x + 1) * scaleX, y: seg.end * scaleY}
                    });
                }
            }
        }
        
        console.log(`Created ${this.walls.length} walls from image (optimized)`);
    }
    
    createWalls() {
        const w = this.width, h = this.height;
        
        // 外壁
        this.walls.push({p1: {x:0, y:0}, p2: {x:w, y:0}});
        this.walls.push({p1: {x:w, y:0}, p2: {x:w, y:h}});
        this.walls.push({p1: {x:w, y:h}, p2: {x:0, y:h}});
        this.walls.push({p1: {x:0, y:h}, p2: {x:0, y:0}});
        
        // 大きな部屋（左上）
        this.addRect(w*0.05, h*0.05, w*0.30, h*0.25);
        
        // 迷路のような通路（中央上部）
        this.addRect(w*0.50, h*0.05, w*0.15, h*0.15);
        this.addRect(w*0.70, h*0.15, w*0.15, h*0.10);
        
        // L字型の障害物（右上）
        this.walls.push({p1: {x:w*0.88, y:h*0.05}, p2: {x:w*0.88, y:h*0.25}});
        this.walls.push({p1: {x:w*0.88, y:h*0.25}, p2: {x:w*0.95, y:h*0.25}});
        
        // 複数の小部屋（中央）- 通路を広めに
        this.addRect(w*0.10, h*0.40, w*0.10, h*0.10);
        this.addRect(w*0.30, h*0.40, w*0.10, h*0.10);
        this.addRect(w*0.50, h*0.40, w*0.10, h*0.10);
        this.addRect(w*0.70, h*0.40, w*0.10, h*0.10);
        
        // ジグザグの壁（右中央）- より緩やかに
        this.walls.push({p1: {x:w*0.85, y:h*0.38}, p2: {x:w*0.90, y:h*0.42}});
        this.walls.push({p1: {x:w*0.90, y:h*0.42}, p2: {x:w*0.85, y:h*0.46}});
        this.walls.push({p1: {x:w*0.85, y:h*0.46}, p2: {x:w*0.90, y:h*0.50}});
        
        // 広い通路（左下）
        this.walls.push({p1: {x:w*0.05, y:h*0.62}, p2: {x:w*0.30, y:h*0.62}});
        this.walls.push({p1: {x:w*0.05, y:h*0.70}, p2: {x:w*0.30, y:h*0.70}});
        
        // 斜めの壁（中央下）- 間隔を広く
        this.walls.push({p1: {x:w*0.40, y:h*0.65}, p2: {x:w*0.48, y:h*0.85}});
        this.walls.push({p1: {x:w*0.55, y:h*0.65}, p2: {x:w*0.63, y:h*0.85}});
        
        // 柱のような障害物（右下）- 間隔を広く
        this.addRect(w*0.72, h*0.70, w*0.03, h*0.03);
        this.addRect(w*0.82, h*0.70, w*0.03, h*0.03);
        this.addRect(w*0.72, h*0.80, w*0.03, h*0.03);
        this.addRect(w*0.82, h*0.80, w*0.03, h*0.03);
        
        // 円形障害物（中央やや左下）- サイズ小さめ
        const cx = w*0.20, cy = h*0.85, r = w*0.04;
        const segments = 8;
        for(let i = 0; i < segments; i++) {
            const a1 = (i / segments) * Math.PI * 2;
            const a2 = ((i+1) / segments) * Math.PI * 2;
            this.walls.push({
                p1: {x: cx + Math.cos(a1)*r, y: cy + Math.sin(a1)*r},
                p2: {x: cx + Math.cos(a2)*r, y: cy + Math.sin(a2)*r}
            });
        }
    }
    
    addRect(x, y, w, h) {
        this.walls.push({p1:{x:x, y:y}, p2:{x:x+w, y:y}});
        this.walls.push({p1:{x:x+w, y:y}, p2:{x:x+w, y:y+h}});
        this.walls.push({p1:{x:x+w, y:y+h}, p2:{x:x, y:y+h}});
        this.walls.push({p1:{x:x, y:y+h}, p2:{x:x, y:y}});
    }
}

// --- Lidar ---
class Lidar {
    constructor(numRays, maxRange) {
        this.numRays = numRays;
        this.maxRange = maxRange;
        this.fov = Math.PI * 2;
    }
    
    scan(pose, walls, noiseStdDev = 0) {
        const ranges = new Float32Array(this.numRays);
        const startAngle = pose.theta;
        const angleStep = this.fov / this.numRays;
        const poseX = pose.x;
        const poseY = pose.y;

        for (let i = 0; i < this.numRays; i++) {
            const angle = startAngle + i * angleStep;
            const dirX = Math.cos(angle);
            const dirY = Math.sin(angle);
            const rayEndX = poseX + dirX * this.maxRange;
            const rayEndY = poseY + dirY * this.maxRange;

            let minDist = this.maxRange;
            for (let j = 0, len = walls.length; j < len; j++) {
                const w = walls[j];
                const hit = MathUtils.intersect(w.p1.x, w.p1.y, w.p2.x, w.p2.y, poseX, poseY, rayEndX, rayEndY);
                if (hit) {
                    const d = Math.hypot(hit.x - poseX, hit.y - poseY);
                    if (d < minDist) minDist = d;
                }
            }
            if (noiseStdDev > 0 && minDist < this.maxRange) {
                minDist += MathUtils.gaussianRandom(0, noiseStdDev);
                if(minDist < CONF.minLidarRange) minDist = CONF.minLidarRange;
            }
            ranges[i] = minDist;
        }
        return ranges;
    }
}

// --- Robot ---
class Robot {
    constructor(x, y, theta, canvasWidth, canvasHeight, useImageMap = false) {
        this.pose = { x, y, theta };
        this.odomPose = { x, y, theta };
        this.radius = CONF.robotRadiusM * METERS_TO_PIXELS;
        // 画像マップの場合はLiDARレイ数を減らして高速化
        const lidarRays = useImageMap ? 180 : CONF.lidarRays;
        this.lidar = new Lidar(lidarRays, CONF.maxLidarRange);
        this.path = null;
        this.pathIndex = 0;
        this.canvasWidth = canvasWidth;
        this.canvasHeight = canvasHeight;
    }

    move(v, w, dt) {
        const trueNextTheta = MathUtils.normalizeAngle(this.pose.theta + w * dt);
        const trueNextX = this.pose.x + v * Math.cos(this.pose.theta) * dt;
        const trueNextY = this.pose.y + v * Math.sin(this.pose.theta) * dt;

        let collision = false;
        if (trueNextX < 0 || trueNextX > this.canvasWidth || trueNextY < 0 || trueNextY > this.canvasHeight) {
            collision = true;
        }
        
        if (!collision) {
            this.pose.x = trueNextX;
            this.pose.y = trueNextY;
            this.pose.theta = trueNextTheta;
        }

        const noiseD = Math.abs(v * dt) * CONF.noise.odom_dist;
        const noiseA = Math.abs(w * dt) * CONF.noise.odom_angle;
        const measDist = (v * dt) + MathUtils.gaussianRandom(0, noiseD);
        const measRot = (w * dt) + MathUtils.gaussianRandom(0, noiseA);

        this.odomPose.theta = MathUtils.normalizeAngle(this.odomPose.theta + measRot);
        this.odomPose.x += measDist * Math.cos(this.odomPose.theta);
        this.odomPose.y += measDist * Math.sin(this.odomPose.theta);

        return { lin: measDist, ang: measRot };
    }
}

// --- Particle Filter ---
class ParticleFilter {
    constructor(count, mapWidth, mapHeight, knownMap, useImageMap = false) {
        this.count = count;
        this.width = mapWidth;
        this.height = mapHeight;
        this.map = knownMap;
        // 画像マップの場合はパーティクルのLiDARレイ数をさらに減らす
        const particleLidarRays = useImageMap ? 20 : CONF.particleLidarRays;
        this.simLidar = new Lidar(particleLidarRays, CONF.maxLidarRange);
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
        const xyStdDev = 0.5 * METERS_TO_PIXELS;
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
    
    predict(odomDelta) {
        for (let p of this.particles) {
            const dSig = Math.abs(odomDelta.lin) * 0.15;
            const aSig = Math.abs(odomDelta.ang) * 0.10;
            const noisyDist = odomDelta.lin + MathUtils.gaussianRandom(0, dSig);
            const noisyRot = odomDelta.ang + MathUtils.gaussianRandom(0, aSig);
            p.theta = MathUtils.normalizeAngle(p.theta + noisyRot);
            p.x += noisyDist * Math.cos(p.theta);
            p.y += noisyDist * Math.sin(p.theta);
        }
    }
    
    update(realScan) {
        let maxWeight = 0;
        const stepRatio = Math.floor(realScan.length / CONF.particleLidarRays);
        const sigma2 = 50.0;
        for (let p of this.particles) {
            if (p.x < 0 || p.x > this.width || p.y < 0 || p.y > this.height) {
                p.weight = 0;
                continue;
            }
            const hypoScan = this.simLidar.scan(p, this.map, 0);
            let logLikelihood = 0;
            let validPoints = 0;
            for (let i = 0; i < hypoScan.length; i++) {
                const realDist = realScan[i * stepRatio];
                const hypoDist = hypoScan[i];
                if (realDist >= CONF.maxLidarRange * 0.99 && hypoDist >= CONF.maxLidarRange * 0.99) continue;
                const diff = realDist - hypoDist;
                logLikelihood += -(diff * diff) / sigma2;
                validPoints++;
            }
            if (validPoints > 0) p.weight = Math.exp(logLikelihood / validPoints);
            else p.weight = 0.000001;
            if (p.weight > maxWeight) maxWeight = p.weight;
        }
        let totalWeight = 0;
        for (let p of this.particles) totalWeight += p.weight;
        if (totalWeight > 0) {
            for (let p of this.particles) p.weight /= totalWeight;
        } else {
            this.initGlobal();
        }
    }
    
    resample() {
        const newParticles = [];
        let index = Math.floor(Math.random() * this.count);
        let beta = 0;
        let maxWeight = 0;
        for (let p of this.particles) if(p.weight > maxWeight) maxWeight = p.weight;
        const randomCount = Math.floor(this.count * 0.01);
        for (let i = 0; i < this.count - randomCount; i++) {
            beta += Math.random() * 2 * maxWeight;
            while (beta > this.particles[index].weight) {
                beta -= this.particles[index].weight;
                index = (index + 1) % this.count;
            }
            const p = this.particles[index];
            newParticles.push({ x: p.x, y: p.y, theta: p.theta, weight: p.weight });
        }
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
        let x = 0, y = 0, sinSum = 0, cosSum = 0;
        let totalW = 0;
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
    
    // ランダムなパーティクルを追加（位置推定が不安定な場合のリカバリー用）
    addRandomParticles(count) {
        for (let i = 0; i < count && this.particles.length > count; i++) {
            // 重みが最も低いパーティクルを置き換え
            let minIdx = 0;
            let minWeight = this.particles[0].weight;
            for (let j = 1; j < this.particles.length; j++) {
                if (this.particles[j].weight < minWeight) {
                    minWeight = this.particles[j].weight;
                    minIdx = j;
                }
            }
            this.particles[minIdx] = {
                x: Math.random() * this.width,
                y: Math.random() * this.height,
                theta: Math.random() * Math.PI * 2,
                weight: 1.0 / this.count
            };
        }
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
        
        // 初期位置を決定（指定があればそれを使用、なければ安全な位置を探す）
        let startX, startY, startTheta;
        if (initialPose) {
            startX = initialPose.x;
            startY = initialPose.y;
            startTheta = initialPose.theta || 0;
        } else {
            // 安全な開始位置を探す（壁から十分離れた場所）
            const safePos = this.findSafeStartPosition();
            startX = safePos.x;
            startY = safePos.y;
            startTheta = 0;
        }
        
        console.log(`Initial position: (${startX.toFixed(1)}, ${startY.toFixed(1)}, ${startTheta.toFixed(2)})`);
        
        // ロボットとパーティクルフィルタを同じ位置で初期化
        this.robot = new Robot(startX, startY, startTheta, canvasWidth, canvasHeight, this.useImageMap);
        this.particleFilter = new ParticleFilter(CONF.particleCount, canvasWidth, canvasHeight, this.world.walls, this.useImageMap);
        
        // 初期位置でパーティクルを初期化し、LIDARを使って即座に位置推定を実行
        this.particleFilter.setEstimatedPose(startX, startY, startTheta);
        this.performInitialLocalization();
    }
    
    // 安全な開始位置を探す
    findSafeStartPosition() {
        const minSafetyDist = CONF.safetyDistM * 2; // 壁から十分な距離
        const gridStep = 20; // ピクセル単位でグリッドサーチ
        
        let bestPos = { x: this.canvasWidth * 0.5, y: this.canvasHeight * 0.5 };
        let maxDist = 0;
        
        for (let y = gridStep; y < this.canvasHeight - gridStep; y += gridStep) {
            for (let x = gridStep; x < this.canvasWidth - gridStep; x += gridStep) {
                const col = Math.floor(x / PIXELS_PER_GRID);
                const row = Math.floor(y / PIXELS_PER_GRID);
                
                if (col >= 0 && col < this.costmap.cols && row >= 0 && row < this.costmap.rows) {
                    const idx = row * this.costmap.cols + col;
                    const dist = this.costmap.distMap[idx];
                    
                    if (dist > maxDist && dist > minSafetyDist) {
                        maxDist = dist;
                        bestPos = { x, y };
                    }
                }
            }
        }
        
        return bestPos;
    }
    
    // 初期化時にLIDARを使って位置推定を行う
    performInitialLocalization() {
        console.log('Performing initial localization with LIDAR...');
        
        // 複数回のLIDARスキャンと更新を実行して初期位置を安定させる
        for (let i = 0; i < 10; i++) {
            const realScan = this.robot.lidar.scan(this.robot.pose, this.world.walls, CONF.noise.lidar);
            this.particleFilter.update(realScan);
            this.particleFilter.resample();
        }
        
        // 推定位置でオドメトリを更新
        const est = this.particleFilter.getEstimate();
        this.robot.odomPose = { x: est.x, y: est.y, theta: est.theta };
        
        const error = MathUtils.dist(this.robot.pose, est) / METERS_TO_PIXELS;
        console.log(`Initial localization complete. Position error: ${error.toFixed(3)}m`);
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
        // パーティクルフィルタを指定位置で初期化
        this.particleFilter.setEstimatedPose(x, y, theta);
        this.robot.odomPose = { x, y, theta };
        
        // LIDARを使って位置推定を実行（指定位置を基準に微調整）
        console.log(`Setting initial pose to (${x.toFixed(1)}, ${y.toFixed(1)}, ${theta.toFixed(2)})`);
        
        for (let i = 0; i < 5; i++) {
            const realScan = this.robot.lidar.scan(this.robot.pose, this.world.walls, CONF.noise.lidar);
            this.particleFilter.update(realScan);
            this.particleFilter.resample();
        }
        
        const est = this.particleFilter.getEstimate();
        this.robot.odomPose = { x: est.x, y: est.y, theta: est.theta };
        
        return { success: true };
    }
    
    // グローバルローカライゼーション（マップ全体で位置を探索）
    globalLocalization() {
        console.log('Performing global localization...');
        
        // パーティクルをマップ全体にランダムに散布
        this.particleFilter.initGlobal();
        
        // 複数回のLIDARスキャンで位置を絞り込む
        for (let i = 0; i < 20; i++) {
            const realScan = this.robot.lidar.scan(this.robot.pose, this.world.walls, CONF.noise.lidar);
            this.particleFilter.update(realScan);
            this.particleFilter.resample();
        }
        
        const est = this.particleFilter.getEstimate();
        this.robot.odomPose = { x: est.x, y: est.y, theta: est.theta };
        
        const error = MathUtils.dist(this.robot.pose, est) / METERS_TO_PIXELS;
        console.log(`Global localization complete. Position error: ${error.toFixed(3)}m`);
        
        return { success: true, estimate: est, error };
    }
    
    kidnap() {
        this.robot.pose.x = Math.random() * this.canvasWidth;
        this.robot.pose.y = Math.random() * this.canvasHeight;
        this.robot.pose.theta = Math.random() * Math.PI * 2;
        return { success: true };
    }
    
    computeControl() {
        if (!this.robot.path || this.robot.path.length === 0) {
            return { v: 0, w: 0 };
        }

        const est = this.particleFilter.getEstimate();
        let targetIdx = this.robot.pathIndex;
        const lookAheadDist = 0.5 * METERS_TO_PIXELS;
        
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
        const dx = target.x - est.x;
        const dy = target.y - est.y;
        const distToTarget = Math.hypot(dx, dy);
        
        if (this.robot.pathIndex >= this.robot.path.length - 1 && distToTarget < 0.1 * METERS_TO_PIXELS) {
            this.robot.path = null;
            return { v: 0, w: 0 };
        }

        const targetAngle = Math.atan2(dy, dx);
        const angleDiff = MathUtils.normalizeAngle(targetAngle - est.theta);
        
        let v = 2.0 * METERS_TO_PIXELS;
        let w = angleDiff * 4.0;
        
        if (Math.abs(angleDiff) > 0.5) v = 0;
        else v *= Math.max(0.2, 1.0 - Math.abs(angleDiff));

        return { v, w };
    }
    
    update(dt) {
        const ctrl = this.computeControl();
        const odomDelta = this.robot.move(ctrl.v, ctrl.w, dt);
        
        // パーティクルフィルタの予測ステップ（移動がある場合のみ）
        if (Math.abs(odomDelta.lin) > 0.1 || Math.abs(odomDelta.ang) > 0.001) {
            this.particleFilter.predict(odomDelta);
        }
        
        // 常にLIDARを使って自己位置を修正（移動の有無に関わらず）
        const realScan = this.robot.lidar.scan(this.robot.pose, this.world.walls, CONF.noise.lidar);
        this.particleFilter.update(realScan);
        this.particleFilter.resample();
        
        // 推定位置が大きくずれている場合は警告
        const est = this.particleFilter.getEstimate();
        const posError = MathUtils.dist(this.robot.pose, est) / METERS_TO_PIXELS;
        if (posError > 1.0) {
            // 大きな誤差の場合、パーティクルをより広く散布
            this.particleFilter.addRandomParticles(Math.floor(CONF.particleCount * 0.1));
        }
    }
    
    getState() {
        const est = this.particleFilter.getEstimate();
        const scan = this.robot.lidar.scan(this.robot.pose, this.world.walls, CONF.noise.lidar);
        
        // Convert Float32Array to regular array for JSON serialization
        const scanArray = Array.from(scan);
        
        return {
            robot: {
                pose: this.robot.pose,
                odomPose: this.robot.odomPose,
                radius: this.robot.radius
            },
            estimate: est,
            particles: this.particleFilter.particles,
            scan: scanArray,
            path: this.robot.path,
            walls: this.world.walls,
            costmap: {
                cols: this.costmap.cols,
                rows: this.costmap.rows,
                safetyDistM: CONF.safetyDistM
            },
            stats: {
                posError: MathUtils.dist(this.robot.pose, est) / METERS_TO_PIXELS,
                odomError: MathUtils.dist(this.robot.pose, this.robot.odomPose) / METERS_TO_PIXELS,
                pathLength: this.robot.path ? this.robot.path.length : 0,
                status: this.robot.path ? "FOLLOWING PATH" : "IDLE"
            }
        };
    }
}

module.exports = { SimulationEngine, METERS_TO_PIXELS, CONF };
