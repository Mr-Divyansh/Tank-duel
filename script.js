
// Mobile browsers resize their address bar in and out of the viewport, and
// 100vh/height:100% is measured against the LARGEST possible viewport, not
// the current visible one — this is what made the board look like it was
// "stuck"/cut off between the browser chrome. Measuring window.innerHeight
// directly in JS and re-applying it on every resize fixes that reliably.
function setAppHeight(){
  document.documentElement.style.setProperty('--app-height', window.innerHeight + 'px');
}
setAppHeight();
window.addEventListener('resize', setAppHeight);
window.addEventListener('orientationchange', ()=>setTimeout(setAppHeight, 60));



(function(){
"use strict";

/* =========================================================================
   CONFIG
   ========================================================================= */
const CFG = {
  cellSize: 46,
  cols: 12,
  rows: 12,
  wallThickness: 6,
  marginX: 34,
  marginTop: 118,
  marginBottom: 70,
  tankRadius: 8,
  tankMaxSpeed: 84,        // px/s
  tankAccel: 380,          // px/s^2
  bulletSpeed: 240,        // px/s
  bulletRadius: 3,
  bulletMaxBounces: 5,
  bulletLifetime: 3.6,     // seconds
  fireCooldownMin: 0.55,
  fireCooldownMax: 1.0,
  dangerMargin: 15,
  predictHorizon: 0.95,    // seconds
  predictDt: 1/40,
  losSampleStep: 4,        // px, for line-of-sight raymarching
  fireAngleTolerance: 0.16,// radians - how tight the aim must be to fire
  recoilKick: 5,           // px the turret pulls back on firing
  roundMin: 60,
  roundMax: 120,
  roundTransition: 1.7,
};

const MAZE_W = CFG.cols * CFG.cellSize;
const MAZE_H = CFG.rows * CFG.cellSize;
const CANVAS_W = MAZE_W + CFG.marginX * 2;
const CANVAS_H = MAZE_H + CFG.marginTop + CFG.marginBottom;

const canvas = document.getElementById('c');
const dpr = Math.min(window.devicePixelRatio || 1, 3);
canvas.width = CANVAS_W * dpr;
canvas.height = CANVAS_H * dpr;
const ctx = canvas.getContext('2d');
ctx.scale(dpr, dpr);

// The maze/canvas is always drawn at its full native resolution
// (CANVAS_W x CANVAS_H) — nothing about the maze itself shrinks. What we
// scale is only the on-screen DISPLAY size, so on a small phone screen the
// whole board shrinks down to fit instead of getting cropped at the edges.
function fitCanvasToScreen(){
  const availW = window.innerWidth * 0.97;
  const availH = window.innerHeight * 0.97;
  const scale = Math.min(1, availW / CANVAS_W, availH / CANVAS_H);
  canvas.style.width = Math.round(CANVAS_W * scale) + 'px';
  canvas.style.height = Math.round(CANVAS_H * scale) + 'px';
}
window.addEventListener('resize', fitCanvasToScreen);
window.addEventListener('orientationchange', ()=>setTimeout(fitCanvasToScreen,60));
fitCanvasToScreen();
requestAnimationFrame(()=>canvas.classList.add('ready'));

const dangerRadius = CFG.tankRadius + CFG.bulletRadius + CFG.dangerMargin;

/* =========================================================================
   UTILS / vector.ts equivalent
   ========================================================================= */
function len(x,y){ return Math.sqrt(x*x+y*y); }
function norm(x,y){ const l = len(x,y)||1; return [x/l, y/l]; }
function clamp(v,a,b){ return Math.max(a, Math.min(b, v)); }
function angleLerp(a, b, maxDelta){
  let diff = ((b - a + Math.PI*3) % (Math.PI*2)) - Math.PI;
  if (Math.abs(diff) < maxDelta) return b;
  return a + Math.sign(diff) * maxDelta;
}
function rand(a,b){ return a + Math.random()*(b-a); }
function pick(arr){ return arr[Math.floor(Math.random()*arr.length)]; }

/* =========================================================================
   MazeGenerator.ts equivalent
   ========================================================================= */
function generateMaze(){
  const { cols, rows } = CFG;
  // cell walls: N,S,E,W = true means wall present
  const cells = [];
  for (let i=0;i<cols;i++){
    cells.push([]);
    for (let j=0;j<rows;j++){
      cells[i].push({N:true,S:true,E:true,W:true,visited:false});
    }
  }
  function neighbors(i,j){
    const list = [];
    if (j>0) list.push([i,j-1,'N','S']);
    if (j<rows-1) list.push([i,j+1,'S','N']);
    if (i>0) list.push([i-1,j,'W','E']);
    if (i<cols-1) list.push([i+1,j,'E','W']);
    return list;
  }
  // recursive backtracker (iterative stack to avoid overflow)
  const stack = [[0,0]];
  cells[0][0].visited = true;
  while (stack.length){
    const [ci,cj] = stack[stack.length-1];
    const opts = neighbors(ci,cj).filter(([ni,nj])=>!cells[ni][nj].visited);
    if (!opts.length){ stack.pop(); continue; }
    const [ni,nj,dirA,dirB] = pick(opts);
    cells[ci][cj][dirA] = false;
    cells[ni][nj][dirB] = false;
    cells[ni][nj].visited = true;
    stack.push([ni,nj]);
  }
  // braid: knock down some dead-end / extra walls to open loops & sightlines
  for (let i=0;i<cols;i++){
    for (let j=0;j<rows;j++){
      if (Math.random() < 0.07){
        const opts = neighbors(i,j).filter(([ni,nj,dirA])=>cells[i][j][dirA]);
        if (opts.length){
          const [ni,nj,dirA,dirB] = pick(opts);
          cells[i][j][dirA]=false;
          cells[ni][nj][dirB]=false;
        }
      }
    }
  }

  // Build wall rects + a per-cell lookup for fast local collision queries
  const wallsByCell = [];
  const allWalls = [];
  const t = CFG.wallThickness;
  function ox(i){ return CFG.marginX + i*CFG.cellSize; }
  function oy(j){ return CFG.marginTop + j*CFG.cellSize; }

  for (let i=0;i<cols;i++){
    wallsByCell.push([]);
    for (let j=0;j<rows;j++){
      const c = cells[i][j];
      const local = [];
      if (c.N){
        const r = {x: ox(i)-t/2, y: oy(j)-t/2, w: CFG.cellSize+t, h: t, horiz:true};
        allWalls.push(r); local.push(r);
      }
      if (c.W){
        const r = {x: ox(i)-t/2, y: oy(j)-t/2, w: t, h: CFG.cellSize+t, horiz:false};
        allWalls.push(r); local.push(r);
      }
      if (i===cols-1 && c.E){
        const r = {x: ox(i+1)-t/2, y: oy(j)-t/2, w: t, h: CFG.cellSize+t, horiz:false};
        allWalls.push(r); local.push(r);
      }
      if (j===rows-1 && c.S){
        const r = {x: ox(i)-t/2, y: oy(j+1)-t/2, w: CFG.cellSize+t, h: t, horiz:true};
        allWalls.push(r); local.push(r);
      }
      wallsByCell[i].push(local);
    }
  }

  function cellIndex(x,y){
    const i = clamp(Math.floor((x-CFG.marginX)/CFG.cellSize), 0, cols-1);
    const j = clamp(Math.floor((y-CFG.marginTop)/CFG.cellSize), 0, rows-1);
    return [i,j];
  }
  function nearbyWalls(x,y){
    const [ci,cj] = cellIndex(x,y);
    const out = [];
    for (let di=-1;di<=1;di++){
      for (let dj=-1;dj<=1;dj++){
        const i=ci+di, j=cj+dj;
        if (i<0||j<0||i>=cols||j>=rows) continue;
        const arr = wallsByCell[i][j];
        for (let k=0;k<arr.length;k++) out.push(arr[k]);
      }
    }
    return out;
  }
  function cellCenter(i,j){
    return [ox(i)+CFG.cellSize/2, oy(j)+CFG.cellSize/2];
  }
  // adjacency for BFS pathfinding (open connections between cells)
  function openNeighbors(i,j){
    const c = cells[i][j];
    const list = [];
    if (!c.N && j>0) list.push([i,j-1]);
    if (!c.S && j<rows-1) list.push([i,j+1]);
    if (!c.W && i>0) list.push([i-1,j]);
    if (!c.E && i<cols-1) list.push([i+1,j]);
    return list;
  }

  return { cells, allWalls, cellIndex, nearbyWalls, cellCenter, openNeighbors };
}

/* BFS shortest path between two cells -> array of [i,j] cells */
function bfsPath(maze, start, goal){
  const key = (i,j)=> i+','+j;
  const visited = new Set([key(...start)]);
  const prev = new Map();
  const q = [start];
  let qi = 0;
  while (qi < q.length){
    const cur = q[qi++];
    if (cur[0]===goal[0] && cur[1]===goal[1]) break;
    for (const nb of maze.openNeighbors(cur[0],cur[1])){
      const k = key(...nb);
      if (!visited.has(k)){
        visited.add(k);
        prev.set(k, cur);
        q.push(nb);
      }
    }
  }
  const gk = key(...goal);
  if (!visited.has(gk)) return [start];
  const path = [goal];
  let cur = goal;
  while (!(cur[0]===start[0] && cur[1]===start[1])){
    cur = prev.get(key(...cur));
    path.push(cur);
  }
  path.reverse();
  return path;
}

/* find two cells that are far apart (for spawns) */
function pickSpawnCells(maze){
  let best = null, bestDist = -1;
  for (let tries=0; tries<10; tries++){
    const a = [Math.floor(Math.random()*CFG.cols), Math.floor(Math.random()*CFG.rows)];
    const b = [Math.floor(Math.random()*CFG.cols), Math.floor(Math.random()*CFG.rows)];
    const path = bfsPath(maze, a, b);
    if (path.length > bestDist){ bestDist = path.length; best = [a,b]; }
  }
  return best;
}

/* =========================================================================
   PhysicsEngine / CollisionSystem
   ========================================================================= */
function circleRectOverlap(px,py,r,rect){
  const cx = clamp(px, rect.x, rect.x+rect.w);
  const cy = clamp(py, rect.y, rect.y+rect.h);
  const dx = px-cx, dy = py-cy;
  const d = len(dx,dy);
  if (d < r){
    // push-out normal
    let nx, ny;
    if (d > 0.0001){ nx = dx/d; ny = dy/d; }
    else { nx = rect.horiz?0:1; ny = rect.horiz?1:0; }
    return { hit:true, nx, ny, penetration: r-d };
  }
  return { hit:false };
}

/* Resolve a moving circle (tank) against nearby maze walls + bounds */
function resolveWallCollision(pos, radius, maze, vel){
  const walls = maze.nearbyWalls(pos.x, pos.y);
  for (const w of walls){
    const res = circleRectOverlap(pos.x,pos.y,radius,w);
    if (res.hit){
      pos.x += res.nx * res.penetration;
      pos.y += res.ny * res.penetration;
      if (vel){
        // remove only the velocity component driving INTO the wall so the
        // tank slides smoothly along it instead of snapping to a stop
        const vn = vel.x*res.nx + vel.y*res.ny;
        if (vn < 0){
          vel.x -= vn*res.nx;
          vel.y -= vn*res.ny;
        }
      }
    }
  }
  pos.x = clamp(pos.x, CFG.marginX+radius, CFG.marginX+MAZE_W-radius);
  pos.y = clamp(pos.y, CFG.marginTop+radius, CFG.marginTop+MAZE_H-radius);
}

/* Advance a bullet-like state {x,y,vx,vy} by dt, bouncing off nearby walls.
   Returns collision info if a bounce happened this step (for FX). Mutates state, bounces counter external. */
function stepBulletState(state, dt, maze){
  // substep so a fast bullet can never cross a thin wall within one step
  // without a collision check happening inside it (tunneling)
  const speed = len(state.vx, state.vy);
  const maxStep = CFG.wallThickness * 0.5;
  const numSub = Math.max(1, Math.ceil((speed*dt) / maxStep));
  const subDt = dt/numSub;

  let bounced = false, bx=0, by=0;
  for (let s=0; s<numSub; s++){
    state.x += state.vx*subDt;
    state.y += state.vy*subDt;
    const walls = maze.nearbyWalls(state.x, state.y);
    for (const w of walls){
      const res = circleRectOverlap(state.x, state.y, CFG.bulletRadius, w);
      if (res.hit){
        state.x += res.nx*res.penetration;
        state.y += res.ny*res.penetration;
        if (w.horiz) state.vy = -state.vy; else state.vx = -state.vx;
        bounced = true; bx=state.x; by=state.y;
      }
    }
    // arena bounds also reflect (outer boundary walls already cover this normally,
    // this is a safety net so bullets never leave the visible arena)
    const minX = CFG.marginX+CFG.bulletRadius, maxX = CFG.marginX+MAZE_W-CFG.bulletRadius;
    const minY = CFG.marginTop+CFG.bulletRadius, maxY = CFG.marginTop+MAZE_H-CFG.bulletRadius;
    if (state.x < minX){ state.x = minX; state.vx = Math.abs(state.vx); bounced=true; }
    if (state.x > maxX){ state.x = maxX; state.vx = -Math.abs(state.vx); bounced=true; }
    if (state.y < minY){ state.y = minY; state.vy = Math.abs(state.vy); bounced=true; }
    if (state.y > maxY){ state.y = maxY; state.vy = -Math.abs(state.vy); bounced=true; }
  }
  return { bounced, x:bx, y:by };
}

/* =========================================================================
   TrajectoryPredictor.ts equivalent
   ========================================================================= */
function evaluateThreat(bullet, maze, tankPos, horizon){
  const sim = { x: bullet.x, y: bullet.y, vx: bullet.vx, vy: bullet.vy };
  const dt = CFG.predictDt;
  let bounces = bullet.bounces;
  let t = 0;
  const trajectory = [{t:0, x:sim.x, y:sim.y}];
  while (t < horizon && bounces <= CFG.bulletMaxBounces){
    const res = stepBulletState(sim, dt, maze);
    if (res.bounced) bounces++;
    t += dt;
    trajectory.push({t, x:sim.x, y:sim.y});
    const d = len(sim.x-tankPos.x, sim.y-tankPos.y);
    if (d < dangerRadius) return { hit:true, t, point:{x:sim.x,y:sim.y}, trajectory };
    if (bounces > CFG.bulletMaxBounces) break;
  }
  return { hit:false };
}

/* True only if a straight line between the two points never passes through
   a wall — tanks use this so they only fire when they can actually SEE
   each other, instead of shooting blindly into a wall between them. */
function hasLineOfSight(ax, ay, bx, by, maze){
  const dx = bx-ax, dy = by-ay;
  const dist = len(dx,dy);
  const steps = Math.max(1, Math.ceil(dist / CFG.losSampleStep));
  for (let i=1; i<steps; i++){
    const t = i/steps;
    const x = ax+dx*t, y = ay+dy*t;
    const walls = maze.nearbyWalls(x,y);
    for (const w of walls){
      if (x >= w.x && x <= w.x+w.w && y >= w.y && y <= w.y+w.h) return false;
    }
  }
  return true;
}

/* Choose a safe short displacement for the tank given active threats */
function calculateSafePosition(tank, threats, maze){
  const N = 16;
  let best = null, bestScore = -Infinity;
  const urgency = threats.length ? Math.min(...threats.map(t=>t.t)) : 1;
  const dodgeDist = clamp(30 * (1.3 - urgency), 14, 40);

  for (let k=0;k<N;k++){
    const ang = (k/N)*Math.PI*2;
    const cx = tank.pos.x + Math.cos(ang)*dodgeDist;
    const cy = tank.pos.y + Math.sin(ang)*dodgeDist;
    // reject if inside a wall
    let blocked = false;
    for (const w of maze.nearbyWalls(cx,cy)){
      if (circleRectOverlap(cx,cy,CFG.tankRadius+2,w).hit){ blocked = true; break; }
    }
    if (cx < CFG.marginX+CFG.tankRadius || cx > CFG.marginX+MAZE_W-CFG.tankRadius) blocked = true;
    if (cy < CFG.marginTop+CFG.tankRadius || cy > CFG.marginTop+MAZE_H-CFG.tankRadius) blocked = true;
    if (blocked) continue;

    // score: minimize proximity to every threat's near-term path, prefer small movement
    let minThreatDist = Infinity;
    for (const th of threats){
      for (const s of th.trajectory){
        if (s.t > 0.5) break;
        const d = len(s.x-cx, s.y-cy);
        if (d < minThreatDist) minThreatDist = d;
      }
    }
    const moveCost = len(cx-tank.pos.x, cy-tank.pos.y) * 0.15;
    const score = minThreatDist - moveCost;
    if (score > bestScore){ bestScore = score; best = {x:cx,y:cy}; }
  }
  return best || tank.pos;
}

/* =========================================================================
   Tank.ts / TankAI.ts (DodgeController folded in)
   ========================================================================= */
class Tank {
  constructor(id, color, pos, bulletColor){
    this.id = id;
    this.color = color;
    this.bulletColor = bulletColor || color;
    this.pos = {x:pos.x, y:pos.y};
    this.vel = {x:0, y:0};
    this.bodyAngle = 0;
    this.turretAngle = 0;
    this.cooldown = rand(CFG.fireCooldownMin, CFG.fireCooldownMax);
    this.path = [];
    this.pathTimer = 0;
    this.dodgeTarget = null;
    this.dodgeTimer = 0;
    this.trail = [];
    this.stats = { shots:0, dodges:0 };
    this.wasThreatened = false;
    this.strafeSeed = Math.random()*1000;
    this.dodgeGlow = 0;
    this.recoil = 0;
    this.ghosts = [];
    this.orbitState = 'approach'; // approach | hold | retreat (hysteresis-based)
    this.retreatTarget = null;
    this.retreatTimer = 0;
    this.dodgeTargetTimer = 0;
    this.time = Math.random()*100;
  }

  currentCell(maze){ return maze.cellIndex(this.pos.x, this.pos.y); }

  recomputePath(maze, opponent){
    const start = this.currentCell(maze);
    const goal = opponent.currentCell(maze);
    this.path = bfsPath(maze, start, goal);
    this.pathTimer = rand(0.9, 1.4);
  }

  desiredCombatTarget(maze, opponent, dt, los){
    this.time += dt;
    this.pathTimer -= dt;
    if (this.pathTimer <= 0 || this.path.length < 2) this.recomputePath(maze, opponent);

    if (!los){
      // Blocked by a wall: being "close" in a straight line means nothing if
      // there's no shot, so ALWAYS keep hunting through the maze here rather
      // than switching to a stationary hold/orbit. Every couple of seconds,
      // sometimes peel off toward a random point along the route instead of
      // always beelining for the opponent's exact cell — this reads as an
      // active search instead of a robotic tunnel-vision approach.
      this.orbitState = 'seek';
      this.searchTimer = (this.searchTimer === undefined ? 0 : this.searchTimer) - dt;
      if (this.searchTimer <= 0){
        this.searchTimer = rand(1.3, 2.4);
        this.wanderPick = Math.random() < 0.35;
      }
      if (this.path.length >= 2){
        const wpIdx = this.wanderPick
          ? clamp(Math.floor(rand(1, this.path.length)), 1, this.path.length-1)
          : clamp(3, 1, this.path.length-1);
        const [wi,wj] = this.path[wpIdx];
        const [wx,wy] = maze.cellCenter(wi,wj);
        return {x:wx, y:wy};
      }
      return this.pos;
    }

    const dist = len(opponent.pos.x-this.pos.x, opponent.pos.y-this.pos.y);

    // hysteresis bands so the tank commits to a mode instead of flickering
    // between "too close" and "too far" every frame — only meaningful once
    // there's an actual sightline to fight along
    if (this.orbitState !== 'approach' && this.orbitState !== 'hold' && this.orbitState !== 'retreat') this.orbitState = 'approach';
    if (this.orbitState === 'approach' && dist < 165) this.orbitState = 'hold';
    else if (this.orbitState === 'hold' && dist > 240) this.orbitState = 'approach';
    if (dist < 85) this.orbitState = 'retreat';
    else if (this.orbitState === 'retreat' && dist > 150) this.orbitState = 'hold';

    if (this.orbitState === 'approach' && this.path.length >= 2){
      // look a couple of cells ahead along the path toward the opponent for
      // a smoother, less twitchy heading than always aiming at the very next cell
      const wpIdx = clamp(2, 1, this.path.length-1);
      const [wi,wj] = this.path[wpIdx];
      const [wx,wy] = maze.cellCenter(wi,wj);
      return {x:wx, y:wy};
    }

    if (this.orbitState === 'retreat'){
      // genuinely move AWAY: greedily step to whichever open neighbor cell
      // is farthest from the opponent, re-picked only occasionally so the
      // tank commits to a direction instead of vibrating in place
      this.retreatTimer -= dt;
      if (!this.retreatTarget || this.retreatTimer <= 0){
        const [ci,cj] = this.currentCell(maze);
        const options = maze.openNeighbors(ci,cj);
        options.push([ci,cj]);
        let best = null, bestD = -1;
        for (const [oi,oj] of options){
          const [cx,cy] = maze.cellCenter(oi,oj);
          const d = len(cx-opponent.pos.x, cy-opponent.pos.y);
          if (d > bestD){ bestD = d; best = {x:cx,y:cy}; }
        }
        this.retreatTarget = best;
        this.retreatTimer = rand(0.5, 0.9);
      }
      return this.retreatTarget || this.pos;
    }

    // hold: sit at a comfortable mid-range and drift side to side rather than
    // freezing dead still — reads as an alert, alive stance
    const [dxo,dyo] = norm(opponent.pos.x-this.pos.x, opponent.pos.y-this.pos.y);
    const perpX = -dyo, perpY = dxo;
    const strafe = Math.sin(this.time*0.9 + this.strafeSeed) * 26;
    return { x:this.pos.x + perpX*strafe*dt*2, y:this.pos.y + perpY*strafe*dt*2 };
  }

  update(dt, maze, opponent, bullets, spawnFire, particles){
    // ---- 1. perceive incoming threats — but ONLY from bullets the tank can
    // actually see right now. A bullet still hidden behind a wall gives no
    // information, even if it will eventually ricochet into view. ----
    const enemyBullets = bullets.filter(b=>b.alive && b.owner !== this.id
      && hasLineOfSight(this.pos.x, this.pos.y, b.x, b.y, maze));
    const threats = [];
    for (const b of enemyBullets){
      const res = evaluateThreat(b, maze, this.pos, CFG.predictHorizon);
      if (res.hit) threats.push({ bullet:b, t:res.t, point:res.point, trajectory:res.trajectory });
    }
    const threatened = threats.length > 0;
    if (threatened && !this.wasThreatened){
      this.stats.dodges++;
      this.dodgeGlow = 1;
      particles.push(makeDodgeRing(this.pos.x, this.pos.y, this.color));
    }
    this.wasThreatened = threatened;

    if (threatened){
      // leave a quick afterimage ghost while actively evading — this is what
      // reads as a sharp, intentional dash rather than a random jitter
      this.ghosts.push({x:this.pos.x, y:this.pos.y, bodyAngle:this.bodyAngle, age:0});
      if (this.ghosts.length > 5) this.ghosts.shift();
    }
    this.dodgeGlow *= 0.88;
    this.recoil *= 0.82;
    for (const g of this.ghosts) g.age += dt;
    this.ghosts = this.ghosts.filter(g=>g.age < 0.22);

    // ---- 1.5 line of sight — computed early so movement can react to it too ----
    const los = hasLineOfSight(this.pos.x, this.pos.y, opponent.pos.x, opponent.pos.y, maze);

    // ---- 2. decide movement target ----
    let target;
    if (threatened){
      this.dodgeTargetTimer -= dt;
      if (!this.dodgeTarget || this.dodgeTargetTimer <= 0){
        this.dodgeTarget = calculateSafePosition(this, threats, maze);
        this.dodgeTargetTimer = 0.14; // hold the chosen escape point briefly so the dash reads as one clean move, not a flicker
      }
      target = this.dodgeTarget;
      this.dodgeTimer = 0.3;
    } else if (this.dodgeTimer > 0){
      this.dodgeTimer -= dt;
      this.dodgeTarget = null;
      target = this.pos; // settle briefly after a dodge before resuming path
    } else {
      this.dodgeTarget = null;
      target = this.desiredCombatTarget(maze, opponent, dt, los);
    }

    // ---- 3. steer toward target (bounded accel/speed => smooth natural motion) ----
    const dx = target.x - this.pos.x, dy = target.y - this.pos.y;
    const d = len(dx,dy);
    let desiredVX = 0, desiredVY = 0;
    if (d > 2){
      const [nx,ny] = norm(dx,dy);
      const speedScale = threatened ? 1 : clamp(d/40, 0.25, 1);
      desiredVX = nx * CFG.tankMaxSpeed * speedScale;
      desiredVY = ny * CFG.tankMaxSpeed * speedScale;

      // ease speed down for a sharp turn instead of snapping straight onto the
      // new heading at full speed — this alone removes most of the "twitchy" look
      const curSpeed = len(this.vel.x, this.vel.y);
      if (curSpeed > 6){
        const curAngle = Math.atan2(this.vel.y, this.vel.x);
        const desAngle = Math.atan2(desiredVY, desiredVX);
        const turnDiff = Math.abs(((desAngle-curAngle+Math.PI*3)%(Math.PI*2))-Math.PI);
        const turnPenalty = threatened ? 1 : clamp(1 - (turnDiff/Math.PI)*0.65, 0.35, 1);
        desiredVX *= turnPenalty; desiredVY *= turnPenalty;
      }
    }
    const ax = clamp(desiredVX - this.vel.x, -CFG.tankAccel*dt, CFG.tankAccel*dt);
    const ay = clamp(desiredVY - this.vel.y, -CFG.tankAccel*dt, CFG.tankAccel*dt);
    this.vel.x += ax; this.vel.y += ay;

    this.pos.x += this.vel.x*dt;
    this.pos.y += this.vel.y*dt;
    resolveWallCollision(this.pos, CFG.tankRadius, maze, this.vel);

    if (len(this.vel.x,this.vel.y) > 6){
      this.bodyAngle = angleLerp(this.bodyAngle, Math.atan2(this.vel.y,this.vel.x), 5*dt);
    }

    // ---- 4. aim (uses the line-of-sight computed above) ----
    let desiredTurret;
    if (los){
      // opponent is visible: lead the shot using their velocity
      const leadTime = clamp(len(opponent.pos.x-this.pos.x, opponent.pos.y-this.pos.y)/CFG.bulletSpeed, 0, 0.6);
      const aimX = opponent.pos.x + opponent.vel.x*leadTime;
      const aimY = opponent.pos.y + opponent.vel.y*leadTime;
      desiredTurret = Math.atan2(aimY-this.pos.y, aimX-this.pos.x);
      this.turretAngle = angleLerp(this.turretAngle, desiredTurret, 6*dt);
    } else {
      // NO sightline = NO information about the opponent, period. The turret
      // must not secretly track their real position — instead it just faces
      // wherever the tank is actually moving (or does a slow idle scan if
      // it's holding still), which is what makes the searching look genuine.
      const speed = len(this.vel.x, this.vel.y);
      if (speed > 10){
        desiredTurret = Math.atan2(this.vel.y, this.vel.x);
      } else {
        desiredTurret = this.bodyAngle + Math.sin(this.time*0.6)*1.1;
      }
      this.turretAngle = angleLerp(this.turretAngle, desiredTurret, 1.8*dt);
    }

    // ---- 5. fire — ONLY when there is a clear line of sight and the turret
    // is actually lined up on the opponent right now ----
    this.cooldown -= dt;
    const aimDiff = Math.abs(((desiredTurret-this.turretAngle+Math.PI*3)%(Math.PI*2))-Math.PI);
    if (los && this.cooldown <= 0 && aimDiff < CFG.fireAngleTolerance && !threatened){
      this.cooldown = rand(CFG.fireCooldownMin, CFG.fireCooldownMax);
      this.stats.shots++;
      spawnFire(this);
    }

    // trail
    this.trail.push({x:this.pos.x,y:this.pos.y,a:1});
    if (this.trail.length > 10) this.trail.shift();
    for (const p of this.trail) p.a *= 0.88;
  }
}

/* =========================================================================
   Bullet.ts
   ========================================================================= */
let bulletId = 0;
class Bullet {
  constructor(owner, x, y, vx, vy, color){
    this.id = bulletId++;
    this.owner = owner;
    this.x = x; this.y = y; this.vx = vx; this.vy = vy;
    this.color = color;
    this.bounces = 0;
    this.age = 0;
    this.alive = true;
    this.trail = [];
  }
  step(dt, maze, particles){
    this.trail.push({x:this.x,y:this.y});
    if (this.trail.length > 5) this.trail.shift();
    const res = stepBulletState(this, dt, maze);
    if (res.bounced){
      this.bounces++;
      particles.push(makeFlash(res.x, res.y, this.color));
      particles.push(makeSpark(res.x, res.y, this.color));
    }
    this.age += dt;
    if (this.age > CFG.bulletLifetime || this.bounces > CFG.bulletMaxBounces) this.alive = false;
  }
}

function makeFlash(x,y,color){ return { x, y, r: 2, life: 0.16, age: 0, type:'flash', color: color||'#ffe08a' }; }
function makeMuzzle(x,y,color){ return { x, y, r: 4, life: 0.12, age: 0, type:'muzzle', color: color||'#fff3c4' }; }
function makeDodgeRing(x,y,color){ return { x, y, r: CFG.tankRadius, life: 0.32, age: 0, type:'ring', color }; }
function makeSpark(x,y,color){
  const ang = rand(0, Math.PI*2);
  const spd = rand(50,130);
  return { x, y, vx:Math.cos(ang)*spd, vy:Math.sin(ang)*spd, r:0, life: rand(0.12,0.22), age:0, type:'spark', color: color||'#ffe98a' };
}

/* =========================================================================
   RoundManager / GameEngine
   ========================================================================= */
class Game {
  constructor(){
    this.round = 0;
    this.particles = [];
    this.bullets = [];
    this.transitionTimer = 0;
    this.state = 'playing';
    this.bgDust = [];
    for (let i=0;i<10;i++){
      this.bgDust.push({
        x: rand(0, CANVAS_W), y: rand(0, CANVAS_H),
        r: rand(0.5, 1.3), vx: rand(-3,3), vy: rand(-3,3),
        a: rand(0.05, 0.14),
      });
    }
    this.startNewRound();
    this.last = performance.now();
    requestAnimationFrame(this.loop.bind(this));
  }

  startNewRound(){
    this.round++;
    this.maze = generateMaze();
    const [a,b] = pickSpawnCells(this.maze);
    const [ax,ay] = this.maze.cellCenter(a[0],a[1]);
    const [bx,by] = this.maze.cellCenter(b[0],b[1]);
    this.tanks = [
      new Tank('red', '#e6483b', {x:ax,y:ay}, '#ff6b52'),
      new Tank('black', '#20232a', {x:bx,y:by}, '#5aa9ff'),
    ];
    this.bullets = [];
    this.particles = [];
    this.introTimer = 1.4;
    this.roundDuration = rand(CFG.roundMin, CFG.roundMax);
    this.roundTime = 0;
    this.state = 'playing';
  }

  spawnFire(tank){
    const muzzleDist = CFG.tankRadius + 6;
    const mx = tank.pos.x + Math.cos(tank.turretAngle)*muzzleDist;
    const my = tank.pos.y + Math.sin(tank.turretAngle)*muzzleDist;
    const vx = Math.cos(tank.turretAngle)*CFG.bulletSpeed;
    const vy = Math.sin(tank.turretAngle)*CFG.bulletSpeed;
    this.bullets.push(new Bullet(tank.id, mx, my, vx, vy, tank.bulletColor));
    this.particles.push(makeMuzzle(mx,my,tank.bulletColor));
    tank.recoil = 1;
  }

  update(dt){
    for (const p of this.bgDust){
      p.x += p.vx*dt; p.y += p.vy*dt;
      if (p.x < 0) p.x += CANVAS_W; if (p.x > CANVAS_W) p.x -= CANVAS_W;
      if (p.y < 0) p.y += CANVAS_H; if (p.y > CANVAS_H) p.y -= CANVAS_H;
    }
    if (this.introTimer > 0) this.introTimer -= dt;

    if (this.state === 'transition'){
      this.transitionTimer -= dt;
      if (this.transitionTimer <= 0) this.startNewRound();
      return;
    }

    const [t0,t1] = this.tanks;
    t0.update(dt, this.maze, t1, this.bullets, this.spawnFire.bind(this), this.particles);
    t1.update(dt, this.maze, t0, this.bullets, this.spawnFire.bind(this), this.particles);

    for (const b of this.bullets) b.step(dt, this.maze, this.particles);
    this.bullets = this.bullets.filter(b=>b.alive);

    // absolute safety net: guarantee no visual overlap between a bullet and a tank
    for (const tank of this.tanks){
      for (const b of this.bullets){
        if (b.owner === tank.id) continue;
        const d = len(b.x-tank.pos.x, b.y-tank.pos.y);
        const minD = CFG.tankRadius + CFG.bulletRadius + 1;
        if (d < minD){
          const [nx,ny] = norm(tank.pos.x-b.x, tank.pos.y-b.y);
          tank.pos.x += nx*(minD-d);
          tank.pos.y += ny*(minD-d);
          resolveWallCollision(tank.pos, CFG.tankRadius, this.maze);
        }
      }
    }

    for (const p of this.particles){
      if (p.type === 'spark'){ p.x += p.vx*dt; p.y += p.vy*dt; }
      p.age += dt;
    }
    this.particles = this.particles.filter(p=>p.age < p.life);

    this.roundTime += dt;
    if (this.roundTime >= this.roundDuration){
      this.state = 'transition';
      this.transitionTimer = CFG.roundTransition;
    }
  }

  loop(now){
    let dt = (now - this.last)/1000;
    this.last = now;
    dt = Math.min(dt, 0.033); // clamp for tab-switch hitches
    this.update(dt);
    render(this);
    requestAnimationFrame(this.loop.bind(this));
  }
}

/* =========================================================================
   RENDERING
   ========================================================================= */
function drawMaze(game){
  ctx.fillStyle = '#f5f4f0';
  ctx.fillRect(CFG.marginX, CFG.marginTop, MAZE_W, MAZE_H);

  // soft vignette on the floor for depth instead of a flat fill
  const vg = ctx.createRadialGradient(
    CFG.marginX+MAZE_W/2, CFG.marginTop+MAZE_H/2, MAZE_W*0.2,
    CFG.marginX+MAZE_W/2, CFG.marginTop+MAZE_H/2, MAZE_W*0.75
  );
  vg.addColorStop(0, 'rgba(0,0,0,0)');
  vg.addColorStop(1, 'rgba(0,0,0,0.10)');
  ctx.fillStyle = vg;
  ctx.fillRect(CFG.marginX, CFG.marginTop, MAZE_W, MAZE_H);

  ctx.strokeStyle = 'rgba(0,0,0,0.06)';
  ctx.lineWidth = 1;
  ctx.strokeRect(CFG.marginX, CFG.marginTop, MAZE_W, MAZE_H);

  for (const w of game.maze.allWalls){
    ctx.fillStyle = '#3a3d42';
    ctx.fillRect(w.x, w.y, w.w, w.h);
    // subtle top/left highlight + bottom/right shade for a beveled, less flat look
    ctx.fillStyle = 'rgba(255,255,255,0.10)';
    if (w.horiz) ctx.fillRect(w.x, w.y, w.w, 1.4);
    else ctx.fillRect(w.x, w.y, 1.4, w.h);
    ctx.fillStyle = 'rgba(0,0,0,0.18)';
    if (w.horiz) ctx.fillRect(w.x, w.y+w.h-1.4, w.w, 1.4);
    else ctx.fillRect(w.x+w.w-1.4, w.y, 1.4, w.h);
  }

  // tactical-HUD corner brackets framing the arena
  const bx = CFG.marginX, by = CFG.marginTop, bw = MAZE_W, bh = MAZE_H, L = 16;
  ctx.strokeStyle = '#8b93ff';
  ctx.lineWidth = 2.5;
  ctx.lineCap = 'round';
  const corners = [
    [bx,by, 1,1], [bx+bw,by, -1,1], [bx,by+bh, 1,-1], [bx+bw,by+bh, -1,-1],
  ];
  for (const [x,y,sx,sy] of corners){
    ctx.beginPath();
    ctx.moveTo(x, y+L*sy);
    ctx.lineTo(x, y);
    ctx.lineTo(x+L*sx, y);
    ctx.stroke();
  }
}

function drawTankShape(id, glowColor, glowAmount){
  if (glowAmount > 0.01){
    ctx.save();
    ctx.shadowColor = glowColor;
    ctx.shadowBlur = 14 * glowAmount;
  }
  if (id === 'red'){
    roundRect(ctx, -CFG.tankRadius, -CFG.tankRadius, CFG.tankRadius*2, CFG.tankRadius*2, 3);
    ctx.fill();
  } else {
    ctx.beginPath();
    ctx.moveTo(0,-CFG.tankRadius*1.15);
    ctx.lineTo(CFG.tankRadius*1.15,0);
    ctx.lineTo(0,CFG.tankRadius*1.15);
    ctx.lineTo(-CFG.tankRadius*1.15,0);
    ctx.closePath();
    ctx.fill();
  }
  if (glowAmount > 0.01) ctx.restore();
}

function drawTank(tank){
  // subtle ambient movement trail (always on, very faint)
  for (let i=0;i<tank.trail.length;i++){
    const p = tank.trail[i];
    ctx.beginPath();
    ctx.fillStyle = tank.color;
    ctx.globalAlpha = p.a * 0.12;
    ctx.arc(p.x, p.y, CFG.tankRadius*0.55, 0, Math.PI*2);
    ctx.fill();
  }
  ctx.globalAlpha = 1;

  // sharp glowing afterimage ghosts, only while actively dodging — this is
  // what sells the "quick, intelligent sidestep" look instead of a jitter
  for (const g of tank.ghosts){
    const t = 1 - g.age/0.22;
    ctx.save();
    ctx.translate(g.x, g.y);
    ctx.rotate(g.bodyAngle);
    ctx.globalAlpha = t * 0.4;
    ctx.strokeStyle = tank.color;
    ctx.shadowColor = tank.color;
    ctx.shadowBlur = 6 * t;
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    if (tank.id === 'red'){
      ctx.rect(-CFG.tankRadius, -CFG.tankRadius, CFG.tankRadius*2, CFG.tankRadius*2);
    } else {
      ctx.moveTo(0,-CFG.tankRadius*1.15);
      ctx.lineTo(CFG.tankRadius*1.15,0);
      ctx.lineTo(0,CFG.tankRadius*1.15);
      ctx.lineTo(-CFG.tankRadius*1.15,0);
      ctx.closePath();
    }
    ctx.stroke();
    ctx.restore();
  }
  ctx.globalAlpha = 1;

  // always-on soft "energy core" glow, brightening sharply during a dodge
  const ambientGlow = 0.08 + tank.dodgeGlow*0.85;
  ctx.save();
  ctx.translate(tank.pos.x, tank.pos.y);
  ctx.rotate(tank.bodyAngle);
  ctx.fillStyle = tank.color;
  drawTankShape(tank.id, tank.dodgeGlow > 0.05 ? '#ffffff' : tank.color, ambientGlow);
  ctx.restore();

  // turret + muzzle (drawn unrotated by body angle, only turret angle) with
  // a small recoil kick on firing for mechanical "punch"
  const kick = tank.recoil * CFG.recoilKick;
  ctx.save();
  ctx.translate(tank.pos.x, tank.pos.y);
  ctx.rotate(tank.turretAngle);
  ctx.strokeStyle = tank.color;
  ctx.lineWidth = 3.2;
  ctx.lineCap = 'round';
  ctx.beginPath();
  ctx.moveTo(-kick*0.4, 0);
  ctx.lineTo(CFG.tankRadius+6-kick, 0);
  ctx.stroke();
  if (tank.recoil > 0.08){
    ctx.fillStyle = '#fffaf0';
    ctx.globalAlpha = tank.recoil;
    ctx.shadowColor = tank.bulletColor;
    ctx.shadowBlur = 6;
    ctx.beginPath();
    ctx.arc(CFG.tankRadius+6-kick, 0, 2.2*tank.recoil, 0, Math.PI*2);
    ctx.fill();
    ctx.globalAlpha = 1;
  }
  ctx.restore();
}

function roundRect(ctx,x,y,w,h,r){
  ctx.beginPath();
  ctx.moveTo(x+r,y);
  ctx.arcTo(x+w,y,x+w,y+h,r);
  ctx.arcTo(x+w,y+h,x,y+h,r);
  ctx.arcTo(x,y+h,x,y,r);
  ctx.arcTo(x,y,x+w,y,r);
  ctx.closePath();
}

function drawBullet(b){
  // one tapered tracer stroke instead of a stack of overlapping trail circles
  if (b.trail.length >= 2){
    const tail = b.trail[0];
    const grad = ctx.createLinearGradient(tail.x, tail.y, b.x, b.y);
    grad.addColorStop(0, 'rgba(255,255,255,0)');
    grad.addColorStop(1, b.color || '#ffd23f');
    ctx.strokeStyle = grad;
    ctx.lineWidth = CFG.bulletRadius*1.1;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(tail.x, tail.y);
    for (let i=1;i<b.trail.length;i++) ctx.lineTo(b.trail[i].x, b.trail[i].y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
  }
  // small white-hot core with a soft tint of the owner's color
  ctx.save();
  ctx.shadowColor = b.color || '#ffd23f';
  ctx.shadowBlur = 4;
  ctx.fillStyle = '#fffaf0';
  ctx.beginPath();
  ctx.arc(b.x,b.y,CFG.bulletRadius*0.75,0,Math.PI*2);
  ctx.fill();
  ctx.restore();
}

function drawParticles(particles){
  for (const p of particles){
    const t = 1 - p.age/p.life;
    if (p.type === 'flash'){
      ctx.globalAlpha = t*0.75;
      ctx.fillStyle = p.color;
      ctx.beginPath();
      ctx.arc(p.x,p.y,p.r*(1.3-t*0.5),0,Math.PI*2);
      ctx.fill();
    } else if (p.type === 'spark'){
      ctx.globalAlpha = t*0.8;
      ctx.strokeStyle = p.color;
      ctx.lineWidth = 1.2;
      const [nx,ny] = norm(p.vx,p.vy);
      ctx.beginPath();
      ctx.moveTo(p.x, p.y);
      ctx.lineTo(p.x - nx*5, p.y - ny*5);
      ctx.stroke();
    } else if (p.type === 'ring'){
      ctx.globalAlpha = t*0.7;
      ctx.strokeStyle = p.color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(p.x,p.y, p.r + (1-t)*20, 0, Math.PI*2);
      ctx.stroke();
    } else {
      ctx.globalAlpha = t*0.7;
      ctx.fillStyle = p.color;
      ctx.beginPath();
      ctx.arc(p.x,p.y,p.r*(1+ (1-t)*1.3),0,Math.PI*2);
      ctx.fill();
    }
  }
  ctx.globalAlpha = 1;
}

function drawHUD(game){
  ctx.textAlign = 'center';
  ctx.shadowColor = 'rgba(139,147,255,0.4)';
  ctx.shadowBlur = 6;
  ctx.fillStyle = '#f0f0f4';
  ctx.font = '700 20px Courier New';
  ctx.fillText('AI TANK DUEL', CANVAS_W/2, 34);
  ctx.shadowBlur = 0;
  ctx.strokeStyle = 'rgba(88,101,242,0.5)';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(CANVAS_W/2-46, 40);
  ctx.lineTo(CANVAS_W/2+46, 40);
  ctx.stroke();

  ctx.font = '600 13px Courier New';
  ctx.fillStyle = '#9a9a9f';
  ctx.fillText('ROUND ' + String(game.round).padStart(2,'0'), CANVAS_W/2, 58);

  const [t0,t1] = game.tanks;
  ctx.textAlign = 'left';
  ctx.font = '600 12px Courier New';
  ctx.fillStyle = '#e6483b';
  ctx.fillText('RED', CFG.marginX, 82);
  ctx.fillStyle = '#c9c9cc';
  ctx.font = '11px Courier New';
  ctx.fillText('Shots: ' + t0.stats.shots, CFG.marginX, 98);
  ctx.fillText('Dodges: ' + t0.stats.dodges, CFG.marginX, 112);

  ctx.textAlign = 'right';
  ctx.font = '600 12px Courier New';
  ctx.fillStyle = '#3a3d42';
  ctx.fillText('BLACK', CANVAS_W-CFG.marginX, 82);
  ctx.fillStyle = '#c9c9cc';
  ctx.font = '11px Courier New';
  ctx.fillText('Shots: ' + t1.stats.shots, CANVAS_W-CFG.marginX, 98);
  ctx.fillText('Dodges: ' + t1.stats.dodges, CANVAS_W-CFG.marginX, 112);

  ctx.textAlign = 'center';
  ctx.fillStyle = '#7a7a80';
  ctx.font = '11px Courier New';
  ctx.fillText('ACTIVE BULLETS: ' + game.bullets.length, CANVAS_W/2, CANVAS_H-42);
  ctx.fillText('RED AI: ACTIVE   |   BLACK AI: ACTIVE', CANVAS_W/2, CANVAS_H-24);

  if (game.state === 'transition'){
    const p = clamp(1 - game.transitionTimer / CFG.roundTransition, 0, 1); // 0 -> 1 over the transition
    const fadeIn = clamp(p/0.25, 0, 1);
    const overlayAlpha = 0.8 * fadeIn;

    ctx.fillStyle = `rgba(8,8,10,${overlayAlpha})`;
    ctx.fillRect(0,0,CANVAS_W,CANVAS_H);

    // scanline sweeping down the arena — reads as "rebuilding the maze"
    const scanY = CFG.marginTop + p * MAZE_H;
    const grad = ctx.createLinearGradient(0, scanY-40, 0, scanY+40);
    grad.addColorStop(0, 'rgba(88,101,242,0)');
    grad.addColorStop(0.5, 'rgba(120,150,255,0.55)');
    grad.addColorStop(1, 'rgba(88,101,242,0)');
    ctx.fillStyle = grad;
    ctx.fillRect(CFG.marginX, scanY-40, MAZE_W, 80);

    const textScale = 0.85 + 0.15*clamp(fadeIn*2,0,1);
    ctx.save();
    ctx.translate(CANVAS_W/2, CANVAS_H/2);
    ctx.scale(textScale, textScale);
    ctx.globalAlpha = Math.min(fadeIn*1.4, 1);
    ctx.shadowColor = 'rgba(88,101,242,0.6)';
    ctx.shadowBlur = 14;
    ctx.fillStyle = '#f5f4f0';
    ctx.font = '700 22px Courier New';
    ctx.fillText('ROUND ' + String(game.round).padStart(2,'0') + ' COMPLETE', 0, -10);
    ctx.shadowBlur = 0;
    ctx.font = '12px Courier New';
    ctx.fillStyle = '#9a9a9f';
    ctx.fillText('Generating new maze...', 0, 16);
    ctx.restore();
    ctx.globalAlpha = 1;
  }
}

function drawBgDust(game){
  for (const p of game.bgDust){
    ctx.globalAlpha = p.a;
    ctx.fillStyle = '#8b93ff';
    ctx.beginPath();
    ctx.arc(p.x, p.y, p.r, 0, Math.PI*2);
    ctx.fill();
  }
  ctx.globalAlpha = 1;
}

function drawIntroSplash(game){
  if (game.introTimer <= 0) return;
  const p = clamp(1 - game.introTimer/1.4, 0, 1);
  const fadeIn = clamp(p/0.2, 0, 1);
  const fadeOut = clamp((1-p)/0.25, 0, 1);
  const a = Math.min(fadeIn, fadeOut);
  const slide = (1-fadeIn) * 60;

  ctx.save();
  ctx.globalAlpha = a;
  ctx.textAlign = 'center';
  ctx.font = '800 30px Courier New';

  ctx.shadowColor = '#e6483b';
  ctx.shadowBlur = 16;
  ctx.fillStyle = '#e6483b';
  ctx.fillText('RED', CANVAS_W/2 - 78 - slide, CANVAS_H/2);

  ctx.shadowColor = 'rgba(255,255,255,0.4)';
  ctx.shadowBlur = 6;
  ctx.fillStyle = '#f0f0f4';
  ctx.font = '700 18px Courier New';
  ctx.fillText('VS', CANVAS_W/2, CANVAS_H/2);

  ctx.shadowColor = '#9aa0ff';
  ctx.shadowBlur = 16;
  ctx.fillStyle = '#3a3d42';
  ctx.font = '800 30px Courier New';
  ctx.fillText('BLACK', CANVAS_W/2 + 90 + slide, CANVAS_H/2);
  ctx.restore();
}

function render(game){
  ctx.clearRect(0,0,CANVAS_W,CANVAS_H);
  ctx.fillStyle = '#0a0a0c';
  ctx.fillRect(0,0,CANVAS_W,CANVAS_H);

  drawBgDust(game);
  drawMaze(game);
  for (const b of game.bullets) drawBullet(b);
  drawParticles(game.particles);
  for (const t of game.tanks) drawTank(t);
  drawHUD(game);
  drawIntroSplash(game);
}

/* =========================================================================
   BOOT
   ========================================================================= */
new Game();

})();
