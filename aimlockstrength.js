// === Core Math Utilities ===
class Vector3 {
  constructor(x = 0, y = 0, z = 0) { 
    this.x = x; this.y = y; this.z = z; 
  }
  
  set(x, y, z) { 
    this.x = x; this.y = y; this.z = z; 
    return this; 
  }
  
  addMut(v) { 
    this.x += v.x; this.y += v.y; this.z += v.z; 
    return this; 
  }
  
  subtractMut(v) { 
    this.x -= v.x; this.y -= v.y; this.z -= v.z; 
    return this; 
  }
  
  multiplyScalarMut(s) { 
    this.x *= s; this.y *= s; this.z *= s; 
    return this; 
  }
  
  add(v) { 
    return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z); 
  }
  
  subtract(v) { 
    return new Vector3(this.x - v.x, this.y - v.y, this.z - v.z); 
  }
  
  multiplyScalar(s) { 
    return new Vector3(this.x * s, this.y * s, this.z * s); 
  }
  
  length() { 
    return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z); 
  }
  
  lengthSq() { 
    return this.x * this.x + this.y * this.y + this.z * this.z; 
  }
  
  normalize() {
    const len = this.length();
    if (len > 0) {
      this.multiplyScalarMut(1 / len);
    }
    return this;
  }
  
  dot(v) {
    return this.x * v.x + this.y * v.y + this.z * v.z;
  }
  
  cross(v) {
    return new Vector3(
      this.y * v.z - this.z * v.y,
      this.z * v.x - this.x * v.z,
      this.x * v.y - this.y * v.x
    );
  }
  
  clone() { 
    return new Vector3(this.x, this.y, this.z); 
  }
  
  lerp(v, t) {
    const inv = 1 - t;
    this.x = this.x * inv + v.x * t;
    this.y = this.y * inv + v.y * t;
    this.z = this.z * inv + v.z * t;
    return this;
  }
  
  distanceTo(v) {
    return this.subtract(v).length();
  }
  
  static zero() { return new Vector3(0, 0, 0); }
  static one() { return new Vector3(1, 1, 1); }
  static forward() { return new Vector3(0, 0, 1); }
  static up() { return new Vector3(0, 1, 0); }
}

// === Advanced Kalman Filter ===
class AdaptiveKalmanFilter {
  constructor(R = 0.01, Q = 0.0001) {
    this.R = R; // Measurement noise
    this.Q = Q; // Process noise
    this.A = 1; // State transition
    this.C = 1; // Observation model
    this.x = NaN; // State estimate
    this.cov = NaN; // Error covariance
    this.innovation = 0; // Innovation sequence
    this.adaptationRate = 0.1;
    this.minR = 0.001;
    this.maxR = 0.1;
  }
  
  filter(z) {
    if (isNaN(this.x)) {
      this.x = z;
      this.cov = this.R;
      return this.x;
    }
    
    // Prediction step
    const predX = this.x;
    const predCov = this.cov + this.Q;
    
    // Update step
    const K = predCov / (predCov + this.R);
    this.innovation = z - predX;
    this.x = predX + K * this.innovation;
    this.cov = predCov * (1 - K);
    
    // Adaptive noise adjustment
    this.adaptNoise();
    
    return this.x;
  }
  
  adaptNoise() {
    const absInnovation = Math.abs(this.innovation);
    if (absInnovation > 0.1) {
      this.R = Math.min(this.R * (1 + this.adaptationRate), this.maxR);
    } else {
      this.R = Math.max(this.R * (1 - this.adaptationRate), this.minR);
    }
  }
  
  reset() { 
    this.x = NaN; 
    this.cov = NaN; 
    this.innovation = 0;
  }
}

// === Target Priority System ===
class TargetPrioritySystem {
  constructor() {
    this.priorities = new Map();
    this.lastUpdate = Date.now();
  }
  
  calculatePriority(targetId, position, health = 200, threat = 1, distance = 9999) {
    const now = Date.now();
    const timeFactor = Math.max(0, 1 - (now - this.lastUpdate) / 5000); // Decay over 5s
    
    // Priority calculation based on multiple factors
    const healthWeight = (100 - health) / 100; // Lower health = higher priority
    const distanceWeight = Math.max(0, 1 - distance / 50); // Closer = higher priority
    const threatWeight = Math.min(threat, 3) / 3; // Threat level 0-3
    
    const priority = (healthWeight * 0.4 + distanceWeight * 0.3 + threatWeight * 0.3) * timeFactor;
    
    this.priorities.set(targetId, {
      priority,
      position: position.clone(),
      lastSeen: now,
      health,
      threat,
      distance
    });
    
    return priority;
  }
  
  getBestTarget() {
    let bestTarget = null;
    let highestPriority = -1;
    
    for (const [id, data] of this.priorities.entries()) {
      if (data.priority > highestPriority) {
        highestPriority = data.priority;
        bestTarget = { id, ...data };
      }
    }
    
    return bestTarget;
  }
  
  removeTarget(targetId) {
    this.priorities.delete(targetId);
  }
  
  cleanup(maxAge = 10000) {
    const now = Date.now();
    for (const [id, data] of this.priorities.entries()) {
      if (now - data.lastSeen > maxAge) {
        this.priorities.delete(id);
      }
    }
  }
}

// === Ballistics Calculator ===
class BallisticsCalculator {
  constructor() {
    this.gravity = -9.81;
    this.airResistance = 0.1;
    this.windSpeed = new Vector3(0, 0, 0);
  }
  
  calculateBulletDrop(distance, velocity, angle = 0) {
    const timeOfFlight = distance / (velocity * Math.cos(angle));
    const drop = 0.5 * Math.abs(this.gravity) * timeOfFlight * timeOfFlight;
    return drop;
  }
  
  calculateLeadTarget(targetPos, targetVel, bulletSpeed, shooterPos) {
    const relativePos = targetPos.subtract(shooterPos);
    const distance = relativePos.length();
    const timeToTarget = distance / bulletSpeed;
    
    // Predict target position
    const predictedPos = targetPos.add(targetVel.multiplyScalar(timeToTarget));
    
    // Account for bullet drop
    const drop = this.calculateBulletDrop(distance, bulletSpeed);
    predictedPos.y += drop;
    
    return predictedPos;
  }
  
  setEnvironmentalFactors(gravity, airResistance, windSpeed) {
    this.gravity = gravity;
    this.airResistance = airResistance;
    this.windSpeed = windSpeed.clone();
  }
}

// === Performance Monitor ===
class PerformanceMonitor {
  constructor(maxSamples = 60) {
    this.frameTimes = [];
    this.maxSamples = maxSamples;
    this.lastFrameTime = performance.now();
    this.totalFrames = 0;
    this.droppedFrames = 0;
    this.targetFPS = 120;
  }
  
  update() {
    const now = performance.now();
    const frameTime = now - this.lastFrameTime;
    this.lastFrameTime = now;
    
    this.frameTimes.push(frameTime);
    if (this.frameTimes.length > this.maxSamples) {
      this.frameTimes.shift();
    }
    
    this.totalFrames++;
    if (frameTime > (1000 / this.targetFPS) * 1.5) {
      this.droppedFrames++;
    }
  }
  
  getAverageFrameTime() {
    if (this.frameTimes.length === 0) return 16.67;
    return this.frameTimes.reduce((a, b) => a + b, 0) / this.frameTimes.length;
  }
  
  getFPS() {
    return 1000 / this.getAverageFrameTime();
  }
  
  getStats() {
    return {
      avgFrameTime: this.getAverageFrameTime().toFixed(2),
      fps: this.getFPS().toFixed(1),
      droppedFrames: this.droppedFrames,
      totalFrames: this.totalFrames,
      frameDropRate: ((this.droppedFrames / this.totalFrames) * 100).toFixed(1)
    };
  }
}

// === Enhanced Aim Lock Tracker ===
class EnhancedAimLockTracker {
  constructor(config = {}) {
    // Filters
    this.kalmanX = new AdaptiveKalmanFilter();
    this.kalmanY = new AdaptiveKalmanFilter();
    this.kalmanZ = new AdaptiveKalmanFilter();
    
    // Position tracking
    this.lastAimPos = new Vector3();
    this.filteredPos = new Vector3();
    this.predictedPos = new Vector3();
    
    // Velocity tracking
    this.headVelocity = new Vector3();
    this.velocityHistory = [];
    this.maxVelocityHistory = 5;
    this.lastHeadPos = new Vector3();
    
    // Matrix operations
    this.worldMatrix = new Float32Array(16);
    this.bindMatrix = new Float32Array(16);
    this.tempPos = new Vector3();
    
    // Timing and state
    this.lastRedTime = 0;
    this.holdDuration = config.holdDuration || 8000;
    this.recoil = Vector3.zero();
    this.frameCount = 0;
    this.lastFrameTime = performance.now();
    
    // Performance
    this.performanceMonitor = new PerformanceMonitor();
    this.adaptiveInterval = 16;
    this.minInterval = 8;
    this.maxInterval = 33;
    
    // Ballistics
    this.ballistics = new BallisticsCalculator();
    this.bulletSpeed = config.bulletSpeed || 800;
    
    // Smoothing and accuracy
    this.smoothingFactor = config.smoothingFactor || 0.6;
    this.aimPrecision = config.aimPrecision || 0.0;
    this.maxAimDistance = config.maxAimDistance || 9999;
    
    // Binding
    this.boundLoop = this.loop.bind(this);
    this.isRunning = false;
    this.isPaused = false;
    
    // Target data
    this.targetId = null;
    this.boneData = null;
    this.shooterPosition = new Vector3();
  }
  
  setConfig(config) {
    Object.assign(this, config);
  }
  
  setRecoil(x, y, z) {
    this.recoil.set(x, y, z);
  }
  
  setShooterPosition(pos) {
    this.shooterPosition.set(pos.x, pos.y, pos.z);
  }
  
  isCrosshairRed() {
    return typeof GameAPI !== "undefined" && GameAPI.crosshairState === "red";
  }
  
  isTargetVisible() {
    return typeof GameAPI !== "undefined" && GameAPI.isTargetVisible && GameAPI.isTargetVisible(this.targetId);
  }
  
  applyAdvancedFiltering(pos) {
    // Apply Kalman filtering
    const filtered = new Vector3(
      this.kalmanX.filter(pos.x),
      this.kalmanY.filter(pos.y),
      this.kalmanZ.filter(pos.z)
    );
    
    // Apply additional smoothing based on movement speed
    const movementSpeed = this.headVelocity.length();
    const dynamicSmoothing = Math.min(0.9, this.smoothingFactor + movementSpeed * 0.1);
    
    this.filteredPos.lerp(filtered, 1 - dynamicSmoothing);
    return this.filteredPos.clone();
  }
  
  applyBallisticsCompensation(targetPos) {
    const distance = targetPos.distanceTo(this.shooterPosition);
    if (distance > this.maxAimDistance) return targetPos;
    
    // Calculate bullet drop and lead
    const compensated = this.ballistics.calculateLeadTarget(
      targetPos,
      this.headVelocity,
      this.bulletSpeed,
      this.shooterPosition
    );
    
    return compensated;
  }
  
  applyRecoilCompensation(vec) {
    const recoilFactor = Math.min(1, this.frameCount / 100); // Gradual recoil buildup
    const adjustedRecoil = this.recoil.multiplyScalar(recoilFactor);
    return vec.subtract(adjustedRecoil);
  }
  
  predictTargetMovement(currentPos, dt) {
    // Update velocity with smoothing
    if (this.frameCount > 1) {
      const instantVel = new Vector3(
        (currentPos.x - this.lastHeadPos.x) / dt,
        (currentPos.y - this.lastHeadPos.y) / dt,
        (currentPos.z - this.lastHeadPos.z) / dt
      );
      
      // Store velocity history for better prediction
      this.velocityHistory.push(instantVel);
      if (this.velocityHistory.length > this.maxVelocityHistory) {
        this.velocityHistory.shift();
      }
      
      // Calculate average velocity
      const avgVel = new Vector3();
      this.velocityHistory.forEach(v => avgVel.addMut(v));
      avgVel.multiplyScalarMut(1 / this.velocityHistory.length);
      
      this.headVelocity.lerp(avgVel, 0.3);
    }
    
    this.lastHeadPos.set(currentPos.x, currentPos.y, currentPos.z);
    
    // Predict future position with acceleration consideration
    const acceleration = this.velocityHistory.length > 1 ? 
      this.velocityHistory[this.velocityHistory.length - 1].subtract(
        this.velocityHistory[this.velocityHistory.length - 2]
      ).multiplyScalar(1 / dt) : Vector3.zero();
    
    const predictionTime = dt * 2; // Predict 2 frames ahead
    const predicted = currentPos.clone()
      .addMut(this.headVelocity.multiplyScalar(predictionTime))
      .addMut(acceleration.multiplyScalar(0.5 * predictionTime * predictionTime));
    
    return predicted;
  }
  
  adaptiveSmoothing(target) {
    const distance = this.lastAimPos.distanceTo(target);
    const speed = this.headVelocity.length();
    
    // Reduce smoothing for fast-moving or distant targets
    let adaptiveFactor = this.smoothingFactor;
    if (distance > 5) adaptiveFactor *= 0.8;
    if (speed > 10) adaptiveFactor *= 0.7;
    
    return this.lastAimPos.lerp(target, 1 - adaptiveFactor);
  }
  
  setAim(vec3) {
    // Add some randomization to avoid detection
    const jitter = new Vector3(
      (Math.random() - 0.5) * 0.001,
      (Math.random() - 0.5) * 0.001,
      (Math.random() - 0.5) * 0.001
    );
    
    const finalAim = vec3.add(jitter);
    
    if (this.frameCount % 30 === 0) {
      console.log("🎯 Enhanced aim lock:", finalAim.x.toFixed(4), finalAim.y.toFixed(4), finalAim.z.toFixed(4));
    }
    
    if (typeof GameAPI !== "undefined" && GameAPI.setCrosshairTarget) {
      GameAPI.setCrosshairTarget(finalAim.x, finalAim.y, finalAim.z);
    }
  }
  
  computeWorldHead(boneData) {
    const { position, rotation, scale, bindpose } = boneData;
    
    // Build world matrix
    quaternionToMatrix(rotation, this.worldMatrix);
    
    // Apply scale
    this.worldMatrix[0] *= scale.x; this.worldMatrix[1] *= scale.y; this.worldMatrix[2] *= scale.z;
    this.worldMatrix[4] *= scale.x; this.worldMatrix[5] *= scale.y; this.worldMatrix[6] *= scale.z;
    this.worldMatrix[8] *= scale.x; this.worldMatrix[9] *= scale.y; this.worldMatrix[10] *= scale.z;
    
    // Apply translation
    this.worldMatrix[3] = position.x;
    this.worldMatrix[7] = position.y;
    this.worldMatrix[11] = position.z;
    
    // Set bind matrix
    const b = this.bindMatrix;
    b[0] = bindpose.e00; b[1] = bindpose.e01; b[2] = bindpose.e02; b[3] = bindpose.e03;
    b[4] = bindpose.e10; b[5] = bindpose.e11; b[6] = bindpose.e12; b[7] = bindpose.e13;
    b[8] = bindpose.e20; b[9] = bindpose.e21; b[10] = bindpose.e22; b[11] = bindpose.e23;
    
    // Get world position
    this.tempPos.set(this.worldMatrix[3], this.worldMatrix[7], this.worldMatrix[11]);
    
    // Transform by bind matrix
    return multiplyMatrixVec(this.bindMatrix, this.tempPos);
  }
  
  lockBoneHead(worldHead, dt) {
    const now = performance.now();
    
    // Check if target is valid and visible
    if (!this.isTargetVisible()) {
      return;
    }
    
    // Enhanced crosshair detection
    if (this.isCrosshairRed()) {
      console.log("💥 Enhanced headshot detection!");
      this.lastRedTime = now;
    }
    
    // Extended tracking after red crosshair
    if (now - this.lastRedTime <= this.holdDuration) {
      // Multi-stage processing pipeline
      const predicted = this.predictTargetMovement(worldHead, dt);
      const ballistics = this.applyBallisticsCompensation(predicted);
      const compensated = this.applyRecoilCompensation(ballistics);
      const filtered = this.applyAdvancedFiltering(compensated);
      const smooth = this.adaptiveSmoothing(filtered);
      
      this.setAim(smooth);
    }
  }
  
  loop() {
    if (!this.isRunning || this.isPaused) {
      if (this.isRunning) {
        setTimeout(this.boundLoop, this.adaptiveInterval);
      }
      return;
    }
    
    const now = performance.now();
    const dt = (now - this.lastFrameTime) / 1000;
    this.lastFrameTime = now;
    
    // Update performance monitoring
    this.performanceMonitor.update();
    
    // Adaptive frame rate based on performance
    const stats = this.performanceMonitor.getStats();
    if (parseFloat(stats.frameDropRate) > 10) {
      this.adaptiveInterval = Math.min(this.adaptiveInterval + 2, this.maxInterval);
    } else if (parseFloat(stats.frameDropRate) < 5) {
      this.adaptiveInterval = Math.max(this.adaptiveInterval - 1, this.minInterval);
    }
    
    try {
      const headWorld = this.computeWorldHead(this.boneData);
      this.lockBoneHead(headWorld, dt);
    } catch (error) {
      console.error("🚨 Tracking error:", error);
    }
    
    this.frameCount++;
    setTimeout(this.boundLoop, this.adaptiveInterval);
  }
  
  start(boneData, targetId = null) {
    if (this.isRunning) return;
    
    this.boneData = boneData;
    this.targetId = targetId;
    this.isRunning = true;
    this.isPaused = false;
    this.lastFrameTime = performance.now();
    this.frameCount = 0;
    
    // Reset filters
    this.kalmanX.reset();
    this.kalmanY.reset();
    this.kalmanZ.reset();
    
    console.log("🚀 Enhanced AimLock started for target:", targetId);
    this.loop();
  }
  
  pause() {
    this.isPaused = true;
    console.log("⏸️ AimLock paused");
  }
  
  resume() {
    this.isPaused = false;
    this.lastFrameTime = performance.now();
    console.log("▶️ AimLock resumed");
  }
  
  stop() {
    this.isRunning = false;
    this.isPaused = false;
    console.log("⏹️ Enhanced AimLock stopped");
  }
  
  getStats() {
    const perfStats = this.performanceMonitor.getStats();
    return {
      ...perfStats,
      frameCount: this.frameCount,
      isRunning: this.isRunning,
      isPaused: this.isPaused,
      targetId: this.targetId,
      adaptiveInterval: this.adaptiveInterval,
      headVelocity: this.headVelocity.length().toFixed(2),
      smoothingFactor: this.smoothingFactor
    };
  }
}

// === Multi-Target Manager with Priority System ===
class IntelligentMultiTargetManager {
  constructor(config = {}) {
    this.trackers = new Map();
    this.prioritySystem = new TargetPrioritySystem();
    this.activeTarget = null;
    this.maxTargets = config.maxTargets || 12;
    this.targetSwitchCooldown = config.targetSwitchCooldown || 0;
    this.lastTargetSwitch = 0;
    this.globalRecoil = Vector3.zero();
    this.config = config;
  }
  
  addTarget(targetId, boneData, priority = 0) {
    if (this.trackers.size >= this.maxTargets) {
      console.warn("Max targets reached, removing lowest priority target");
      this.removeLowPriorityTarget();
    }
    
    const tracker = new EnhancedAimLockTracker(this.config);
    tracker.setRecoil(this.globalRecoil.x, this.globalRecoil.y, this.globalRecoil.z);
    
    this.trackers.set(targetId, tracker);
    this.prioritySystem.calculatePriority(targetId, new Vector3(), 100, priority, 10);
    
    console.log(`➕ Added target ${targetId}`);
    
    // Auto-start if no active target
    if (!this.activeTarget) {
      this.switchToTarget(targetId, boneData);
    }
  }
  
  removeTarget(targetId) {
    const tracker = this.trackers.get(targetId);
    if (tracker) {
      tracker.stop();
      this.trackers.delete(targetId);
      this.prioritySystem.removeTarget(targetId);
      
      console.log(`➖ Removed target ${targetId}`);
      
      // Switch to next best target if this was active
      if (this.activeTarget === targetId) {
        this.activeTarget = null;
        this.switchToBestTarget();
      }
    }
  }
  
  removeLowPriorityTarget() {
    let lowestPriority = Infinity;
    let targetToRemove = null;
    
    for (const [id] of this.trackers) {
      const priority = this.prioritySystem.priorities.get(id)?.priority || 0;
      if (priority < lowestPriority) {
        lowestPriority = priority;
        targetToRemove = id;
      }
    }
    
    if (targetToRemove) {
      this.removeTarget(targetToRemove);
    }
  }
  
  updateTargetPriority(targetId, health, threat, distance, position) {
    this.prioritySystem.calculatePriority(targetId, position, health, threat, distance);
  }
  
  switchToTarget(targetId, boneData) {
    const now = Date.now();
    if (now - this.lastTargetSwitch < this.targetSwitchCooldown) {
      return false;
    }
    
    // Stop current active target
    if (this.activeTarget) {
      const currentTracker = this.trackers.get(this.activeTarget);
      if (currentTracker) {
        currentTracker.pause();
      }
    }
    
    // Start new target
    const newTracker = this.trackers.get(targetId);
    if (newTracker) {
      if (this.activeTarget === targetId) {
        newTracker.resume();
      } else {
        newTracker.start(boneData, targetId);
      }
      
      this.activeTarget = targetId;
      this.lastTargetSwitch = now;
      
      console.log(`🔄 Switched to target ${targetId}`);
      return true;
    }
    
    return false;
  }
  
  switchToBestTarget() {
    const bestTarget = this.prioritySystem.getBestTarget();
    if (bestTarget && this.trackers.has(bestTarget.id)) {
      // Need bone data - this would come from game API
      const boneData = this.getTargetBoneData(bestTarget.id);
      if (boneData) {
        this.switchToTarget(bestTarget.id, boneData);
      }
    }
  }
  
  // Placeholder - would integrate with game API
  getTargetBoneData(targetId) {
    // This would fetch real bone data from the game
    return null;
  }
  
  updateGlobalRecoil(recoilVec) {
    this.globalRecoil.set(recoilVec.x, recoilVec.y, recoilVec.z);
    
    // Update all trackers
    for (const tracker of this.trackers.values()) {
      tracker.setRecoil(recoilVec.x, recoilVec.y, recoilVec.z);
    }
  }
  
  pauseAll() {
    for (const tracker of this.trackers.values()) {
      tracker.pause();
    }
    console.log("⏸️ All trackers paused");
  }
  
  resumeAll() {
    for (const tracker of this.trackers.values()) {
      tracker.resume();
    }
    console.log("▶️ All trackers resumed");
  }
  
  stopAll() {
    for (const tracker of this.trackers.values()) {
      tracker.stop();
    }
    this.trackers.clear();
    this.activeTarget = null;
    console.log("⏹️ All trackers stopped");
  }
  
  cleanup() {
    this.prioritySystem.cleanup();
  }
  
  getSystemStats() {
    const activeTracker = this.activeTarget ? this.trackers.get(this.activeTarget) : null;
    const activeStats = activeTracker ? activeTracker.getStats() : null;
    
    return {
      totalTargets: this.trackers.size,
      activeTarget: this.activeTarget,
      activeStats,
      priorities: Array.from(this.prioritySystem.priorities.entries()),
      lastTargetSwitch: this.lastTargetSwitch
    };
  }
}

// === Matrix Utilities ===
const matrixCache = new Float32Array(16);
const tempVec = new Vector3();

function quaternionToMatrix(q, out = matrixCache) {
  const { x, y, z, w } = q;
  const x2 = x + x, y2 = y + y, z2 = z + z;
  const xx = x * x2, xy = x * y2, xz = x * z2;
  const yy = y * y2, yz = y * z2, zz = z * z2;
  const wx = w * x2, wy = w * y2, wz = w * z2;
  
  out[0] = 1 - (yy + zz); out[1] = xy - wz; out[2] = xz + wy; out[3] = 0;
  out[4] = xy + wz; out[5] = 1 - (xx + zz); out[6] = yz - wx; out[7] = 0;
  out[8] = xz - wy; out[9] = yz + wx; out[10] = 1 - (xx + yy); out[11] = 0;
  out[12] = out[13] = out[14] = 0; out[15] = 1;
  return out;
}

function multiplyMatrixVec(m, v, out = tempVec) {
  const x = m[0]*v.x + m[1]*v.y + m[2]*v.z + m[3];
  const y = m[4]*v.x + m[5]*v.y + m[6]*v.z + m[7];
  const z = m[8]*v.x + m[9]*v.y + m[10]*v.z + m[11];
  return out.set(x, y, z);
}

// === Configuration Presets ===
const AimLockPresets = {
  AGGRESSIVE: {
    holdDuration: 1200,
    smoothingFactor: 0.6,
    aimPrecision: 0.0,
    bulletSpeed: 1000,
    maxAimDistance: 99999
  },
  
  BALANCED: {
    holdDuration: 800,
    smoothingFactor: 0.75,
    aimPrecision: 0.0,
    bulletSpeed: 800,
    maxAimDistance: 9999
  },
  
  SUBTLE: {
    holdDuration: 600,
    smoothingFactor: 0.85,
    aimPrecision: 0.0,
    bulletSpeed: 600,
    maxAimDistance: 9999
  },
  
  SNIPER: {
    holdDuration: 2000,
    smoothingFactor: 0.9,
    aimPrecision: 0.0,
    bulletSpeed: 1200,
    maxAimDistance: 9999
  }
};

// === Anti-Detection System ===
class AntiDetectionSystem {
  constructor() {
    this.humanizationPatterns = [];
    this.lastPattern = 0;
    this.patternCooldown = 2000;
    this.detectionRisk = 0;
    this.maxRisk = 0.8;
    this.riskDecayRate = 0.01;
  }
  
  addHumanization(aimVector) {
    // Add subtle human-like imperfections
    const time = Date.now();
    const micro_tremor = this.getMicroTremor(time);
    const fatigue_factor = this.getFatigueFactor();
    const reaction_delay = this.getReactionDelay();
    
    // Apply humanization
    aimVector.x += micro_tremor.x * fatigue_factor;
    aimVector.y += micro_tremor.y * fatigue_factor;
    aimVector.z += micro_tremor.z * fatigue_factor;
    
    return aimVector;
  }
  
  getMicroTremor(time) {
    // Simulate natural hand tremor
    const freq1 = 0.1, freq2 = 0.05, freq3 = 0.02;
    const amp = 0.0005;
    
    return new Vector3(
      Math.sin(time * freq1) * amp + Math.sin(time * freq2) * amp * 0.5,
      Math.cos(time * freq1) * amp + Math.cos(time * freq3) * amp * 0.3,
      (Math.sin(time * freq2) + Math.cos(time * freq3)) * amp * 0.2
    );
  }
  
  getFatigueFactor() {
    // Simulate increasing fatigue over time
    const sessionTime = Date.now() / 60000; // minutes
    return 1 + Math.min(sessionTime * 0.1, 2); // Max 3x fatigue
  }
  
  getReactionDelay() {
    // Human reaction time variation (100-300ms)
    return 100 + Math.random() * 200;
  }
  
  updateDetectionRisk(perfect_shots, total_shots) {
    const accuracy = perfect_shots / Math.max(total_shots, 1);
    if (accuracy > 0.95) {
      this.detectionRisk += 0.1;
    } else if (accuracy < 0.7) {
      this.detectionRisk -= 0.05;
    }
    
    // Natural decay
    this.detectionRisk -= this.riskDecayRate;
    this.detectionRisk = Math.max(0, Math.min(1, this.detectionRisk));
  }
  
  shouldReduceAccuracy() {
    return this.detectionRisk > this.maxRisk;
  }
  
  getRecommendedMissProbability() {
    if (this.detectionRisk > 0.9) return 0.3;
    if (this.detectionRisk > 0.7) return 0.15;
    if (this.detectionRisk > 0.5) return 0.05;
    return 0;
  }
}

// === Statistics and Analytics ===
class AimLockAnalytics {
  constructor() {
    this.stats = {
      shotsTotal: 0,
      shotsHit: 0,
      headshots: 0,
      avgAccuracy: 0,
      sessionStartTime: Date.now(),
      targetSwitches: 0,
      avgTrackingTime: 0,
      performanceMetrics: []
    };
    
    this.sessionData = [];
    this.maxSessionData = 1000;
  }
  
  recordShot(hit, headshot = false) {
    this.stats.shotsTotal++;
    if (hit) {
      this.stats.shotsHit++;
      if (headshot) this.stats.headshots++;
    }
    
    this.stats.avgAccuracy = this.stats.shotsHit / this.stats.shotsTotal;
    
    this.sessionData.push({
      timestamp: Date.now(),
      hit,
      headshot,
      accuracy: this.stats.avgAccuracy
    });
    
    if (this.sessionData.length > this.maxSessionData) {
      this.sessionData.shift();
    }
  }
  
  recordTargetSwitch() {
    this.stats.targetSwitches++;
  }
  
  recordPerformanceMetric(metric) {
    this.stats.performanceMetrics.push({
      timestamp: Date.now(),
      ...metric
    });
    
    // Keep only last 100 metrics
    if (this.stats.performanceMetrics.length > 100) {
      this.stats.performanceMetrics.shift();
    }
  }
  
  getSessionSummary() {
    const sessionTime = (Date.now() - this.stats.sessionStartTime) / 60000; // minutes
    const headshotRate = this.stats.headshots / Math.max(this.stats.shotsHit, 1);
    
    return {
      ...this.stats,
      sessionTimeMinutes: sessionTime.toFixed(1),
      headshotRate: (headshotRate * 100).toFixed(1) + '%',
      accuracy: (this.stats.avgAccuracy * 100).toFixed(1) + '%',
      shotsPerMinute: (this.stats.shotsTotal / Math.max(sessionTime, 0.1)).toFixed(1)
    };
  }
  
  exportData() {
    return {
      stats: this.stats,
      sessionData: this.sessionData,
      exportTime: Date.now()
    };
  }
  
  reset() {
    this.stats = {
      shotsTotal: 0,
      shotsHit: 0,
      headshots: 0,
      avgAccuracy: 0,
      sessionStartTime: Date.now(),
      targetSwitches: 0,
      avgTrackingTime: 0,
      performanceMetrics: []
    };
    this.sessionData = [];
  }
}

// === Master Control System ===
class MasterAimLockSystem {
  constructor(preset = 'BALANCED') {
    this.config = { ...AimLockPresets[preset] };
    this.multiTargetManager = new IntelligentMultiTargetManager(this.config);
    this.antiDetection = new AntiDetectionSystem();
    this.analytics = new AimLockAnalytics();
    this.isActive = false;
    this.currentPreset = preset;
    
    // Bind methods
    this.boundGameEventHandler = this.handleGameEvent.bind(this);
    
    console.log(`🎮 Master AimLock System initialized with ${preset} preset`);
  }
  
  // Configuration Management
  setPreset(presetName) {
    if (AimLockPresets[presetName]) {
      this.config = { ...AimLockPresets[presetName] };
      this.currentPreset = presetName;
      this.multiTargetManager.config = this.config;
      console.log(`🔧 Switched to ${presetName} preset`);
      return true;
    }
    return false;
  }
  
  updateConfig(newConfig) {
    Object.assign(this.config, newConfig);
    this.multiTargetManager.config = this.config;
  }
  
  // Target Management
  addTarget(targetId, boneData, metadata = {}) {
    const priority = metadata.threat || 1;
    this.multiTargetManager.addTarget(targetId, boneData, priority);
    
    if (metadata.health !== undefined) {
      this.multiTargetManager.updateTargetPriority(
        targetId, 
        metadata.health, 
        metadata.threat || 1, 
        metadata.distance || 10,
        new Vector3(boneData.position.x, boneData.position.y, boneData.position.z)
      );
    }
  }
  
  removeTarget(targetId) {
    this.multiTargetManager.removeTarget(targetId);
  }
  
  updateTargetData(targetId, boneData, metadata = {}) {
    if (metadata.health !== undefined) {
      this.multiTargetManager.updateTargetPriority(
        targetId,
        metadata.health,
        metadata.threat || 1,
        metadata.distance || 10,
        new Vector3(boneData.position.x, boneData.position.y, boneData.position.z)
      );
    }
    
    // Update active tracker if this is the current target
    if (this.multiTargetManager.activeTarget === targetId) {
      const tracker = this.multiTargetManager.trackers.get(targetId);
      if (tracker) {
        tracker.boneData = boneData;
      }
    }
  }
  
  // Game Event Handling
  handleGameEvent(event) {
    switch (event.type) {
      case 'shot_fired':
        this.onShotFired(event);
        break;
      case 'target_hit':
        this.onTargetHit(event);
        break;
      case 'target_killed':
        this.onTargetKilled(event);
        break;
      case 'crosshair_red':
        this.onCrosshairRed(event);
        break;
      case 'recoil_update':
        this.onRecoilUpdate(event);
        break;
    }
  }
  
  onShotFired(event) {
    this.analytics.recordShot(false); // Will be updated on hit
    
    // Update anti-detection system
    this.antiDetection.updateDetectionRisk(
      this.analytics.stats.shotsHit,
      this.analytics.stats.shotsTotal
    );
  }
  
  onTargetHit(event) {
    this.analytics.recordShot(true, event.headshot);
    
    // Reward priority for accurate targets
    if (event.targetId && event.headshot) {
      const currentPriority = this.multiTargetManager.prioritySystem.priorities.get(event.targetId);
      if (currentPriority) {
        currentPriority.priority += 0.1;
      }
    }
  }
  
  onTargetKilled(event) {
    if (event.targetId) {
      this.removeTarget(event.targetId);
    }
  }
  
  onCrosshairRed(event) {
    // Enhanced crosshair detection handling
    console.log("🔴 Master system detected red crosshair");
  }
  
  onRecoilUpdate(event) {
    const recoilVec = new Vector3(event.recoil.x, event.recoil.y, event.recoil.z);
    this.multiTargetManager.updateGlobalRecoil(recoilVec);
  }
  
  // System Control
  activate() {
    if (this.isActive) return;
    
    this.isActive = true;
    this.analytics.reset();
    
    // Setup game event listeners if available
    if (typeof GameAPI !== "undefined" && GameAPI.addEventListener) {
      GameAPI.addEventListener('game_event', this.boundGameEventHandler);
    }
    
    console.log("🟢 Master AimLock System ACTIVATED");
  }
  
  deactivate() {
    if (!this.isActive) return;
    
    this.isActive = false;
    this.multiTargetManager.stopAll();
    
    // Remove game event listeners
    if (typeof GameAPI !== "undefined" && GameAPI.removeEventListener) {
      GameAPI.removeEventListener('game_event', this.boundGameEventHandler);
    }
    
    console.log("🔴 Master AimLock System DEACTIVATED");
  }
  
  pause() {
    this.multiTargetManager.pauseAll();
    console.log("⏸️ Master system paused");
  }
  
  resume() {
    this.multiTargetManager.resumeAll();
    console.log("▶️ Master system resumed");
  }
  
  // Emergency stop with cleanup
  emergencyStop() {
    this.deactivate();
    this.multiTargetManager.stopAll();
    this.analytics.reset();
    console.log("🚨 EMERGENCY STOP - All systems disabled");
  }
  
  // Diagnostics and Monitoring
  getSystemStatus() {
    const managerStats = this.multiTargetManager.getSystemStats();
    const analytics = this.analytics.getSessionSummary();
    const antiDetection = {
      detectionRisk: (this.antiDetection.detectionRisk * 100).toFixed(1) + '%',
      shouldReduceAccuracy: this.antiDetection.shouldReduceAccuracy(),
      recommendedMissProbability: this.antiDetection.getRecommendedMissProbability()
    };
    
    return {
      isActive: this.isActive,
      currentPreset: this.currentPreset,
      config: this.config,
      targetManager: managerStats,
      analytics,
      antiDetection,
      timestamp: Date.now()
    };
  }
  
  // Performance optimization
  optimize() {
    this.multiTargetManager.cleanup();
    
    // Adjust settings based on performance
    const systemStats = this.getSystemStatus();
    const activeStats = systemStats.targetManager.activeStats;
    
    if (activeStats && parseFloat(activeStats.frameDropRate) > 10) {
      console.log("🔧 Performance optimization: Reducing update frequency");
      this.config.updateInterval = Math.min(this.config.updateInterval + 2, 50);
    }
  }
  
  // Data export for analysis
  exportSessionData() {
    return {
      systemStatus: this.getSystemStatus(),
      analyticsData: this.analytics.exportData(),
      config: this.config,
      exportTime: new Date().toISOString()
    };
  }
}

// === Usage Examples and Integration ===

// Example: Initialize system
const masterSystem = new MasterAimLockSystem('BALANCED');

// Example: Add targets with sample data
const sampleEnemyData = {
  position: { x: -0.0456970781, y: -0.004478302, z: -0.0200432576 },
  rotation: { x: 0.0258174837, y: -0.08611039, z: -0.1402113, w: 0.9860321 },
  scale: { x: 1, y: 1, z: 1 },
  bindpose: {
    e00: -1.34559613e-13, e01: 8.881784e-14, e02: -1.0, e03: 0.487912,
    e10: -2.84512817e-6, e11: -1.0, e12: 8.881784e-14, e13: -2.842171e-14,
    e20: -1.0, e21: 2.84512817e-6, e22: -1.72951931e-13, e23: 0.0,
    e30: 0.0, e31: 0.0, e32: 0.0, e33: 1.0
  }
};

// Example usage:
/*
// Activate the system
masterSystem.activate();

// Add targets
masterSystem.addTarget('enemy1', sampleEnemyData, { 
  health: 200, 
  threat: 2, 
  distance: 99999 
});

masterSystem.addTarget('enemy2', sampleEnemyData, { 
  health: 500, 
  threat: 1, 
  distance: 99999
});

// Switch presets dynamically
masterSystem.setPreset('AGGRESSIVE');

// Handle game events
masterSystem.handleGameEvent({
  type: 'recoil_update',
  recoil: { x: 0.002, y: -0.001, z: 0.001 }
});

// Get system status
console.log('System Status:', masterSystem.getSystemStatus());

// Pause/Resume
masterSystem.pause();
masterSystem.resume();

// Export data for analysis
const sessionData = masterSystem.exportSessionData();
console.log('Session Data:', sessionData);

// Deactivate when done
masterSystem.deactivate();
*/

console.log("🎯 Enhanced AimLock System loaded successfully!");
console.log("Available presets:", Object.keys(AimLockPresets));
console.log("Usage: const system = new MasterAimLockSystem('PRESET_NAME');");
