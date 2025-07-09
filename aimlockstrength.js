// == Vector3 ==
class Vector3 {
  constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; }
  set(x, y, z) { this.x = x; this.y = y; this.z = z; return this; }
  addMut(v) { this.x += v.x; this.y += v.y; this.z += v.z; return this; }
  subtractMut(v) { this.x -= v.x; this.y -= v.y; this.z -= v.z; return this; }
  multiplyScalarMut(s) { this.x *= s; this.y *= s; this.z *= s; return this; }
  add(v) { return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z); }
  subtract(v) { return new Vector3(this.x - v.x, this.y - v.y, this.z - v.z); }
  multiplyScalar(s) { return new Vector3(this.x * s, this.y * s, this.z * s); }
  length() { return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z); }
  lengthSq() { return this.x * this.x + this.y * this.y + this.z * this.z; }
  clone() { return new Vector3(this.x, this.y, this.z); }
  lerp(v, t) {
    const inv = 1 - t;
    this.x = this.x * inv + v.x * t;
    this.y = this.y * inv + v.y * t;
    this.z = this.z * inv + v.z * t;
    return this;
  }
  static zero() { return new Vector3(0, 0, 0); }
}

// == Kalman Filter ==
class KalmanFilter {
  constructor(R = 0.01, Q = 0.0001) {
    this.R = R; this.Q = Q; this.A = 1; this.C = 1;
    this.x = NaN; this.cov = NaN;
  }
  filter(z) {
    if (isNaN(this.x)) {
      this.x = z; this.cov = this.R;
      return this.x;
    }
    const predX = this.x;
    const predCov = this.cov + this.Q;
    const K = predCov / (predCov + this.R);
    this.x = predX + K * (z - predX);
    this.cov = predCov * (1 - K);
    return this.x;
  }
  reset() { this.x = NaN; this.cov = NaN; }
}

// == Matrix ==
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

// == AimLockHeadTracker ==
class AimLockHeadTracker {
  constructor() {
    this.kalmanX = new KalmanFilter();
    this.kalmanY = new KalmanFilter();
    this.kalmanZ = new KalmanFilter();
    this.lastAimPos = new Vector3();
    this.filteredPos = new Vector3();
    this.headVelocity = new Vector3();
    this.lastHeadPos = new Vector3();
    this.worldMatrix = new Float32Array(16);
    this.bindMatrix = new Float32Array(16);
    this.tempPos = new Vector3();
    this.lastRedTime = 0;
    this.holdDuration = 600;
    this.recoil = Vector3.zero();
    this.frameCount = 0;
    this.lastFrameTime = Date.now();
    this.currentFrameTime = 16;
    this.avgFrameTime = 16;
    this.minFrameTime = 8;
    this.maxFrameTime = 33;
    this.boundLoop = this.loop.bind(this);
    this.isRunning = false;
  }

  setRecoil(x, y, z) {
    this.recoil.set(x, y, z);
  }

  isCrosshairRed() {
    return typeof GameAPI !== "undefined" && GameAPI.crosshairState === "red";
  }

  applyKalman(pos) {
    return this.filteredPos.set(
      this.kalmanX.filter(pos.x),
      this.kalmanY.filter(pos.y),
      this.kalmanZ.filter(pos.z)
    );
  }

  applyRecoilCompensation(vec) {
    return vec.clone().subtractMut(this.recoil);
  }

  smoothTo(target, factor = 0.85) {
    return this.lastAimPos.lerp(target, 1 - factor);
  }

  setAim(vec3) {
    if (this.frameCount % 10 === 0) {
      console.log("🎯 Head lock:", vec3.x.toFixed(3), vec3.y.toFixed(3), vec3.z.toFixed(3));
    }
    if (typeof GameAPI !== "undefined" && GameAPI.setCrosshairTarget) {
      GameAPI.setCrosshairTarget(vec3.x, vec3.y, vec3.z);
    }
  }

  computeWorldHead(boneData) {
    const { position, rotation, scale, bindpose } = boneData;
    quaternionToMatrix(rotation, this.worldMatrix);
    this.worldMatrix[0] *= scale.x; this.worldMatrix[1] *= scale.y; this.worldMatrix[2] *= scale.z;
    this.worldMatrix[4] *= scale.x; this.worldMatrix[5] *= scale.y; this.worldMatrix[6] *= scale.z;
    this.worldMatrix[8] *= scale.x; this.worldMatrix[9] *= scale.y; this.worldMatrix[10] *= scale.z;
    this.worldMatrix[3] = position.x;
    this.worldMatrix[7] = position.y;
    this.worldMatrix[11] = position.z;
    const b = this.bindMatrix;
    b[0] = bindpose.e00; b[1] = bindpose.e01; b[2] = bindpose.e02; b[3] = bindpose.e03;
    b[4] = bindpose.e10; b[5] = bindpose.e11; b[6] = bindpose.e12; b[7] = bindpose.e13;
    b[8] = bindpose.e20; b[9] = bindpose.e21; b[10] = bindpose.e22; b[11] = bindpose.e23;
    this.tempPos.set(this.worldMatrix[3], this.worldMatrix[7], this.worldMatrix[11]);
    return multiplyMatrixVec(this.bindMatrix, this.tempPos);
  }

  predictHeadPosition(currentPos, dt) {
    if (this.frameCount > 1) {
      this.headVelocity.set(
        (currentPos.x - this.lastHeadPos.x) / dt,
        (currentPos.y - this.lastHeadPos.y) / dt,
        (currentPos.z - this.lastHeadPos.z) / dt
      );
      const predicted = currentPos.clone().addMut(this.headVelocity.clone().multiplyScalarMut(dt));
      this.lastHeadPos.set(currentPos.x, currentPos.y, currentPos.z);
      return predicted;
    }
    this.lastHeadPos.set(currentPos.x, currentPos.y, currentPos.z);
    return currentPos;
  }

  lockBoneHead(worldHead, dt) {
    const now = Date.now();
    if (this.isCrosshairRed()) {
      console.log("💥 Headshot Detected!");
      this.lastRedTime = now;
    }
    if (now - this.lastRedTime <= this.holdDuration) {
      const predicted = this.predictHeadPosition(worldHead, dt);
      const compensated = this.applyRecoilCompensation(predicted);
      const filtered = this.applyKalman(compensated);
      const smooth = this.smoothTo(filtered, 0.7);
      this.setAim(smooth);
    }
  }

  loop() {
    if (!this.isRunning) return;
    const now = Date.now();
    const dt = (now - this.lastFrameTime) / 1000;
    this.lastFrameTime = now;
    const headWorld = this.computeWorldHead(this.boneData);
    this.lockBoneHead(headWorld, dt);
    this.frameCount++;
    setTimeout(this.boundLoop, this.currentFrameTime);
  }

  start(boneData) {
    if (this.isRunning) return;
    this.boneData = boneData;
    this.isRunning = true;
    this.lastFrameTime = Date.now();
    console.log("🚀 AimLock started");
    this.loop();
  }

  stop() {
    this.isRunning = false;
    console.log("⏹️ AimLock stopped");
  }

  getStats() {
    return {
      fps: Math.round(1000 / this.currentFrameTime),
      avgFrameTime: this.avgFrameTime.toFixed(2),
      frameCount: this.frameCount,
      isRunning: this.isRunning
    };
  }
}

class MultiEnemyTrackerManager {
  constructor(enemyList) {
    this.trackers = enemyList.map(data => {
      const tracker = new AimLockHeadTracker();
      tracker.start(data);
      return tracker;
    });
  }

  updateRecoilToAll(recoilVec) {
    this.trackers.forEach(t => t.setRecoil(recoilVec.x, recoilVec.y, recoilVec.z));
  }

  stopAll() {
    this.trackers.forEach(t => t.stop());
  }
}

const enemy1_boneHeadData = {
  position: { x: -0.045, y: -0.004, z: -0.020 },
  rotation: { x: 0.02, y: -0.08, z: -0.14, w: 0.98 },
  scale: { x: 1, y: 1, z: 1 },
  bindpose: {
    e00: -1.34559613e-13, e01: 8.881784e-14, e02: -1.0, e03: 0.487912,
    e10: -2.84512817e-6, e11: -1.0, e12: 8.881784e-14, e13: -2.842171e-14,
    e20: -1.0, e21: 2.84512817e-6, e22: -1.72951931e-13, e23: 0.0,
    e30: 0.0, e31: 0.0, e32: 0.0, e33: 1.0
  }
};

const enemy2_boneHeadData = {
  position: { x: -0.045, y: -0.004, z: -0.020 },
  rotation: { x: 0.02, y: -0.08, z: -0.14, w: 0.98 },
  scale: { x: 1, y: 1, z: 1 },
  bindpose: {
    e00: -1.34559613e-13, e01: 8.881784e-14, e02: -1.0, e03: 0.487912,
    e10: -2.84512817e-6, e11: -1.0, e12: 8.881784e-14, e13: -2.842171e-14,
    e20: -1.0, e21: 2.84512817e-6, e22: -1.72951931e-13, e23: 0.0,
    e30: 0.0, e31: 0.0, e32: 0.0, e33: 1.0
  }
};

const enemy3_boneHeadData = {
  position: { x: -0.045, y: -0.004, z: -0.020 },
  rotation: { x: 0.02, y: -0.08, z: -0.14, w: 0.98 },
  scale: { x: 1, y: 1, z: 1 },
  bindpose: {
    e00: -1.34559613e-13, e01: 8.881784e-14, e02: -1.0, e03: 0.487912,
    e10: -2.84512817e-6, e11: -1.0, e12: 8.881784e-14, e13: -2.842171e-14,
    e20: -1.0, e21: 2.84512817e-6, e22: -1.72951931e-13, e23: 0.0,
    e30: 0.0, e31: 0.0, e32: 0.0, e33: 1.0
  }
};

const enemies = [enemy1_boneHeadData, enemy2_boneHeadData, enemy3_boneHeadData];
const recoilVec = new Vector3(0.002, -0.001, 0.001);
const manager = new MultiEnemyTrackerManager(enemies);

// Update recoil liên tục nếu cần
manager.updateRecoilToAll(recoilVec);

// Khi muốn dừng
// manager.stopAll();
