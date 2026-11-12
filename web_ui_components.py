#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PIPER 视觉抓放控制台前端组件库 (单臂演示台 :8081 + 双臂协同总控台 :8080)
特性:
1. 100% 离线零外部 CDN 依赖的原生 HTML5 Canvas 3D 质点轨迹引擎 (支持鼠标拖拽旋转、滚轮缩放、立体/俯视/侧视切换)
2. 双臂网页采用【机械臂 A 栏 | 机械臂 B 栏 | 双臂配合栏】三栏集成设计 + 预留臂端手眼一体摄像头接口
3. 单臂网页保持纯净单臂体验，同样内置 3D 桌面与方块质点轨迹界面
"""

TRAJ_3D_SCRIPT = r'''
class Trajectory3DViewer {
  constructor(canvasId, infoId, opts = {}) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas ? this.canvas.getContext('2d') : null;
    this.infoEl = infoId ? document.getElementById(infoId) : null;
    this.defaultCenter = opts.center ? [...opts.center] : [200, 0, 0];
    this.center = [...this.defaultCenter];
    this.gridW = opts.gridW || 440;
    this.gridH = opts.gridH || 760;
    this.zoom = 1.0;
    this.dualMode = false;
    this.points = [];     // 单臂轨迹 [[x, y, z_above_table], ...] (mm)
    this.currPt = null;
    this.pointsA = [];    // 双臂模式下：方块 A 轨迹 (青蓝色)
    this.currPtA = null;
    this.pointsB = [];    // 双臂模式下：方块 B 轨迹 (亮紫色)
    this.currPtB = null;
    this.dragging = false;
    this.dragButton = 0;  // 0: left (orbit), 1: middle (pan), 2: right (zoom)
    this.lastX = 0;
    this.lastY = 0;
    // 3x3 正交视图矩阵 R：将世界 (x,y,z) 转为右手法相机系 (Xc右, Yc上, Zc朝向观察者)
    this.R = [1, 0, 0,  0, 1, 0,  0, 0, 1];
    this.resetView();

    if (this.canvas) {
      this.canvas.addEventListener('contextmenu', (e) => e.preventDefault());

      const startDrag = (e) => {
        this.dragging = true;
        this.dragButton = e.button !== undefined ? e.button : 0;
        this.lastX = e.clientX;
        this.lastY = e.clientY;
      };
      const moveDrag = (e) => {
        if (!this.dragging) return;
        const dx = e.clientX - this.lastX;
        const dy = e.clientY - this.lastY;
        this.lastX = e.clientX;
        this.lastY = e.clientY;

        if (this.dragButton === 0) {
          // 左键：Gazebo 360° 自由旋转 Orbit
          this.rotateByScreenDelta(dx, dy);
        } else if (this.dragButton === 1) {
          // 中键/滚轮拖动：Gazebo 视平面平移 Pan
          this.panByScreenDelta(dx, dy);
        } else if (this.dragButton === 2) {
          // 右键拖动：Gazebo 缩放 Zoom
          this.zoomByScreenDelta(dy);
        }
        this.render();
      };
      this.canvas.addEventListener('mousedown', (e) => { startDrag(e); });
      window.addEventListener('mouseup', () => { this.dragging = false; });
      window.addEventListener('mousemove', (e) => { moveDrag(e); });

      this.canvas.addEventListener('touchstart', (e) => {
        if (e.touches.length === 1) {
          e.preventDefault();
          this.dragging = true;
          this.dragButton = 0;
          this.lastX = e.touches[0].clientX;
          this.lastY = e.touches[0].clientY;
        }
      }, { passive: false });
      window.addEventListener('touchend', () => { this.dragging = false; });
      window.addEventListener('touchmove', (e) => {
        if (this.dragging && e.touches.length === 1) {
          const dx = e.touches[0].clientX - this.lastX;
          const dy = e.touches[0].clientY - this.lastY;
          this.lastX = e.touches[0].clientX;
          this.lastY = e.touches[0].clientY;
          this.rotateByScreenDelta(dx, dy);
          this.render();
        }
      }, { passive: true });

      // 双击复位
      this.canvas.addEventListener('dblclick', () => { this.resetView(); });

      // 滚轮缩放
      this.canvas.addEventListener('wheel', (e) => {
        e.preventDefault();
        this.zoom = Math.max(0.35, Math.min(3.5, this.zoom * (e.deltaY > 0 ? 0.92 : 1.09)));
        this.render();
      }, { passive: false });

      // Gazebo 键盘快捷键 (R/Home:复位, T:俯视, F:正视, S:侧视)
      window.addEventListener('keydown', (e) => {
        const tag = (e.target.tagName || '').toLowerCase();
        if (tag === 'input' || tag === 'textarea' || tag === 'select') return;
        const key = e.key.toLowerCase();
        if (key === 'r' || e.key === 'Home') { this.resetView(); }
        else if (key === 't') { this.setTopView(); }
        else if (key === 'f') { this.setFrontView(); }
        else if (key === 's') { this.setSideView(); }
      });

      this.render();
    }
  }

  panByScreenDelta(dx, dy) {
    const f = (680 / (680 * this.zoom)) * 0.85;
    const r = this.R;
    this.center[0] += (-dx * r[0] + dy * r[3]) * f;
    this.center[1] += (-dx * r[1] + dy * r[4]) * f;
    this.center[2] += (-dx * r[2] + dy * r[5]) * f;
  }

  zoomByScreenDelta(dy) {
    this.zoom = Math.max(0.35, Math.min(3.5, this.zoom * Math.exp(-dy * 0.007)));
  }

  resetView() {
    this.center = [...this.defaultCenter];
    this.setView(0.95, 0.58, 1.0);
  }

  setTopView() {
    this.center = [...this.defaultCenter];
    this.setView(0, Math.PI / 2 - 0.001, this.zoom);
  }

  setFrontView() {
    this.center = [...this.defaultCenter];
    this.setView(Math.PI / 2, 0.05, this.zoom);
  }

  setSideView() {
    this.center = [...this.defaultCenter];
    this.setView(0, 0.05, this.zoom);
  }

  _orthonormalize() {
    // Gram-Schmidt 正交化并保证右手系 det(R)=+1，消除连续 360° 拖拽浮点漂移
    const r = this.R;
    let l0 = Math.hypot(r[0], r[1], r[2]) || 1;
    let r0 = [r[0] / l0, r[1] / l0, r[2] / l0];
    const dot01 = r0[0] * r[3] + r0[1] * r[4] + r0[2] * r[5];
    let r1 = [r[3] - dot01 * r0[0], r[4] - dot01 * r0[1], r[5] - dot01 * r0[2]];
    let l1 = Math.hypot(r1[0], r1[1], r1[2]) || 1;
    r1 = [r1[0] / l1, r1[1] / l1, r1[2] / l1];
    const r2 = [
      r0[1] * r1[2] - r0[2] * r1[1],
      r0[2] * r1[0] - r0[0] * r1[2],
      r0[0] * r1[1] - r0[1] * r1[0],
    ];
    this.R = [r0[0], r0[1], r0[2], r1[0], r1[1], r1[2], r2[0], r2[1], r2[2]];
  }

  rotateByScreenDelta(dx, dy) {
    // 无死角 360° 自由轨迹球：绕屏幕上方轴(Yc)水平旋转，绕屏幕右轴(Xc)竖直翻转，无角度夹逼、无万向节锁
    const ay = dx * 0.0085;
    const ax = dy * 0.0085;
    const cy = Math.cos(ay), sy = Math.sin(ay);
    const cx = Math.cos(ax), sx = Math.sin(ax);
    const r = this.R;
    // 先绕 Yc 偏航: Xc' = cy*Xc + sy*Zc, Zc' = -sy*Xc + cy*Zc
    const a0 = [cy * r[0] + sy * r[6], cy * r[1] + sy * r[7], cy * r[2] + sy * r[8]];
    const a2 = [-sy * r[0] + cy * r[6], -sy * r[1] + cy * r[7], -sy * r[2] + cy * r[8]];
    // 再绕 Xc 俯仰: Yc' = cx*Yc - sx*Zc', Zc'' = sx*Yc + cx*Zc'
    const b1 = [cx * r[3] - sx * a2[0], cx * r[4] - sx * a2[1], cx * r[5] - sx * a2[2]];
    const b2 = [sx * r[3] + cx * a2[0], sx * r[4] + cx * a2[1], sx * r[5] + cx * a2[2]];
    this.R = [a0[0], a0[1], a0[2], b1[0], b1[1], b1[2], b2[0], b2[1], b2[2]];
    this._orthonormalize();
  }

  setView(yaw, pitch, zoom = 1.0) {
    // pitch>0 代表相机抬升至桌面之上俯视 (pitch=pi/2 为正上方垂直俯视，+Z 朝向人眼)
    const cy = Math.cos(yaw), sy = Math.sin(yaw);
    const cp = Math.cos(pitch), sp = Math.sin(pitch);
    this.R = [
      cy,      -sy,      0,
      sp * sy,  sp * cy, cp,
     -cp * sy, -cp * cy, sp
    ];
    this.zoom = zoom;
    if (this.ctx) this.render();
  }

  updateData(points, currPt) {
    this.dualMode = false;
    this.points = Array.isArray(points) ? points : [];
    this.currPt = currPt || null;
    this.render();
  }

  updateDualData(dataA, dataB) {
    this.dualMode = true;
    this.pointsA = (dataA && Array.isArray(dataA.points)) ? dataA.points : [];
    this.currPtA = (dataA && dataA.curr) ? dataA.curr : null;
    this.pointsB = (dataB && Array.isArray(dataB.points)) ? dataB.points : [];
    this.currPtB = (dataB && dataB.curr) ? dataB.curr : null;
    this.render();
  }

  project(pt) {
    const x = pt[0] - this.center[0], y = pt[1] - this.center[1], z = pt[2] - this.center[2];
    const r = this.R;
    const xc = r[0] * x + r[1] * y + r[2] * z;
    const yc = r[3] * x + r[4] * y + r[5] * z;
    const zc = r[6] * x + r[7] * y + r[8] * z; // zc>0 靠近人眼，zc<0 远离人眼
    const depth = Math.max(160, 1080 - zc);
    const scale = (680 / depth) * this.zoom;
    const w = this.canvas.width, h = this.canvas.height;
    return [w * 0.5 + xc * scale, h * 0.52 - yc * scale, scale, depth];
  }

  heightColor(z) {
    const t = Math.max(0, Math.min(1, z / 90.0));
    if (t < 0.35) return '#38bdf8';
    if (t < 0.7) return '#4ade80';
    return '#facc15';
  }

  _drawTableAndAxes(ctx, fromTop) {
    const cx = this.center[0], cy = this.center[1];
    const xMin = cx - this.gridW / 2, xMax = cx + this.gridW / 2;
    const yMin = cy - this.gridH / 2, yMax = cy + this.gridH / 2;

    const c00 = this.project([xMin, yMin, 0]);
    const c10 = this.project([xMax, yMin, 0]);
    const c11 = this.project([xMax, yMax, 0]);
    const c01 = this.project([xMin, yMax, 0]);
    ctx.beginPath();
    ctx.moveTo(c00[0], c00[1]); ctx.lineTo(c10[0], c10[1]);
    ctx.lineTo(c11[0], c11[1]); ctx.lineTo(c01[0], c01[1]);
    ctx.closePath();
    ctx.fillStyle = fromTop ? 'rgba(16, 42, 48, 0.56)' : 'rgba(30, 27, 75, 0.42)';
    ctx.fill();
    ctx.strokeStyle = fromTop ? '#1e4a5f' : '#4338ca';
    ctx.lineWidth = 1.6;
    ctx.stroke();

    ctx.lineWidth = 1;
    ctx.strokeStyle = fromTop ? 'rgba(56, 189, 248, 0.16)' : 'rgba(129, 140, 248, 0.16)';
    for (let x = Math.ceil(xMin / 100) * 100; x <= xMax; x += 100) {
      const a = this.project([x, yMin, 0]), b = this.project([x, yMax, 0]);
      ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
    }
    for (let y = Math.ceil(yMin / 100) * 100; y <= yMax; y += 100) {
      const a = this.project([xMin, y, 0]), b = this.project([xMax, y, 0]);
      ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
    }

    // 绘制极简桌面坐标角标 (+X/+Y/+Z)
    const o = this.project([xMin + 25, yMin + 25, 0]);
    const ox = this.project([xMin + 105, yMin + 25, 0]);
    const oy = this.project([xMin + 25, yMin + 105, 0]);
    const oz = this.project([xMin + 25, yMin + 25, 80]);
    const drawAxis = (p, color, lbl) => {
      ctx.beginPath(); ctx.moveTo(o[0], o[1]); ctx.lineTo(p[0], p[1]);
      ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.stroke();
      ctx.fillStyle = color; ctx.font = 'bold 11px monospace'; ctx.fillText(lbl, p[0] + 4, p[1] - 2);
    };
    drawAxis(ox, '#f87171', '+X');
    drawAxis(oy, '#4ade80', '+Y');
    drawAxis(oz, '#60a5fa', '+Z');
  }

  _drawTrajectory(ctx, ptsList = null, fixedColor = null) {
    const pts = ptsList || this.points;
    if (!pts || pts.length < 2) return;
    const segs = [];
    for (let i = 1; i < pts.length; i++) {
      const p0 = this.project(pts[i - 1]);
      const p1 = this.project(pts[i]);
      segs.push({ p0, p1, zMid: (pts[i - 1][2] + pts[i][2]) * 0.5, dMid: (p0[3] + p1[3]) * 0.5 });
    }
    segs.sort((a, b) => b.dMid - a.dMid); // 由远到近遮挡排序
    ctx.lineCap = 'round';
    for (const s of segs) {
      ctx.beginPath();
      ctx.moveTo(s.p0[0], s.p0[1]);
      ctx.lineTo(s.p1[0], s.p1[1]);
      ctx.strokeStyle = fixedColor || this.heightColor(s.zMid);
      ctx.lineWidth = 2.4;
      ctx.stroke();
    }
    const pStart = this.project(pts[0]);
    ctx.beginPath();
    ctx.arc(pStart[0], pStart[1], 3.2, 0, Math.PI * 2);
    ctx.fillStyle = fixedColor || '#94a3b8';
    ctx.fill();
  }

  _drawCubeParticle(ctx, w, pt = null, palette = null, labelPrefix = '') {
    const target = pt || this.currPt;
    if (!target) return;
    const pal = palette || {
      core: '#ffffff', inner: '#fef08a', mid: '#facc15',
      fade: 'rgba(250, 204, 21, 0)', line: 'rgba(250, 204, 21, 0.75)',
      shadow: 'rgba(250, 204, 21, 0.45)', txt: '#fef08a'
    };

    const pTop = this.project(target);
    const pShadow = this.project([target[0], target[1], 0]);

    ctx.save();
    ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.moveTo(pShadow[0], pShadow[1]); ctx.lineTo(pTop[0], pTop[1]);
    ctx.strokeStyle = pal.line; ctx.lineWidth = 1.4; ctx.stroke();
    ctx.restore();

    ctx.beginPath(); ctx.arc(pShadow[0], pShadow[1], 4, 0, Math.PI * 2);
    ctx.fillStyle = pal.shadow; ctx.fill();

    const r = Math.max(6, Math.min(12, 7.5 * pTop[2]));
    const grad = ctx.createRadialGradient(pTop[0] - 2, pTop[1] - 2, 1, pTop[0], pTop[1], r * 1.8);
    grad.addColorStop(0, pal.inner);
    grad.addColorStop(0.5, pal.mid);
    grad.addColorStop(1, pal.fade);
    ctx.beginPath(); ctx.arc(pTop[0], pTop[1], r * 1.8, 0, Math.PI * 2);
    ctx.fillStyle = grad; ctx.fill();

    ctx.beginPath(); ctx.arc(pTop[0], pTop[1], r * 0.65, 0, Math.PI * 2);
    ctx.fillStyle = pal.core; ctx.fill();

    ctx.fillStyle = pal.txt;
    ctx.font = 'bold 11px monospace';
    ctx.fillText(
      `${labelPrefix}(${target[0].toFixed(0)}, ${target[1].toFixed(0)}, H:${target[2].toFixed(0)})`,
      Math.min(w - 155, Math.max(8, pTop[0] + 10)),
      Math.max(16, Math.min(this.canvas.height - 8, pTop[1] - 8))
    );
  }

  render() {
    if (!this.ctx) return;
    const ctx = this.ctx, w = this.canvas.width, h = this.canvas.height;
    ctx.fillStyle = '#050b16';
    ctx.fillRect(0, 0, w, h);

    const palA = {
      core: '#ffffff', inner: '#bae6fd', mid: '#38bdf8',
      fade: 'rgba(56, 189, 248, 0)', line: 'rgba(56, 189, 248, 0.78)',
      shadow: 'rgba(56, 189, 248, 0.45)', txt: '#38bdf8'
    };
    const palB = {
      core: '#ffffff', inner: '#f5d0fe', mid: '#e879f9',
      fade: 'rgba(232, 121, 249, 0)', line: 'rgba(232, 121, 249, 0.78)',
      shadow: 'rgba(232, 121, 249, 0.45)', txt: '#f0abfc'
    };

    const drawContents = () => {
      if (this.dualMode) {
        this._drawTrajectory(ctx, this.pointsA, 'rgba(56, 189, 248, 0.85)');
        this._drawTrajectory(ctx, this.pointsB, 'rgba(232, 121, 249, 0.85)');
        // 按深度从远到近绘制两个彩色质点，确保堆叠时空间遮挡正确
        const items = [];
        if (this.currPtA) items.push({ pt: this.currPtA, pal: palA, lbl: 'A:', d: this.project(this.currPtA)[3] });
        if (this.currPtB) items.push({ pt: this.currPtB, pal: palB, lbl: 'B:', d: this.project(this.currPtB)[3] });
        items.sort((x, y) => y.d - x.d);
        for (const it of items) {
          this._drawCubeParticle(ctx, w, it.pt, it.pal, it.lbl);
        }
      } else {
        this._drawTrajectory(ctx, this.points, null);
        this._drawCubeParticle(ctx, w, this.currPt, null, '');
      }
    };

    // Z 轴在相机朝向分量 R[8] >= 0 表示处于桌面上方（俯视/侧俯）；<0 表示处于桌面下方（仰视）
    const fromTop = this.R[8] >= 0;
    if (fromTop) {
      this._drawTableAndAxes(ctx, true);
      drawContents();
    } else {
      // 从桌底仰视时，先画上方方块与轨迹，再覆盖半透明桌底网格，保持空间前后关系真实不颠倒
      drawContents();
      this._drawTableAndAxes(ctx, false);
    }

    if (this.infoEl) {
      const sideLabel = fromTop ? '俯视角' : '底面仰视';
      if (this.dualMode) {
        const sA = this.currPtA ? `🔵A(${this.currPtA[0].toFixed(0)},${this.currPtA[1].toFixed(0)},H${this.currPtA[2].toFixed(0)})` : '🔵A:--';
        const sB = this.currPtB ? `🟣B(${this.currPtB[0].toFixed(0)},${this.currPtB[1].toFixed(0)},H${this.currPtB[2].toFixed(0)})` : '🟣B:--';
        this.infoEl.innerText = `${sA} | ${sB} · ${sideLabel}`;
      } else {
        this.infoEl.innerText = this.currPt
          ? `质点: (${this.currPt[0].toFixed(1)}, ${this.currPt[1].toFixed(1)}, Z=${this.currPt[2].toFixed(1)}mm) · ${this.points.length}点 · ${sideLabel}`
          : `桌面待测 (${sideLabel} · 360°自由拖拽 · 双击复位) · ${this.points.length}点`;
      }
    }
  }
}
'''


def render_single_arm_html() -> str:
    return r'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>PIPER 机械臂 × RealSense 全自动视觉抓取与 3D 轨迹控制台</title>
<style>
  * { box-sizing: border-box; }
  body { margin: 0; padding: 14px; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; background: #0b1120; color: #f8fafc; }
  .container { max-width: 1440px; margin: 0 auto; }
  header { display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 10px; border-bottom: 1px solid #1e293b; padding-bottom: 10px; margin-bottom: 14px; }
  h1 { margin: 0; font-size: 18px; color: #38bdf8; }
  .hdr-badges { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
  .pill { font-size: 12px; font-weight: 600; padding: 4px 10px; border-radius: 9999px; background: #1e293b; border: 1px solid #334155; }
  .pill.ok { background: #14532d; color: #86efac; border-color: #22c55e; }
  .pill.warn { background: #78350f; color: #fde68a; border-color: #f59e0b; }
  .pill.err { background: #450a0a; color: #fca5a5; border-color: #ef4444; }
  .top-grid { display: grid; grid-template-columns: 1.35fr 1fr; gap: 16px; }
  @media (max-width: 1060px) { .top-grid { grid-template-columns: 1fr; } }
  .card { background: #111827; border: 1px solid #334155; border-radius: 12px; overflow: visible; box-shadow: 0 8px 20px rgba(0,0,0,0.45); position: relative; }
  .card-hdr { padding: 9px 14px; background: #1e293b; display: flex; justify-content: space-between; align-items: center; font-size: 13px; font-weight: 600; }
  .canvas-wrapper { position: relative; background: #000; display: flex; justify-content: center; overflow: hidden; border-radius: 0 0 11px 11px; }
  canvas { width: 100%; height: auto; display: block; }
  #live-canvas { max-width: 640px; aspect-ratio: 4/3; cursor: crosshair; }
  #traj-3d-canvas { aspect-ratio: 16/10; cursor: grab; background: #050b16; }
  #traj-3d-canvas:active { cursor: grabbing; }
  .fps-badge { position: absolute; top: 10px; right: 14px; background: rgba(0,0,0,0.75); color: #4ade80; padding: 3px 8px; border-radius: 6px; font-size: 11px; font-family: monospace; font-weight: 700; border: 1px solid rgba(74,222,128,0.4); }
  .tip-banner { padding: 8px 12px; background: #0f172a; font-size: 12px; color: #93c5fd; border-top: 1px solid #1e293b; display: flex; justify-content: space-between; align-items: center; gap: 8px; flex-wrap: wrap; }
  .tabs { display: flex; background: #0f172a; border-bottom: 1px solid #334155; }
  .tab-btn { flex: 1; padding: 10px 8px; background: transparent; border: none; color: #94a3b8; font-weight: 700; font-size: 13px; cursor: pointer; border-bottom: 3px solid transparent; border-radius: 0; }
  .tab-btn.active { color: #38bdf8; border-bottom-color: #38bdf8; background: #111827; }
  .tab-content { display: none; padding: 12px; flex-direction: column; gap: 9px; }
  .tab-content.active { display: flex; }
  .box { background: #0f172a; border: 1px solid #1e293b; border-radius: 8px; padding: 9px 11px; font-size: 12px; }
  .row-between { display: flex; justify-content: space-between; align-items: center; gap: 8px; }
  .mono { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-weight: 600; color: #38bdf8; }
  button { border: none; padding: 9px 13px; border-radius: 7px; font-size: 13px; font-weight: 600; cursor: pointer; color: #fff; transition: 0.15s; }
  button:disabled { opacity: 0.4; cursor: not-allowed; }
  button:hover:not(:disabled) { filter: brightness(1.12); }
  .btn-green { background: linear-gradient(135deg, #16a34a, #15803d); }
  .btn-blue { background: linear-gradient(135deg, #2563eb, #1d4ed8); }
  .btn-amber { background: #d97706; }
  .btn-purple { background: #7c3aed; }
  .btn-red { background: #dc2626; }
  .btn-slate { background: #475569; }
  .btn-sm { padding: 4px 9px; font-size: 11.5px; }
  .log-box { background: #020617; border: 1px solid #1e293b; border-radius: 8px; padding: 8px 10px; font-family: monospace; font-size: 11px; line-height: 1.5; color: #cbd5e1; height: 125px; overflow-y: auto; white-space: pre-wrap; }
  table { width: 100%; border-collapse: collapse; font-size: 11px; }
  th, td { border-bottom: 1px solid #1e293b; padding: 4px; text-align: left; }
  select, input[type="range"] { background: #1e293b; color: #f8fafc; border: 1px solid #334155; border-radius: 6px; padding: 4px 7px; font-size: 12px; }
  .has-tip { position: relative; cursor: help; border-bottom: 1px dotted #38bdf8; display: inline-flex; align-items: center; }
  .has-tip:hover::after {
    content: attr(data-tip);
    position: absolute;
    bottom: 125%;
    left: 50%;
    transform: translateX(-50%);
    background: #0f172a;
    color: #e2e8f0;
    padding: 7px 10px;
    border-radius: 6px;
    font-size: 11px;
    line-height: 1.45;
    white-space: normal;
    width: max-content;
    max-width: 230px;
    z-index: 9999;
    box-shadow: 0 4px 14px rgba(0,0,0,0.6);
    border: 1px solid #38bdf8;
    pointer-events: none;
    font-weight: normal;
    text-align: left;
  }
  .has-tip:hover::before {
    content: '';
    position: absolute;
    bottom: 115%;
    left: 50%;
    transform: translateX(-50%);
    border-width: 5px;
    border-style: solid;
    border-color: #38bdf8 transparent transparent transparent;
    z-index: 10000;
  }
  /* 靠左对齐：防止最左侧文字被遮挡 */
  .has-tip.tip-left:hover::after {
    left: 0;
    transform: none;
  }
  .has-tip.tip-left:hover::before {
    left: 18px;
    transform: none;
  }
  /* 靠右对齐：防止最右侧文字被遮挡 */
  .has-tip.tip-right:hover::after {
    left: auto;
    right: 0;
    transform: none;
  }
  .has-tip.tip-right:hover::before {
    left: auto;
    right: 18px;
    transform: none;
  }
</style>
</head>
<body>
<div class="container">
  <header>
    <h1>🦾 203实验室 PIPER 机械臂视觉抓取与 3D 质点轨迹总控台</h1>
    <div class="hdr-badges">
      <span class="pill err" id="badge-cam">📷 相机: 检测中</span>
      <span class="pill err" id="badge-arm">🦾 机械臂: 检测中</span>
      <button class="btn-amber btn-sm" onclick="postApi('/api/enable_arm', {})">⚡ 上电使能</button>
      <button class="btn-green btn-sm" onclick="postApi('/api/start_camera')">📷 启动/重启摄像头</button>
    </div>
  </header>

  <div class="top-grid">
    <!-- 左列：实时摄像头 + 3D 桌面质点轨迹 + 辅助视角 -->
    <div style="display:flex; flex-direction:column; gap:12px;">
      <div class="card">
        <div class="card-hdr">
          <span>📷 RealSense D435 实时桌面视野 (鼠标点击桌面选定放置目标)</span>
          <span style="color:#94a3b8; font-size:11.5px;">640x480 · 深度对齐</span>
        </div>
        <div class="canvas-wrapper">
          <canvas id="live-canvas" width="640" height="480"></canvas>
          <div class="fps-badge" id="fps-indicator">SYNC...</div>
        </div>
        <div class="tip-banner">
          <span>💡 左键点击绿桌面设定目标 · 右键点击或按右侧按钮清除 🎯</span>
          <button class="btn-red btn-sm" id="btn-clear-pt" onclick="postApi('/api/clear_point')" style="display:none;">❌ 消除目标点</button>
          <button class="btn-slate btn-sm" id="btn-clear-pt-idle" onclick="postApi('/api/clear_point')">消除目标点</button>
        </div>
      </div>

      <!-- 3D 质点轨迹视图 (只有桌面与方块质点轨迹) -->
      <div class="card">
        <div class="card-hdr">
          <span>🧊 3D 桌面与方块质点运动轨迹 (Gazebo视角风格)</span>
          <div style="display:flex; gap:5px;">
            <button class="btn-slate btn-sm" onclick="viewer3d.resetView()" title="快捷键: R 或 Home">🔄 透视(R)</button>
            <button class="btn-slate btn-sm" onclick="viewer3d.setTopView()" title="快捷键: T">⬇️ 俯视(T)</button>
            <button class="btn-slate btn-sm" onclick="viewer3d.setFrontView()" title="快捷键: F">👁️ 正视(F)</button>
            <button class="btn-slate btn-sm" onclick="viewer3d.setSideView()" title="快捷键: S">➡️ 侧视(S)</button>
            <button class="btn-red btn-sm" onclick="postApi('/api/clear_trajectory')">清空轨迹</button>
          </div>
        </div>
        <canvas id="traj-3d-canvas" width="640" height="360"></canvas>
        <div class="tip-banner">
          <span id="traj-3d-info" class="mono" style="font-size:11.5px;">等待方块识别...</span>
          <span style="color:#38bdf8; font-size:11px;">🖱️ Gazebo操作: 左键旋转 · 中键平移 · 右键/滚轮缩放</span>
        </div>
      </div>

      <!-- Windows 本地辅助摄像头 -->
      <div class="card" style="padding:10px 12px;">
        <div class="row-between">
          <span style="font-size:12.5px; font-weight:600; color:#38bdf8;">🎥 Windows 本地辅助视角 (侧视补盲)</span>
          <button class="btn-slate btn-sm" id="btn-wincam" onclick="toggleWinCam()">开启副视角</button>
        </div>
        <div id="win-cam-box" style="display:none; margin-top:8px;">
          <img id="win-cam-img" style="width:100%; border-radius:6px; border:1px solid #334155;" src="" onerror="onWinCamErr()" />
          <div id="win-cam-msg" style="font-size:11px; color:#94a3b8; margin-top:4px;">连接源: http://127.0.0.1:8082/stream</div>
        </div>
      </div>
    </div>

    <!-- 右列：控制与标定面板 -->
    <div class="card" style="display:flex; flex-direction:column;">
      <div class="tabs">
        <button class="tab-btn active" onclick="switchTab('tab-ctrl', this)">🎮 视觉抓放操控</button>
        <button class="tab-btn" onclick="switchTab('tab-calib', this)">📐 在线手眼标定</button>
      </div>

      <div id="tab-ctrl" class="tab-content active">
        <div class="box row-between"><span>方块识别状态 (AprilTag):</span><span class="mono" id="st-tag">--</span></div>
        <div class="box row-between"><span>方块基座坐标 (X, Y, Z):</span><span class="mono" id="st-tag-pos">--</span></div>
        <div class="box row-between"><span>夹爪实时物理开度:</span><span class="mono" id="st-grip">0.0 mm</span></div>
        <div class="box">
          <div class="row-between" style="margin-bottom:2px;">
            <span style="color:#94a3b8; font-size:11.5px;">🎯 鼠标点击的目标放置点评估:</span>
            <button class="btn-slate btn-sm" style="padding:1px 6px; font-size:10.5px;" onclick="postApi('/api/clear_point')">❌ 消除</button>
          </div>
          <div class="mono" id="st-click" style="font-size:12px; color:#facc15;">请点击左侧桌面空白位置</div>
        </div>

        <div class="box">
          <div class="row-between"><span class="has-tip tip-left" data-tip="【下探深度】夹爪指尖相对方块顶面(AprilTag)向下深入量。默认-25.5mm直达方块腰部中心；数值太浅抓不稳，数值过深擦碰桌面。">⬇️ 相对方块顶下探深度:</span><span class="mono" id="val-zoff">-25.5 mm</span></div>
          <input type="range" id="slider-zoff" style="width:100%;" min="-32" max="-10" step="0.5" value="-25.5" onchange="updateGripperParams()" oninput="document.getElementById('val-zoff').innerText=this.value+' mm'">
          <div class="row-between" style="margin-top:4px;"><span class="has-tip tip-left" data-tip="【X轴抓点补偿】夹爪抓取中心沿机械臂基座X轴前后微调(mm)。正值向前推，负值向后收，用于补偿手眼标定轻微偏差。">↔️ X轴抓点补偿:</span><span class="mono" id="val-xoff">0.0 mm</span></div>
          <input type="range" id="slider-xoff" style="width:100%;" min="-15" max="15" step="0.5" value="0.0" onchange="updateGripperParams()" oninput="document.getElementById('val-xoff').innerText=(this.value>0?'+':'')+this.value+' mm'">
          <div class="row-between" style="margin-top:4px;"><span class="has-tip tip-left" data-tip="【Y轴抓点补偿】夹爪抓取中心沿机械臂基座Y轴左右微调(mm)。正值向左偏，负值向右偏，确保夹爪两指对称卡入方块两侧。">↕️ Y轴抓点补偿:</span><span class="mono" id="val-yoff">0.0 mm</span></div>
          <input type="range" id="slider-yoff" style="width:100%;" min="-15" max="15" step="0.5" value="0.0" onchange="updateGripperParams()" oninput="document.getElementById('val-yoff').innerText=(this.value>0?'+':'')+this.value+' mm'">
          <div class="row-between" style="margin-top:4px;"><span class="has-tip tip-left" data-tip="【抓持力矩与开度】左滑块为闭合电机力矩限制(默认350mN·m即0.35N·m)；右滑块为闭合目标物理间距(默认26.5mm包夹25mm方块)。">🤏 抓持力矩 / 开度:</span><span class="mono" id="val-ew">350 mN·m / 26.5 mm</span></div>
          <div style="display:flex; gap:8px;">
            <input type="range" id="slider-effort" style="flex:1;" min="200" max="1000" step="50" value="350" onchange="updateGripperParams()" oninput="updEWLabel()">
            <input type="range" id="slider-width" style="flex:1;" min="20" max="29" step="0.5" value="26.5" onchange="updateGripperParams()" oninput="updEWLabel()">
          </div>
          <div class="row-between" style="margin-top:6px; border-top:1px dashed #1e293b; padding-top:4px;">
            <span class="has-tip tip-left" data-tip="【运行速度】控制机械臂所有抓放与过渡轨迹的运动速率。默认100%标准速度，可调慢至20%方便细致观察，或调至150%高速。">⚡ 抓放速度: <b id="val-spd">100</b>%</span>
            <span class="mono" id="spd-tag" style="font-size:11px; color:#38bdf8;">标准</span>
          </div>
          <input type="range" id="slider-speed" style="width:100%;" min="20" max="150" step="5" value="100" onchange="updateGripperParams()" oninput="document.getElementById('val-spd').innerText=this.value; document.getElementById('spd-tag').innerText=(this.value<70?'慢速':(this.value>110?'高速':'标准'))">
        </div>

        <button class="btn-green" id="btn-return" onclick="postApi('/api/pick_and_return')">🤖 一键原地抓取并放回</button>
        <button class="btn-blue" id="btn-place" onclick="postApi('/api/pick_and_place')">📍 确认抓取并搬运到点击位置</button>
        <button class="btn-slate" id="btn-home" onclick="postApi('/api/retract_home')">🏠 机械臂安全回原点</button>
      </div>

      <div id="tab-calib" class="tab-content">
        <div class="box" style="border-color:#2563eb;">
          <div style="font-weight:700; color:#60a5fa; margin-bottom:6px;">第一步：示教与对准模式</div>
          <select id="cal-touch-mode" style="width:100%; margin-bottom:8px;" onchange="postApi('/api/calib/select_arm',{touch_mode:this.value})">
            <option value="tip_touch">指尖闭合轻点 Tag 中心 (推荐最准)</option>
            <option value="cube_grasp">指尖张开居中包络方块</option>
          </select>
          <div style="display:flex; gap:8px;">
            <button class="btn-amber" style="flex:1;" onclick="postApi('/api/arm_teach_mode',{enable:true})">🔓 开启示教拖动</button>
            <button class="btn-slate" style="flex:1;" onclick="postApi('/api/arm_teach_mode',{enable:false})">🔒 锁回 CAN 伺服</button>
          </div>
          <div style="font-size:11px; color:#94a3b8; margin-top:5px;" id="cal-arm-live">等待连接...</div>
        </div>
        <div class="box" style="border-color:#16a34a;">
          <div style="font-weight:700; color:#4ade80; margin-bottom:6px;">第二步：采集 4~6 个桌面分散点</div>
          <div style="display:flex; gap:8px;">
            <button class="btn-blue" style="flex:1;" onclick="postApi('/api/calib/capture_cam')">📸 1. 视觉锁定方块</button>
            <button class="btn-purple" style="flex:1;" onclick="postApi('/api/calib/record_arm')">📍 2. 记录臂尖位置</button>
          </div>
          <div style="margin-top:6px; font-size:11.5px;" id="cal-pending">请先移开手臂，点击【1. 视觉锁定方块】</div>
        </div>
        <div class="box" style="max-height:175px; overflow-y:auto;">
          <div class="row-between" style="margin-bottom:4px;"><b>标定点列表:</b><span class="mono" id="cal-rmse">RMSE: --</span></div>
          <table><thead><tr><th>#</th><th>相机坐标</th><th>臂尖坐标</th><th>误差</th><th>删</th></tr></thead><tbody id="cal-tbody"></tbody></table>
        </div>
        <div style="display:flex; gap:8px;">
          <button class="btn-green" style="flex:2;" onclick="postApi('/api/calib/solve_and_save')">💾 计算并保存外参生效</button>
          <button class="btn-red" style="flex:1;" onclick="postApi('/api/calib/clear_points')">🗑️ 清空</button>
        </div>
      </div>

      <div style="padding:0 12px 12px 12px;">
        <div style="font-size:11.5px; color:#94a3b8; font-weight:700; margin-bottom:4px;">运行日志:</div>
        <div class="log-box" id="log-box">等待指令...</div>
      </div>
    </div>
  </div>
</div>

<script>
''' + TRAJ_3D_SCRIPT + r'''
  const viewer3d = new Trajectory3DViewer('traj-3d-canvas', 'traj-3d-info', { center: [240, -50, 0], gridW: 540, gridH: 620 });
  let centerInitialized = false;
  const canvas = document.getElementById('live-canvas');
  const ctx = canvas.getContext('2d');
  const fpsBadge = document.getElementById('fps-indicator');
  let isFetching = false, frameCount = 0, lastFpsTime = performance.now(), paramsLoaded = false;

  function loadNextFrame() {
    if (isFetching) return;
    isFetching = true;
    const img = new Image();
    img.onload = () => {
      ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
      isFetching = false; frameCount++;
      const now = performance.now();
      if (now - lastFpsTime >= 1000) {
        fpsBadge.innerText = 'LIVE: ' + Math.round((frameCount * 1000) / (now - lastFpsTime)) + ' FPS';
        frameCount = 0; lastFpsTime = now;
      }
      requestAnimationFrame(loadNextFrame);
    };
    img.onerror = () => { isFetching = false; setTimeout(loadNextFrame, 250); };
    img.src = '/api/latest_frame.jpg?t=' + performance.now();
  }
  document.addEventListener('DOMContentLoaded', loadNextFrame);

  canvas.addEventListener('click', (e) => {
    const r = canvas.getBoundingClientRect();
    postApi('/api/select_point', {
      u: Math.round((e.clientX - r.left) * (640 / r.width)),
      v: Math.round((e.clientY - r.top) * (480 / r.height))
    });
  });
  canvas.addEventListener('contextmenu', (e) => {
    e.preventDefault();
    postApi('/api/clear_point');
  });

  function switchTab(id, btn) {
    document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
    document.querySelectorAll('.tab-btn').forEach(el => el.classList.remove('active'));
    document.getElementById(id).classList.add('active'); btn.classList.add('active');
  }
  function postApi(url, body = {}) {
    return fetch(url, { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify(body) })
      .then(r => r.json()).then(d => { if (d.status === 'error') alert(d.msg); updateStatus(); return d; });
  }
  function updEWLabel() {
    document.getElementById('val-ew').innerText =
      document.getElementById('slider-effort').value + ' mN·m / ' + document.getElementById('slider-width').value + ' mm';
  }
  function updateGripperParams() {
    postApi('/api/set_gripper', {
      effort: document.getElementById('slider-effort').value,
      width_mm: document.getElementById('slider-width').value,
      z_offset_mm: document.getElementById('slider-zoff').value,
      x_offset_mm: document.getElementById('slider-xoff').value,
      y_offset_mm: document.getElementById('slider-yoff').value,
      speed_pct: document.getElementById('slider-speed').value
    });
  }
  let winCamOn = false;
  function toggleWinCam() {
    winCamOn = !winCamOn;
    const box = document.getElementById('win-cam-box'), img = document.getElementById('win-cam-img'), btn = document.getElementById('btn-wincam');
    if (winCamOn) {
      box.style.display = 'block';
      img.src = 'http://127.0.0.1:8082/stream?t=' + Date.now();
      document.getElementById('win-cam-msg').innerText = '已连接本地辅助流: http://127.0.0.1:8082/stream';
      btn.innerText = '关闭副视角'; btn.style.background = '#dc2626';
    } else {
      box.style.display = 'none'; img.src = '';
      btn.innerText = '开启副视角'; btn.style.background = '#475569';
    }
  }
  function onWinCamErr() {
    document.getElementById('win-cam-msg').innerHTML = '<span style="color:#f87171;">⚠️ 未检测到 Windows 8082 摄像头推流，请先双击启动 start_win_camera.bat</span>';
  }

  function updateStatus() {
    fetch('/api/status?t=' + Date.now()).then(r => r.json()).then(d => {
      const bCam = document.getElementById('badge-cam');
      bCam.innerText = d.camera_online ? '📷 相机: 在线' : '📷 相机: 未启动';
      bCam.className = 'pill ' + (d.camera_online ? 'ok' : 'err');

      const aid = d.primary_arm || 'arm_a';
      const arm = d.arms[aid], rt = arm.runtime;
      const bArm = document.getElementById('badge-arm');
      if (rt.motors_alive) {
        bArm.innerText = '🦾 机械臂: 在线(' + (rt.ctrl_mode === 2 ? '示教模式' : (arm.calibrated ? '已标定' : '未标定')) + ')';
        bArm.className = 'pill ok';
      } else if (rt.logic_alive) {
        bArm.innerText = '🦾 机械臂: 主控在线(无状态流/查急停与重启)';
        bArm.className = 'pill warn';
      } else {
        bArm.innerText = '🦾 机械臂: CAN未连接';
        bArm.className = 'pill err';
      }

      if (!paramsLoaded) {
        paramsLoaded = true;
        document.getElementById('slider-zoff').value = arm.grasp_z_offset_mm;
        document.getElementById('val-zoff').innerText = arm.grasp_z_offset_mm + ' mm';
        document.getElementById('slider-xoff').value = arm.grasp_x_offset_mm;
        document.getElementById('val-xoff').innerText = (arm.grasp_x_offset_mm > 0 ? '+' : '') + arm.grasp_x_offset_mm + ' mm';
        const yoff = arm.grasp_y_offset_mm || 0.0;
        document.getElementById('slider-yoff').value = yoff;
        document.getElementById('val-yoff').innerText = (yoff > 0 ? '+' : '') + yoff + ' mm';
        document.getElementById('slider-effort').value = arm.grasp_effort;
        document.getElementById('slider-width').value = arm.grasp_target_mm;
        const s_spd = arm.speed_pct || 100;
        document.getElementById('slider-speed').value = s_spd;
        document.getElementById('val-spd').innerText = s_spd;
        document.getElementById('spd-tag').innerText = (s_spd < 70 ? '慢速' : (s_spd > 110 ? '高速' : '标准'));
        updEWLabel();
      }

      const stTag = document.getElementById('st-tag');
      stTag.innerText = d.tag_detected ? `已锁定 (ID: ${d.tag_id})` : '未检测到方块';
      stTag.style.color = d.tag_detected ? '#22c55e' : '#f97316';
      const armPos = (aid === 'arm_a') ? d.tag_arm_a_mm : d.tag_arm_b_mm;
      document.getElementById('st-tag-pos').innerText = armPos
        ? `[${armPos.join(', ')}] mm` : (d.tag_cam_mm ? `相机系 [${d.tag_cam_mm.join(', ')}]` : '--');
      document.getElementById('st-grip').innerText = rt.gripper_mm + ' mm';

      const stClick = document.getElementById('st-click');
      if (d.clicked_eval && d.clicked_eval[aid]) {
        const ev = d.clicked_eval[aid];
        stClick.innerText = (ev.valid ? '✅ ' : '❌ ') + ev.msg;
        stClick.style.color = ev.valid ? '#4ade80' : '#f87171';
      } else {
        stClick.innerText = '请点击左侧桌面空白位置'; stClick.style.color = '#facc15';
      }

      // 更新 3D 质点轨迹画面 (确保中心点跟随一次，不破坏用户平移视角)
      if (d.traj_3d) {
        if (d.traj_3d.center) {
          viewer3d.defaultCenter = d.traj_3d.center;
          if (!centerInitialized) {
            viewer3d.center = [...d.traj_3d.center];
            centerInitialized = true;
          }
        }
        viewer3d.updateData(d.traj_3d.points, d.traj_3d.curr);
      }

      const bClr = document.getElementById('btn-clear-pt'), bClrIdle = document.getElementById('btn-clear-pt-idle');
      if (bClr && bClrIdle) {
        bClr.style.display = d.clicked_uv ? 'inline-block' : 'none';
        bClrIdle.style.display = d.clicked_uv ? 'none' : 'inline-block';
      }

      document.getElementById('cal-arm-live').innerText =
        `状态=${rt.motors_alive ? (rt.ctrl_mode===2 ? '🔓自由拖动示教' : '🔒CAN伺服') : (rt.logic_alive ? '⚠️仅主控应答0x4AF(无0x2A1状态流)' : '🔴离线')} | 关节=[${rt.joints_deg.join(', ')}]`;
      document.getElementById('cal-pending').innerHTML = d.calib.pending_cam_mm
        ? `<span style="color:#4ade80;">✅ 已锁存视觉 [${d.calib.pending_cam_mm.join(', ')}] mm</span>` : '请移开手臂点【1. 视觉锁定方块】';
      document.getElementById('cal-rmse').innerText = d.calib.preview_rmse_mm !== null ? `RMSE: ${d.calib.preview_rmse_mm} mm` : `点=${d.calib.points.length}`;
      document.getElementById('cal-tbody').innerHTML = d.calib.points.map((p, idx) => `<tr>
        <td>C${p.idx}</td><td>${p.p_cam_mm.join(',')}</td><td>${p.p_base_mm.join(',')}</td>
        <td>${p.err_mm !== null ? p.err_mm+'mm' : '-'}</td>
        <td><button class="btn-red btn-sm" style="padding:1px 5px;" onclick="postApi('/api/calib/delete_point',{idx:${idx}})">删</button></td>
      </tr>`).join('');

      document.getElementById('btn-return').disabled = d.task_busy || !d.tag_detected;
      document.getElementById('btn-place').disabled = d.task_busy || !d.tag_detected || !d.clicked_uv;
      document.getElementById('btn-home').disabled = d.task_busy;

      const logBox = document.getElementById('log-box');
      if (d.logs && d.logs.length > 0) {
        logBox.innerText = d.logs.slice(-12).join('\n');
        logBox.scrollTop = logBox.scrollHeight;
      }
    }).catch(() => {});
  }
  setInterval(updateStatus, 250);
</script>
</body>
</html>
'''


def render_dual_arm_html() -> str:
    return r'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>PIPER 双臂协同 · 三栏式操控与 3D 质点轨迹总控台</title>
<style>
  * { box-sizing: border-box; }
  body { margin: 0; padding: 14px; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; background: #0b1120; color: #f8fafc; }
  .container { max-width: 1520px; margin: 0 auto; }
  header { display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 10px; border-bottom: 1px solid #1e293b; padding-bottom: 10px; margin-bottom: 14px; }
  h1 { margin: 0; font-size: 18px; color: #38bdf8; }
  .hdr-badges { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
  .pill { font-size: 12px; font-weight: 600; padding: 4px 10px; border-radius: 9999px; background: #1e293b; border: 1px solid #334155; }
  .pill.ok { background: #14532d; color: #86efac; border-color: #22c55e; }
  .pill.warn { background: #78350f; color: #fde68a; border-color: #f59e0b; }
  .pill.err { background: #450a0a; color: #fca5a5; border-color: #ef4444; }
  .top-visuals { display: grid; grid-template-columns: 1.25fr 1fr; gap: 14px; margin-bottom: 14px; }
  .three-cols { display: grid; grid-template-columns: repeat(3, 1fr); gap: 14px; align-items: start; }
  @media (max-width: 1120px) { .top-visuals, .three-cols { grid-template-columns: 1fr; } }
  .card { background: #111827; border: 1px solid #334155; border-radius: 12px; overflow: visible; box-shadow: 0 8px 20px rgba(0,0,0,0.45); display: flex; flex-direction: column; position: relative; }
  .card-hdr { padding: 9px 13px; background: #1e293b; display: flex; justify-content: space-between; align-items: center; font-size: 13px; font-weight: 700; }
  .col-body { padding: 12px; display: flex; flex-direction: column; gap: 9px; }
  .canvas-wrapper { position: relative; background: #000; display: flex; justify-content: center; overflow: hidden; border-radius: 0 0 11px 11px; }
  canvas { width: 100%; height: auto; display: block; }
  #live-canvas { aspect-ratio: 4/3; cursor: crosshair; }
  #traj-3d-canvas { aspect-ratio: 16/10; cursor: grab; background: #050b16; }
  #traj-3d-canvas:active { cursor: grabbing; }
  .fps-badge { position: absolute; top: 8px; right: 10px; background: rgba(0,0,0,0.75); color: #4ade80; padding: 2px 7px; border-radius: 5px; font-size: 11px; font-family: monospace; font-weight: 700; }
  .tip-banner { padding: 7px 12px; background: #0f172a; font-size: 11.5px; color: #93c5fd; border-top: 1px solid #1e293b; display: flex; justify-content: space-between; align-items: center; gap: 6px; flex-wrap: wrap; }
  .sub-cams { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 10px; }
  .box { background: #0f172a; border: 1px solid #1e293b; border-radius: 8px; padding: 8px 10px; font-size: 12px; }
  .row-between { display: flex; justify-content: space-between; align-items: center; gap: 6px; }
  .btn-row { display: flex; gap: 6px; flex-wrap: wrap; }
  .btn-row > button { flex: 1; min-width: 72px; }
  .mono { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-weight: 600; color: #38bdf8; }
  button { border: none; padding: 8px 11px; border-radius: 7px; font-size: 12.5px; font-weight: 600; cursor: pointer; color: #fff; transition: 0.15s; }
  button:disabled { opacity: 0.4; cursor: not-allowed; }
  button:hover:not(:disabled) { filter: brightness(1.12); }
  .btn-green { background: linear-gradient(135deg, #16a34a, #15803d); }
  .btn-blue { background: linear-gradient(135deg, #2563eb, #1d4ed8); }
  .btn-amber { background: #d97706; }
  .btn-purple { background: linear-gradient(135deg, #7c3aed, #6d28d9); }
  .btn-red { background: #dc2626; }
  .btn-slate { background: #475569; }
  .btn-sm { padding: 4px 8px; font-size: 11px; }
  .log-box { background: #020617; border: 1px solid #1e293b; border-radius: 8px; padding: 8px 10px; font-family: monospace; font-size: 11px; line-height: 1.45; color: #cbd5e1; height: 145px; max-height: 145px; min-height: 145px; overflow-y: auto; white-space: pre-wrap; flex: none; }
  table { width: 100%; border-collapse: collapse; font-size: 10.5px; }
  th, td { border-bottom: 1px solid #1e293b; padding: 3px; text-align: left; }
  input[type="range"] { width: 100%; }
  input[type="text"] { background: #1e293b; color: #f8fafc; border: 1px solid #334155; border-radius: 5px; padding: 4px 7px; font-size: 11.5px; }
  .has-tip { position: relative; cursor: help; border-bottom: 1px dotted #38bdf8; display: inline-flex; align-items: center; }
  .has-tip:hover::after {
    content: attr(data-tip);
    position: absolute;
    bottom: 125%;
    left: 50%;
    transform: translateX(-50%);
    background: #0f172a;
    color: #e2e8f0;
    padding: 7px 10px;
    border-radius: 6px;
    font-size: 11px;
    line-height: 1.45;
    white-space: normal;
    width: max-content;
    max-width: 230px;
    z-index: 9999;
    box-shadow: 0 4px 14px rgba(0,0,0,0.6);
    border: 1px solid #38bdf8;
    pointer-events: none;
    font-weight: normal;
    text-align: left;
  }
  .has-tip:hover::before {
    content: '';
    position: absolute;
    bottom: 115%;
    left: 50%;
    transform: translateX(-50%);
    border-width: 5px;
    border-style: solid;
    border-color: #38bdf8 transparent transparent transparent;
    z-index: 10000;
  }
  /* 靠左对齐：防止最左侧文字被遮挡 */
  .has-tip.tip-left:hover::after {
    left: 0;
    transform: none;
  }
  .has-tip.tip-left:hover::before {
    left: 18px;
    transform: none;
  }
  /* 靠右对齐：防止最右侧文字被遮挡 */
  .has-tip.tip-right:hover::after {
    left: auto;
    right: 0;
    transform: none;
  }
  .has-tip.tip-right:hover::before {
    left: auto;
    right: 18px;
    transform: none;
  }
</style>
</head>
<body>
<div class="container">
  <header>
    <h1>🦾 203实验室 PIPER 双臂协同 · 三栏控制与 3D 质点轨迹总控台</h1>
    <div class="hdr-badges">
      <span class="pill err" id="badge-cam">📷 相机: 检测中</span>
      <span class="pill err" id="badge-a">🦾 A臂: 检测中</span>
      <span class="pill err" id="badge-b">🦾 B臂: 检测中</span>
      <button class="btn-green btn-sm" onclick="postApi('/api/start_camera')">📷 启动/重启主相机</button>
    </div>
  </header>

  <!-- 上排视觉区：左侧相机组 (俯视主相机 + 预留手眼一体 + Windows副视角)，右侧 3D 质点轨迹 -->
  <div class="top-visuals">
    <div>
      <div class="card">
        <div class="card-hdr">
          <span>📷 主视角：RealSense D435 全局俯视 (点击桌面选定搬运点)</span>
          <span style="color:#94a3b8; font-size:11px;">双臂 AR 叠加 · 2x 超分感知</span>
        </div>
        <div class="canvas-wrapper">
          <canvas id="live-canvas" width="640" height="480"></canvas>
          <div class="fps-badge" id="fps-indicator">SYNC...</div>
        </div>
        <div class="tip-banner">
          <span id="st-click" style="color:#facc15;">💡 左键点击绿桌面选点 · 右键或按右侧按钮清除 🎯</span>
          <button class="btn-red btn-sm" id="btn-clear-pt" onclick="postApi('/api/clear_point')" style="display:none;">❌ 消除目标点</button>
          <button class="btn-slate btn-sm" id="btn-clear-pt-idle" onclick="postApi('/api/clear_point')">消除目标点</button>
        </div>
      </div>

      <div class="sub-cams">
        <!-- 预留：手眼一体摄像头 -->
        <div class="card">
          <div class="card-hdr" style="padding:7px 10px;">
            <span style="font-size:12px; color:#38bdf8;">📷 手眼一体相机 (Eye-in-Hand · 预留)</span>
            <span class="pill warn" id="handcam-pill" style="padding:2px 7px; font-size:10.5px;">待接入</span>
          </div>
          <div class="canvas-wrapper" style="min-height:165px; align-items:center;">
            <img id="hand-cam-img" src="/api/hand_cam_frame.jpg" style="width:100%; height:165px; object-fit:contain; background:#050b16;" />
          </div>
          <div class="tip-banner" style="padding:5px 8px; font-size:11px;">
            <span>话题: <code style="color:#93c5fd;">/hand_camera/color/image_raw</code></span>
            <button class="btn-slate btn-sm" onclick="cfgHandCam()">绑定源</button>
          </div>
        </div>

        <!-- Windows 辅助视角摄像头 -->
        <div class="card">
          <div class="card-hdr" style="padding:7px 10px;">
            <span style="font-size:12px; color:#4ade80;">🎥 Windows 辅助侧视摄像头</span>
            <button class="btn-slate btn-sm" id="btn-wincam" onclick="toggleWinCam()">开启</button>
          </div>
          <div class="canvas-wrapper" style="min-height:165px; align-items:center; justify-content:center;">
            <img id="win-cam-img" src="" style="display:none; width:100%; height:165px; object-fit:contain;" onerror="onWinCamErr()" />
            <div id="win-cam-ph" style="color:#64748b; font-size:11.5px; text-align:center; padding:12px;">点击右上角【开启】加载本机辅助视角<br>(端口 8082)</div>
          </div>
          <div class="tip-banner" style="padding:5px 8px; font-size:11px;">
            <span id="win-cam-msg">推流源: 127.0.0.1:8082</span>
          </div>
        </div>
      </div>
    </div>

    <!-- 右侧 3D 质点轨迹界面 -->
    <div class="card">
      <div class="card-hdr">
        <span>🧊 3D 桌面与方块质点轨迹 (Gazebo视角风格)</span>
        <div style="display:flex; gap:5px;">
          <button class="btn-slate btn-sm" onclick="viewer3d.resetView()" title="快捷键: R 或 Home">🔄 透视(R)</button>
          <button class="btn-slate btn-sm" onclick="viewer3d.setTopView()" title="快捷键: T">⬇️ 俯视(T)</button>
          <button class="btn-slate btn-sm" onclick="viewer3d.setFrontView()" title="快捷键: F">👁️ 正视(F)</button>
          <button class="btn-slate btn-sm" onclick="viewer3d.setSideView()" title="快捷键: S">➡️ 侧视(S)</button>
          <button class="btn-red btn-sm" onclick="postApi('/api/clear_trajectory')">清空轨迹</button>
        </div>
      </div>
      <canvas id="traj-3d-canvas" width="640" height="440" style="flex:1;"></canvas>
      <div class="tip-banner">
        <span id="traj-3d-info" class="mono">等待方块识别...</span>
        <span style="color:#38bdf8; font-size:11px;">🖱️ Gazebo操作: 左键旋转 · 中键平移 · 右键/滚轮缩放</span>
      </div>
      <div class="box" style="margin:10px; border-color:#1e293b;">
        <div class="row-between"><span>全局 AprilTag 检测:</span><span class="mono" id="st-tag">--</span></div>
        <div class="row-between" style="margin-top:4px;"><span>相机系坐标 (mm):</span><span class="mono" id="st-tag-cam">--</span></div>
      </div>
    </div>
  </div>

  <!-- 下排核心控制区：A 臂一栏 | B 臂一栏 | 双臂配合一栏 -->
  <div class="three-cols">
    <!-- ==================== 第一栏：机械臂 A ==================== -->
    <div class="card" style="border-top:3px solid #2563eb;">
      <div class="card-hdr" style="color:#60a5fa;">
        <span>🟦 机械臂 A 专属控制台</span>
        <span class="mono" id="a-cal-pill" style="font-size:11px;">未标定</span>
      </div>
      <div class="col-body">
        <div class="box">
          <div class="row-between"><span>硬件状态:</span><span class="mono" id="a-hw-st">--</span></div>
          <div class="row-between" style="margin-top:3px;"><span>方块 A 系坐标:</span><span class="mono" id="a-tag-pos">--</span></div>
          <div class="row-between" style="margin-top:3px;"><span>点击点评估 / 夹爪:</span><span class="mono" id="a-click-grip" style="font-size:11px;">--</span></div>
        </div>

        <div class="btn-row">
          <button class="btn-amber btn-sm" onclick="postApi('/api/enable_arm',{arm_id:'arm_a'})">⚡ 使能</button>
          <button class="btn-blue btn-sm" onclick="postApi('/api/wiggle_arm',{arm_id:'arm_a'})">👋 轻摇认臂</button>
          <button class="btn-purple btn-sm" onclick="postApi('/api/arm_teach_mode',{arm_id:'arm_a',enable:true})">🔓 示教</button>
          <button class="btn-slate btn-sm" onclick="postApi('/api/arm_teach_mode',{arm_id:'arm_a',enable:false})">🔒 锁臂</button>
        </div>

        <div class="box">
          <div class="row-between">
            <span class="has-tip tip-left" data-tip="【下探深度】指尖相对方块顶(AprilTag面)向下垂直深入量。默认-25.5mm深入至方块腰部中心；过浅夹不稳易脱落，过深会擦碰桌面。">⬇️ 下探: <b id="a-v-z">-25.5</b>mm</span>
            <span class="has-tip" data-tip="【X轴微调】夹爪抓取中心沿机械臂基座X轴前后补偿量(mm)。正值向前推，负值向后拉，用于补偿相机轻微安装偏差。">↔️ X偏: <b id="a-v-x">0</b>mm</span>
            <span class="has-tip tip-right" data-tip="【Y轴微调】夹爪抓取中心沿机械臂基座Y轴左右补偿量(mm)。正值向左，负值向右，保证夹爪对称两指居中跨入方块两侧。">↕️ Y偏: <b id="a-v-y">0</b>mm</span>
          </div>
          <div style="display:flex; gap:6px; margin:4px 0;">
            <input type="range" id="a-z" min="-32" max="-10" step="0.5" value="-25.5" onchange="saveArmParams('arm_a')" oninput="document.getElementById('a-v-z').innerText=this.value">
            <input type="range" id="a-x" min="-15" max="15" step="0.5" value="0" onchange="saveArmParams('arm_a')" oninput="document.getElementById('a-v-x').innerText=this.value">
            <input type="range" id="a-y" min="-15" max="15" step="0.5" value="0" onchange="saveArmParams('arm_a')" oninput="document.getElementById('a-v-y').innerText=this.value">
          </div>
          <div class="row-between">
            <span class="has-tip tip-left" data-tip="【抓持力矩】夹爪闭合抱紧方块的最大电机力矩(mN·m)。默认350mN·m(0.35N·m)。过小方块滑脱，过大(>600)易压损方块或电机过流。">🤏 力矩: <b id="a-v-e">350</b> mN·m</span>
            <span class="has-tip tip-right" data-tip="【抓取开度】夹爪闭合抓取时的目标物理间距(mm)。方块边长25mm，默认设26.5mm，配合限矩模式实现无损柔顺包夹。">📏 开度: <b id="a-v-w">26.5</b> mm</span>
          </div>
          <div style="display:flex; gap:6px; margin-top:4px;">
            <input type="range" id="a-e" min="200" max="1000" step="50" value="350" onchange="saveArmParams('arm_a')" oninput="document.getElementById('a-v-e').innerText=this.value">
            <input type="range" id="a-w" min="20" max="29" step="0.5" value="26.5" onchange="saveArmParams('arm_a')" oninput="document.getElementById('a-v-w').innerText=this.value">
          </div>
          <div class="row-between" style="margin-top:6px; border-top:1px dashed #1e293b; padding-top:4px;">
            <span class="has-tip tip-left" data-tip="【运行速度】控制机械臂所有抓放与过渡轨迹的运动速率。默认100%标准速度，可调慢至20%方便细致观察，或调至150%高速。">⚡ 抓放速度: <b id="a-v-spd">100</b>%</span>
            <span class="mono" id="a-spd-tag" style="font-size:11px; color:#38bdf8;">标准</span>
          </div>
          <input type="range" id="a-spd" min="20" max="150" step="5" value="100" style="margin-top:2px;" onchange="saveArmParams('arm_a')" oninput="document.getElementById('a-v-spd').innerText=this.value; document.getElementById('a-spd-tag').innerText=(this.value<70?'慢速':(this.value>110?'高速':'标准'))">
        </div>

        <button class="btn-green" id="a-btn-ret" onclick="postApi('/api/pick_and_return',{arm_mode:'arm_a'})">🤖 A臂 原地抓取并放回</button>
        <div class="btn-row">
          <button class="btn-blue" id="a-btn-pl" onclick="postApi('/api/pick_and_place',{arm_mode:'arm_a'})">📍 A臂 搬至点击点</button>
          <button class="btn-slate" id="a-btn-hm" onclick="postApi('/api/retract_home',{arm_id:'arm_a'})">🏠 A臂 回原点</button>
        </div>

        <!-- A臂在线标定区 -->
        <div class="box" style="border-color:#1e3a8a;">
          <div class="row-between" style="margin-bottom:5px;">
            <b style="color:#60a5fa;">📐 A臂 6点手眼标定</b>
            <span class="mono" id="a-cal-rmse" style="font-size:11px;">点数:0</span>
          </div>
          <div class="btn-row" style="margin-bottom:5px;">
            <button class="btn-blue btn-sm" onclick="capCalib('arm_a')">📸 1.锁方块</button>
            <button class="btn-purple btn-sm" onclick="recCalib('arm_a')">📍 2.记A臂尖</button>
            <button class="btn-green btn-sm" onclick="saveCalib('arm_a')">💾 保存A外参</button>
            <button class="btn-red btn-sm" onclick="clrCalib('arm_a')">清</button>
          </div>
          <div style="font-size:11px; color:#94a3b8;" id="a-cal-pending">点击【1.锁方块】开始采样</div>
          <div style="max-height:95px; overflow-y:auto; margin-top:4px;">
            <table><thead><tr><th>#</th><th>相机</th><th>A臂尖</th><th>误差</th><th></th></tr></thead><tbody id="a-cal-tb"></tbody></table>
          </div>
        </div>
      </div>
    </div>

    <!-- ==================== 第二栏：机械臂 B ==================== -->
    <div class="card" style="border-top:3px solid #9333ea;">
      <div class="card-hdr" style="color:#c084fc;">
        <span>🟪 机械臂 B 专属控制台</span>
        <span class="mono" id="b-cal-pill" style="font-size:11px;">未标定</span>
      </div>
      <div class="col-body">
        <div class="box">
          <div class="row-between"><span>硬件状态:</span><span class="mono" id="b-hw-st">--</span></div>
          <div class="row-between" style="margin-top:3px;"><span>方块 B 系坐标:</span><span class="mono" id="b-tag-pos">--</span></div>
          <div class="row-between" style="margin-top:3px;"><span>点击点评估 / 夹爪:</span><span class="mono" id="b-click-grip" style="font-size:11px;">--</span></div>
        </div>

        <div class="btn-row">
          <button class="btn-amber btn-sm" onclick="postApi('/api/enable_arm',{arm_id:'arm_b'})">⚡ 使能</button>
          <button class="btn-purple btn-sm" onclick="postApi('/api/wiggle_arm',{arm_id:'arm_b'})">👋 轻摇认臂</button>
          <button class="btn-blue btn-sm" onclick="postApi('/api/arm_teach_mode',{arm_id:'arm_b',enable:true})">🔓 示教</button>
          <button class="btn-slate btn-sm" onclick="postApi('/api/arm_teach_mode',{arm_id:'arm_b',enable:false})">🔒 锁臂</button>
        </div>

        <div class="box">
          <div class="row-between">
            <span class="has-tip tip-left" data-tip="【下探深度】指尖相对方块顶(AprilTag面)向下垂直深入量。默认-25.5mm深入至方块腰部中心；过浅夹不稳易脱落，过深会擦碰桌面。">⬇️ 下探: <b id="b-v-z">-25.5</b>mm</span>
            <span class="has-tip" data-tip="【X轴微调】夹爪抓取中心沿机械臂基座X轴前后补偿量(mm)。正值向前推，负值向后拉，用于补偿相机轻微安装偏差。">↔️ X偏: <b id="b-v-x">0</b>mm</span>
            <span class="has-tip tip-right" data-tip="【Y轴微调】夹爪抓取中心沿机械臂基座Y轴左右补偿量(mm)。正值向左，负值向右，保证夹爪对称两指居中跨入方块两侧。">↕️ Y偏: <b id="b-v-y">0</b>mm</span>
          </div>
          <div style="display:flex; gap:6px; margin:4px 0;">
            <input type="range" id="b-z" min="-32" max="-10" step="0.5" value="-25.5" onchange="saveArmParams('arm_b')" oninput="document.getElementById('b-v-z').innerText=this.value">
            <input type="range" id="b-x" min="-15" max="15" step="0.5" value="0" onchange="saveArmParams('arm_b')" oninput="document.getElementById('b-v-x').innerText=this.value">
            <input type="range" id="b-y" min="-15" max="15" step="0.5" value="0" onchange="saveArmParams('arm_b')" oninput="document.getElementById('b-v-y').innerText=this.value">
          </div>
          <div class="row-between">
            <span class="has-tip tip-left" data-tip="【抓持力矩】夹爪闭合抱紧方块的最大电机力矩(mN·m)。默认350mN·m(0.35N·m)。过小方块滑脱，过大(>600)易压损方块或电机过流。">🤏 力矩: <b id="b-v-e">350</b> mN·m</span>
            <span class="has-tip tip-right" data-tip="【抓取开度】夹爪闭合抓取时的目标物理间距(mm)。方块边长25mm，默认设26.5mm，配合限矩模式实现无损柔顺包夹。">📏 开度: <b id="b-v-w">26.5</b> mm</span>
          </div>
          <div style="display:flex; gap:6px; margin-top:4px;">
            <input type="range" id="b-e" min="200" max="1000" step="50" value="350" onchange="saveArmParams('arm_b')" oninput="document.getElementById('b-v-e').innerText=this.value">
            <input type="range" id="b-w" min="20" max="29" step="0.5" value="26.5" onchange="saveArmParams('arm_b')" oninput="document.getElementById('b-v-w').innerText=this.value">
          </div>
          <div class="row-between" style="margin-top:6px; border-top:1px dashed #1e293b; padding-top:4px;">
            <span class="has-tip tip-left" data-tip="【运行速度】控制机械臂所有抓放与过渡轨迹的运动速率。默认100%标准速度，可调慢至20%方便细致观察，或调至150%高速。">⚡ 抓放速度: <b id="b-v-spd">100</b>%</span>
            <span class="mono" id="b-spd-tag" style="font-size:11px; color:#c084fc;">标准</span>
          </div>
          <input type="range" id="b-spd" min="20" max="150" step="5" value="100" style="margin-top:2px;" onchange="saveArmParams('arm_b')" oninput="document.getElementById('b-v-spd').innerText=this.value; document.getElementById('b-spd-tag').innerText=(this.value<70?'慢速':(this.value>110?'高速':'标准'))">
        </div>

        <button class="btn-green" id="b-btn-ret" onclick="postApi('/api/pick_and_return',{arm_mode:'arm_b'})">🤖 B臂 原地抓取并放回</button>
        <div class="btn-row">
          <button class="btn-purple" id="b-btn-pl" onclick="postApi('/api/pick_and_place',{arm_mode:'arm_b'})">📍 B臂 搬至点击点</button>
          <button class="btn-slate" id="b-btn-hm" onclick="postApi('/api/retract_home',{arm_id:'arm_b'})">🏠 B臂 回原点</button>
        </div>

        <!-- B臂在线标定区 -->
        <div class="box" style="border-color:#581c87;">
          <div class="row-between" style="margin-bottom:5px;">
            <b style="color:#c084fc;">📐 B臂 6点手眼标定</b>
            <span class="mono" id="b-cal-rmse" style="font-size:11px;">点数:0</span>
          </div>
          <div class="btn-row" style="margin-bottom:5px;">
            <button class="btn-blue btn-sm" onclick="capCalib('arm_b')">📸 1.锁方块</button>
            <button class="btn-purple btn-sm" onclick="recCalib('arm_b')">📍 2.记B臂尖</button>
            <button class="btn-green btn-sm" onclick="saveCalib('arm_b')">💾 保存B外参</button>
            <button class="btn-red btn-sm" onclick="clrCalib('arm_b')">清</button>
          </div>
          <div style="font-size:11px; color:#94a3b8;" id="b-cal-pending">点击【1.锁方块】开始采样</div>
          <div style="max-height:95px; overflow-y:auto; margin-top:4px;">
            <table><thead><tr><th>#</th><th>相机</th><th>B臂尖</th><th>误差</th><th></th></tr></thead><tbody id="b-cal-tb"></tbody></table>
          </div>
        </div>
      </div>
    </div>

    <!-- ==================== 第三栏：双臂配合与系统中心 ==================== -->
    <div class="card" style="border-top:3px solid #16a34a;">
      <div class="card-hdr" style="color:#4ade80;">
        <span>🤝 双臂配合与协同调度中心</span>
        <span class="mono" id="dual-tf-pill" style="font-size:11px;">等待标定</span>
      </div>
      <div class="col-body">
        <div class="box">
          <div style="font-weight:700; color:#4ade80; margin-bottom:4px;">🔗 协同空间变换与 CAN 总线诊断</div>
          <div id="hw-can-list" style="font-size:11px; color:#cbd5e1;">--</div>
          <div id="hw-can-hint" style="font-size:11px; color:#facc15; margin-top:4px;"></div>
        </div>

        <div class="btn-row">
          <button class="btn-amber btn-sm" onclick="postApi('/api/swap_ab')">🔄 互换 A/B 硬件绑定</button>
          <button class="btn-slate btn-sm" onclick="postApi('/api/reconnect_arms')">🔌 重扫 CAN 总线</button>
        </div>

        <!-- 双方块堆叠与空间互换专区 -->
        <div class="box" style="border:1.5px solid #10b981; background:rgba(6,78,59,0.18); border-radius:8px; padding:9px; margin-top:6px;">
          <div class="row-between" style="margin-bottom:5px;">
            <b style="color:#34d399; font-size:12.5px;">🧱 双方块堆叠与空间互换</b>
            <span class="mono" id="swap-step-pill" style="font-size:10.5px; padding:2px 7px; border-radius:4px; background:#047857; color:#ecfdf5;">待命</span>
          </div>

          <div style="font-size:11px; color:#cbd5e1; line-height:1.5; margin-bottom:7px;">
            <div class="row-between"><span>左侧方块 A:</span><span class="mono" id="swap-cube-a" style="color:#38bdf8;">未检测</span></div>
            <div class="row-between"><span>右侧方块 B:</span><span class="mono" id="swap-cube-b" style="color:#c084fc;">未检测</span></div>
            <div class="row-between" style="margin-top:2px;"><span>流程进度:</span><span id="swap-status-txt" style="color:#facc15; font-size:10.5px;">待命 (请在左侧放A，右侧放B)</span></div>
          </div>

          <button class="btn-green" style="background:#059669; font-weight:700; width:100%; margin-bottom:6px;" onclick="startSwap('all')">🚀 双方块堆叠与空间互换 (一键全流程)</button>

          <div class="btn-row" style="margin-bottom:6px;">
            <button class="btn-blue btn-sm" onclick="startSwap(1)" title="机械臂A抓取方块A送至公共交接点底座，A避让让位">① A送底座</button>
            <button class="btn-purple btn-sm" onclick="startSwap(2)" title="机械臂B抓取方块B堆叠在方块A上方(+25mm)，B避让让位">② B叠顶层</button>
            <button class="btn-blue btn-sm" onclick="startSwap(3)" title="机械臂A取下顶层方块B(+25mm)并送回方块A初始位，A避让让位">③ A取顶归位</button>
            <button class="btn-purple btn-sm" onclick="startSwap(4)" title="机械臂B取下底层方块A并送回方块B初始位，B归位">④ B取底归位</button>
          </div>

          <div class="btn-row">
            <button class="btn-red btn-sm" style="flex:1;" onclick="postApi('/api/workflow/abort')">🛑 紧急停止</button>
            <button class="btn-slate btn-sm" style="flex:1;" onclick="postApi('/api/workflow/reset_anchors')">🔄 重置记忆点</button>
          </div>
        </div>

        <button class="btn-green" id="c-btn-auto" onclick="postApi('/api/pick_and_return',{arm_mode:'auto'})">🤖 智能择近臂 · 原地抓放</button>
        <button class="btn-blue" id="c-btn-autopl" onclick="postApi('/api/pick_and_place',{arm_mode:'auto'})">📍 智能调度 · 搬至点击点 (超距自动接力)</button>
        <button class="btn-purple" id="c-btn-relay" onclick="postApi('/api/pick_and_place',{arm_mode:'relay'})">🤝 双臂协同接力 (源臂→中继交接→另一臂)</button>
        <button class="btn-slate" id="c-btn-home" onclick="postApi('/api/retract_home',{arm_id:'all'})">🏠 双臂同步安全回原点</button>

        <div style="font-size:11.5px; color:#94a3b8; font-weight:700; margin-top:2px;">双臂运行实时日志 (最近12条):</div>
        <div class="log-box" id="log-box">等待指令...</div>
      </div>
    </div>
  </div>
</div>

<script>
''' + TRAJ_3D_SCRIPT + r'''
  const viewer3d = new Trajectory3DViewer('traj-3d-canvas', 'traj-3d-info', { center: [240, -315, 0], gridW: 540, gridH: 820 });
  let centerInitialized = false;
  const canvas = document.getElementById('live-canvas');
  const ctx = canvas.getContext('2d');
  const fpsBadge = document.getElementById('fps-indicator');
  let isFetching = false, frameCount = 0, lastFpsTime = performance.now(), slidersSynced = false;

  function loadNextFrame() {
    if (isFetching) return;
    isFetching = true;
    const img = new Image();
    img.onload = () => {
      ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
      isFetching = false; frameCount++;
      const now = performance.now();
      if (now - lastFpsTime >= 1000) {
        fpsBadge.innerText = 'LIVE: ' + Math.round((frameCount * 1000) / (now - lastFpsTime)) + ' FPS';
        frameCount = 0; lastFpsTime = now;
      }
      requestAnimationFrame(loadNextFrame);
    };
    img.onerror = () => { isFetching = false; setTimeout(loadNextFrame, 250); };
    img.src = '/api/latest_frame.jpg?t=' + performance.now();
  }
  document.addEventListener('DOMContentLoaded', () => {
    loadNextFrame();
    setInterval(() => {
      const hImg = document.getElementById('hand-cam-img');
      if (hImg) hImg.src = '/api/hand_cam_frame.jpg?t=' + Date.now();
    }, 350);
  });

  canvas.addEventListener('click', (e) => {
    const r = canvas.getBoundingClientRect();
    postApi('/api/select_point', {
      u: Math.round((e.clientX - r.left) * (640 / r.width)),
      v: Math.round((e.clientY - r.top) * (480 / r.height))
    });
  });
  canvas.addEventListener('contextmenu', (e) => {
    e.preventDefault();
    postApi('/api/clear_point');
  });

  function postApi(url, body = {}) {
    return fetch(url, { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify(body) })
      .then(r => r.json()).then(d => { if (d.status === 'error') alert(d.msg); updateStatus(); return d; });
  }

  function saveArmParams(aid) {
    const p = aid === 'arm_a' ? 'a' : 'b';
    postApi('/api/set_gripper', {
      arm_id: aid,
      z_offset_mm: document.getElementById(p + '-z').value,
      x_offset_mm: document.getElementById(p + '-x').value,
      y_offset_mm: document.getElementById(p + '-y').value,
      effort: document.getElementById(p + '-e').value,
      width_mm: document.getElementById(p + '-w').value,
      speed_pct: document.getElementById(p + '-spd').value
    });
  }

  function capCalib(aid) { postApi('/api/calib/capture_cam', {arm_id: aid}); }
  function recCalib(aid) { postApi('/api/calib/record_arm', {arm_id: aid}); }
  function saveCalib(aid) { postApi('/api/calib/solve_and_save', {arm_id: aid}); }
  function clrCalib(aid) { postApi('/api/calib/clear_points', {arm_id: aid}); }
  function delCalib(aid, idx) { postApi('/api/calib/delete_point', {arm_id: aid, idx}); }

  function cfgHandCam() {
    const url = prompt('请输入后续手眼一体摄像头 ROS 2 话题名或 HTTP MJPEG 地址 (当前预留默认 /hand_camera/color/image_raw):', '/hand_camera/color/image_raw');
    if (url) postApi('/api/set_hand_cam_source', {source: url});
  }

  let winCamOn = false;
  function toggleWinCam() {
    winCamOn = !winCamOn;
    const img = document.getElementById('win-cam-img'), ph = document.getElementById('win-cam-ph'), btn = document.getElementById('btn-wincam');
    if (winCamOn) {
      img.style.display = 'block'; ph.style.display = 'none';
      img.src = 'http://127.0.0.1:8082/stream?t=' + Date.now();
      document.getElementById('win-cam-msg').innerText = '已连接: http://127.0.0.1:8082/stream';
      btn.innerText = '关闭'; btn.style.background = '#dc2626';
    } else {
      img.style.display = 'none'; ph.style.display = 'block'; img.src = '';
      btn.innerText = '开启'; btn.style.background = '#475569';
    }
  }
  function onWinCamErr() {
    if (!winCamOn) return;
    document.getElementById('win-cam-msg').innerHTML = '<span style="color:#f87171;">⚠️ Windows 8082 服务未启动，请双击 start_win_camera.bat</span>';
  }

  function formatArmPill(rt, arm, ch) {
    if (rt.motors_alive) return { txt: `🦾 ${ch.toUpperCase()}臂: 在线(${rt.ctrl_mode===2 ? '示教' : '伺服'})`, cls: 'pill ok' };
    if (rt.logic_alive) return { txt: `🦾 ${ch.toUpperCase()}臂: 主控在线/无状态流`, cls: 'pill warn' };
    if (rt.can_up) return { txt: `🦾 ${ch.toUpperCase()}臂: 待上电`, cls: 'pill warn' };
    return { txt: `🦾 ${ch.toUpperCase()}臂: CAN离线`, cls: 'pill err' };
  }

  function updateStatus() {
    fetch('/api/status?t=' + Date.now()).then(r => r.json()).then(d => {
      const bCam = document.getElementById('badge-cam');
      bCam.innerText = d.camera_online ? '📷 主相机: 在线' : '📷 主相机: 未启动';
      bCam.className = 'pill ' + (d.camera_online ? 'ok' : 'err');

      const hcPill = document.getElementById('handcam-pill');
      hcPill.innerText = d.hand_cam_online ? '在线' : '待接入(预留)';
      hcPill.className = 'pill ' + (d.hand_cam_online ? 'ok' : 'warn');

      ['a', 'b'].forEach(ch => {
        const aid = 'arm_' + ch, arm = d.arms[aid], rt = arm.runtime;
        const pInfo = formatArmPill(rt, arm, ch);
        const b = document.getElementById('badge-' + ch);
        b.innerText = pInfo.txt; b.className = pInfo.cls;

        document.getElementById(ch + '-cal-pill').innerText = arm.calibrated ? `✅ 已标定 (${arm.calib_rmse_mm}mm)` : '⚠️ 未标定';
        document.getElementById(ch + '-hw-st').innerText =
          `${rt.can_port} | ${rt.motors_alive ? (rt.ctrl_mode===2 ? '🔓示教' : '⚡伺服就绪') : (rt.logic_alive ? '⚠️仅0x4AF应答/无2A1状态流' : '🔴离线')}`;
        const tpos = ch === 'a' ? d.tag_arm_a_mm : d.tag_arm_b_mm;
        document.getElementById(ch + '-tag-pos').innerText = tpos ? `[${tpos.join(', ')}] mm` : '--';
        const ev = (d.clicked_eval && d.clicked_eval[aid]) ? d.clicked_eval[aid] : null;
        document.getElementById(ch + '-click-grip').innerText =
          `${ev ? (ev.valid ? '✅'+ev.msg : '❌'+ev.msg) : '未选点'} | 夹爪=${rt.gripper_mm}mm`;

        if (!slidersSynced) {
          document.getElementById(ch + '-z').value = arm.grasp_z_offset_mm;
          document.getElementById(ch + '-v-z').innerText = arm.grasp_z_offset_mm;
          document.getElementById(ch + '-x').value = arm.grasp_x_offset_mm;
          document.getElementById(ch + '-v-x').innerText = arm.grasp_x_offset_mm;
          const yv = arm.grasp_y_offset_mm || 0;
          document.getElementById(ch + '-y').value = yv;
          document.getElementById(ch + '-v-y').innerText = yv;
          document.getElementById(ch + '-e').value = arm.grasp_effort;
          document.getElementById(ch + '-v-e').innerText = arm.grasp_effort;
          document.getElementById(ch + '-w').value = arm.grasp_target_mm;
          document.getElementById(ch + '-v-w').innerText = arm.grasp_target_mm;
          const spd = arm.speed_pct || 100;
          document.getElementById(ch + '-spd').value = spd;
          document.getElementById(ch + '-v-spd').innerText = spd;
          document.getElementById(ch + '-spd-tag').innerText = (spd < 70 ? '慢速' : (spd > 110 ? '高速' : '标准'));
        }

        // 每臂独立标定表
        const calData = (d.calib_arms && d.calib_arms[aid]) ? d.calib_arms[aid] : {points:[], pending_cam_mm:null, preview_rmse_mm:null};
        document.getElementById(ch + '-cal-rmse').innerText =
          calData.preview_rmse_mm !== null ? `RMSE:${calData.preview_rmse_mm}mm` : `点数:${calData.points.length}`;
        document.getElementById(ch + '-cal-pending').innerHTML = calData.pending_cam_mm
          ? `<span style="color:#4ade80;">✅ 已锁存 [${calData.pending_cam_mm.join(',')}] → 点记臂尖</span>` : '移开手臂点【1.锁方块】';
        document.getElementById(ch + '-cal-tb').innerHTML = calData.points.map((pt, idx) => `<tr>
          <td>C${pt.idx}</td><td>${pt.p_cam_mm.join(',')}</td><td>${pt.p_base_mm.join(',')}</td>
          <td>${pt.err_mm !== null ? pt.err_mm : '-'}</td>
          <td><button class="btn-red btn-sm" style="padding:1px 4px;" onclick="delCalib('${aid}',${idx})">删</button></td>
        </tr>`).join('');

        document.getElementById(ch + '-btn-ret').disabled = d.task_busy || !d.tag_detected;
        document.getElementById(ch + '-btn-pl').disabled = d.task_busy || !d.tag_detected || !d.clicked_uv;
        document.getElementById(ch + '-btn-hm').disabled = d.task_busy;
      });
      slidersSynced = true;

      // 公共感知 & 第三栏状态
      const stTag = document.getElementById('st-tag');
      stTag.innerText = d.tag_detected ? `已锁定 (ID:${d.tag_id})` : '未检测到方块';
      stTag.style.color = d.tag_detected ? '#22c55e' : '#f97316';
      document.getElementById('st-tag-cam').innerText = d.tag_cam_mm ? `[${d.tag_cam_mm.join(', ')}]` : '--';

      if (d.clicked_eval && Object.keys(d.clicked_eval).length > 0) {
        const ea = d.clicked_eval.arm_a, eb = d.clicked_eval.arm_b;
        document.getElementById('st-click').innerText =
          `选点评估 → A: ${ea.valid ? '✅'+ea.msg : '❌'+ea.msg} | B: ${eb.valid ? '✅'+eb.msg : '❌'+eb.msg}`;
      } else {
        document.getElementById('st-click').innerText = '💡 点击绿桌面任意空位评估双臂可达性 🎯';
      }

      document.getElementById('dual-tf-pill').innerText = d.has_dual_rel_tf ? '🤝 协同外参已就绪' : '待双臂标定';
      document.getElementById('hw-can-list').innerHTML = (d.raw_can_interfaces || []).map(c =>
        `${c.is_up ? '🟢' : '🔴'} <b>${c.iface}</b> | UP=${c.is_up} | ...${(c.serial||'').slice(-6)}`
      ).join('<br>') || '未检测到 CAN 适配器';

      const anyNoMotor = !d.arms.arm_a.runtime.motors_alive || !d.arms.arm_b.runtime.motors_alive;
      const anyLogic = d.arms.arm_a.runtime.logic_alive || d.arms.arm_b.runtime.logic_alive;
      document.getElementById('hw-can-hint').innerText = anyNoMotor
        ? (anyLogic ? '⚠️ 检测到 CAN 主控(0x4AF)已应答，但无 0x2A1 关节状态流（若确认24V已开，请看底座指示灯：绿灯常亮/红灯闪烁说明主控异常，需拔掉电源重插15秒；或查急停按钮/主从模式）'
                    : '⚠️ CAN 离线或机械臂未通电，请检查电源与 USB-CAN 接线')
        : (d.has_dual_rel_tf ? `🤝 双臂外参联调就绪 (A↔B 间距约 ${d.dual_dist_mm || '--'} mm)` : '请完成两臂标定以解锁协同搬运');

      if (d.traj_3d) {
        if (d.traj_3d.center) {
          viewer3d.defaultCenter = d.traj_3d.center;
          if (!centerInitialized) {
            viewer3d.center = [...d.traj_3d.center];
            centerInitialized = true;
          }
        }
        viewer3d.updateDualData(d.traj_3d.a, d.traj_3d.b);
      }

      // 双方块堆叠与空间互换专区数据渲染
      if (d.cubes) {
        const ca = d.cubes.A, cb = d.cubes.B;
        const aEl = document.getElementById('swap-cube-a');
        if (aEl) {
          aEl.innerText = ca.detected
            ? (ca.p_base_mm ? `A:[${ca.p_base_mm.join(',')}] mm` : `CAM:[${(ca.p_cam_mm||[]).join(',')}]`)
            : (ca.init_p_base_mm ? `🔒锁初位:[${ca.init_p_base_mm.join(',')}]` : '未识别');
          aEl.style.color = ca.detected ? '#38bdf8' : '#94a3b8';
        }
        const bEl = document.getElementById('swap-cube-b');
        if (bEl) {
          bEl.innerText = cb.detected
            ? (cb.p_base_mm ? `B:[${cb.p_base_mm.join(',')}] mm` : `CAM:[${(cb.p_cam_mm||[]).join(',')}]`)
            : (cb.init_p_base_mm ? `🔒锁初位:[${cb.init_p_base_mm.join(',')}]` : '未识别');
          bEl.style.color = cb.detected ? '#c084fc' : '#94a3b8';
        }
      }
      if (d.swap_workflow) {
        const sw = d.swap_workflow;
        const pill = document.getElementById('swap-step-pill');
        if (pill) {
          const lockMark = sw.anchors_locked ? '🔒 ' : '';
          pill.innerText = sw.busy ? `⏳ 执行中(${sw.step})` : (sw.step === 4 ? '🎉 互换完成' : `${lockMark}步进:${sw.step}/4`);
          pill.style.background = sw.busy ? '#d97706' : (sw.step === 4 ? '#059669' : (sw.anchors_locked ? '#1d4ed8' : '#334155'));
        }
        const txt = document.getElementById('swap-status-txt');
        if (txt) txt.innerText = sw.status || '--';
      }

      const bClr = document.getElementById('btn-clear-pt'), bClrIdle = document.getElementById('btn-clear-pt-idle');
      if (bClr && bClrIdle) {
        bClr.style.display = d.clicked_uv ? 'inline-block' : 'none';
        bClrIdle.style.display = d.clicked_uv ? 'none' : 'inline-block';
      }

      document.getElementById('c-btn-auto').disabled = d.task_busy || !d.tag_detected;
      document.getElementById('c-btn-autopl').disabled = d.task_busy || !d.tag_detected || !d.clicked_uv;
      document.getElementById('c-btn-relay').disabled = d.task_busy || !d.tag_detected || !d.clicked_uv;
      document.getElementById('c-btn-home').disabled = d.task_busy;

      const logBox = document.getElementById('log-box');
      if (d.logs && d.logs.length > 0) {
        logBox.innerText = d.logs.slice(-12).join('\n');
        logBox.scrollTop = logBox.scrollHeight;
      }
    }).catch(() => {});
  }
  setInterval(updateStatus, 250);

  function startSwap(step) {
    const title = step === 'all'
      ? '确定启动双方块【堆叠与空间互换】一键全流程吗？\n\n执行顺序:\n① 机械臂 A 抓取方块 A 送往公共底座并避让\n② 机械臂 B 抓取方块 B 堆叠在方块 A 顶面并避让\n③ 机械臂 A 取下顶层方块 B 送回方块 A 初始位并避让\n④ 机械臂 B 取下底层方块 A 送回方块 B 初始位并归位'
      : '确定执行双方块协同步骤 ' + step + ' 吗？';
    if (confirm(title)) {
      postApi('/api/workflow/swap_cubes', { step: step });
    }
  }
</script>
</body>
</html>
'''
