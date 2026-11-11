#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
纯单臂视觉抓取与在线手眼标定 Web 控制台前端模板 (用于单臂独立演示，无双臂/A-B/协同概念)
"""


def render_single_arm_html() -> str:
    return r'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>PIPER 机械臂 × RealSense 全自动视觉抓取控制台</title>
<style>
  * { box-sizing: border-box; }
  body { margin: 0; padding: 16px; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; background: #0b1120; color: #f8fafc; }
  .container { max-width: 1320px; margin: 0 auto; }
  header { display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 10px; border-bottom: 1px solid #1e293b; padding-bottom: 12px; margin-bottom: 16px; }
  h1 { margin: 0; font-size: 19px; color: #38bdf8; }
  .hdr-badges { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
  .pill { font-size: 12px; font-weight: 600; padding: 4px 10px; border-radius: 9999px; background: #1e293b; border: 1px solid #334155; }
  .pill.ok { background: #14532d; color: #86efac; border-color: #22c55e; }
  .pill.err { background: #450a0a; color: #fca5a5; border-color: #ef4444; }
  .grid { display: grid; grid-template-columns: 1.45fr 1.05fr; gap: 18px; }
  @media (max-width: 960px) { .grid { grid-template-columns: 1fr; } }
  .card { background: #111827; border: 1px solid #334155; border-radius: 12px; overflow: hidden; box-shadow: 0 10px 25px rgba(0,0,0,0.45); }
  .video-header { padding: 10px 14px; background: #1e293b; display: flex; justify-content: space-between; align-items: center; font-size: 13px; font-weight: 600; }
  .canvas-wrapper { position: relative; background: #000; display: flex; justify-content: center; }
  canvas { width: 100%; max-width: 640px; height: auto; aspect-ratio: 4/3; cursor: crosshair; display: block; }
  .fps-badge { position: absolute; top: 10px; right: 14px; background: rgba(0,0,0,0.75); color: #4ade80; padding: 3px 8px; border-radius: 6px; font-size: 11px; font-family: monospace; font-weight: 700; border: 1px solid rgba(74,222,128,0.4); }
  .tip-banner { padding: 8px 14px; background: #0f172a; font-size: 12px; color: #93c5fd; border-top: 1px solid #1e293b; display: flex; justify-content: space-between; align-items: center; }
  .tabs { display: flex; background: #0f172a; border-bottom: 1px solid #334155; }
  .tab-btn { flex: 1; padding: 11px 8px; background: transparent; border: none; color: #94a3b8; font-weight: 700; font-size: 13px; cursor: pointer; border-bottom: 3px solid transparent; border-radius: 0; }
  .tab-btn.active { color: #38bdf8; border-bottom-color: #38bdf8; background: #111827; }
  .tab-content { display: none; padding: 14px; flex-direction: column; gap: 10px; }
  .tab-content.active { display: flex; }
  .box { background: #0f172a; border: 1px solid #1e293b; border-radius: 8px; padding: 10px 12px; font-size: 12.5px; }
  .row-between { display: flex; justify-content: space-between; align-items: center; gap: 8px; }
  .mono { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-weight: 600; color: #38bdf8; }
  button { border: none; padding: 10px 14px; border-radius: 7px; font-size: 13.5px; font-weight: 600; cursor: pointer; color: #fff; transition: 0.15s; }
  button:disabled { opacity: 0.4; cursor: not-allowed; }
  button:hover:not(:disabled) { filter: brightness(1.12); }
  .btn-green { background: linear-gradient(135deg, #16a34a, #15803d); }
  .btn-blue { background: linear-gradient(135deg, #2563eb, #1d4ed8); }
  .btn-amber { background: #d97706; }
  .btn-purple { background: #7c3aed; }
  .btn-red { background: #dc2626; }
  .btn-slate { background: #475569; }
  .btn-sm { padding: 5px 10px; font-size: 12px; }
  .log-box { background: #020617; border: 1px solid #1e293b; border-radius: 8px; padding: 8px 10px; font-family: monospace; font-size: 11px; line-height: 1.5; color: #cbd5e1; height: 150px; overflow-y: auto; white-space: pre-wrap; }
  table { width: 100%; border-collapse: collapse; font-size: 11.5px; }
  th, td { border-bottom: 1px solid #1e293b; padding: 5px 4px; text-align: left; }
  th { color: #94a3b8; }
  select, input[type="range"] { background: #1e293b; color: #f8fafc; border: 1px solid #334155; border-radius: 6px; padding: 5px 8px; font-size: 12.5px; }
</style>
</head>
<body>
<div class="container">
  <header>
    <h1>🦾 203实验室 PIPER 机械臂视觉抓取与点击放置总控台</h1>
    <div class="hdr-badges">
      <span class="pill err" id="badge-cam">📷 相机: 检测中</span>
      <span class="pill err" id="badge-arm">🦾 机械臂: 检测中</span>
      <button class="btn-green btn-sm" onclick="startCamera()">📷 启动/重启摄像头</button>
    </div>
  </header>

  <div class="grid">
    <!-- 左侧主视频与副视角 -->
    <div style="display:flex; flex-direction:column; gap:12px;">
      <div class="card">
        <div class="video-header">
          <span>📷 RealSense D435 实时桌面视野 (鼠标直接点击选定放置目标)</span>
          <span style="color:#94a3b8; font-size:11.5px;">640x480 · 深度防抖三维对齐</span>
        </div>
        <div class="canvas-wrapper">
          <canvas id="live-canvas" width="640" height="480"></canvas>
          <div class="fps-badge" id="fps-indicator">SYNC...</div>
        </div>
        <div class="tip-banner">
          <span>💡 <b>交互提示</b>：用鼠标在画面空白绿色桌面上<b>点击一下</b>即可设定放置坐标准星 🎯</span>
          <button class="btn-slate btn-sm" onclick="clearPoint()">清除选点</button>
        </div>
      </div>

      <div class="card" style="padding:10px 14px;">
        <div class="row-between">
          <span style="font-size:13px; font-weight:600; color:#38bdf8;">🎥 本地辅助视角监视 (侧视补盲)</span>
          <button class="btn-slate btn-sm" id="btn-wincam" onclick="toggleWinCam()">开启副视角</button>
        </div>
        <div id="win-cam-box" style="display:none; margin-top:8px;">
          <img id="win-cam-img" style="width:100%; border-radius:6px; border:1px solid #334155;" src="" />
        </div>
      </div>
    </div>

    <!-- 右侧单臂双标签页 -->
    <div class="card" style="display:flex; flex-direction:column;">
      <div class="tabs">
        <button class="tab-btn active" onclick="switchTab('tab-ctrl', this)">🎮 视觉抓放操控</button>
        <button class="tab-btn" onclick="switchTab('tab-calib', this)">📐 在线手眼标定</button>
      </div>

      <!-- TAB 1: 视觉抓放操控 -->
      <div id="tab-ctrl" class="tab-content active">
        <div class="box row-between">
          <span>方块识别状态 (AprilTag):</span>
          <span class="mono" id="st-tag" style="color:#f97316;">搜寻中...</span>
        </div>
        <div class="box row-between">
          <span>方块基座坐标 (X, Y, Z):</span>
          <span class="mono" id="st-tag-pos">--</span>
        </div>
        <div class="box row-between">
          <span>夹爪实时物理开度:</span>
          <span class="mono" id="st-grip">0.0 mm</span>
        </div>
        <div class="box">
          <div style="color:#94a3b8; font-size:11.5px; margin-bottom:3px;">🎯 鼠标点击的目标放置点评估:</div>
          <div class="mono" id="st-click" style="font-size:12px; color:#facc15;">请点击左侧桌面空白位置</div>
        </div>

        <div class="box">
          <div class="row-between"><span>⬇️ 相对方块顶下探深度 (负值越深):</span><span class="mono" id="val-zoff">-25.5 mm</span></div>
          <input type="range" id="slider-zoff" style="width:100%;" min="-32" max="-10" step="0.5" value="-25.5" onchange="updateGripperParams()" oninput="document.getElementById('val-zoff').innerText=this.value+' mm'">
          <div class="row-between" style="margin-top:4px;"><span>↔️ X轴抓点补偿 (+前/-后):</span><span class="mono" id="val-xoff">0.0 mm</span></div>
          <input type="range" id="slider-xoff" style="width:100%;" min="-15" max="15" step="0.5" value="0.0" onchange="updateGripperParams()" oninput="document.getElementById('val-xoff').innerText=(this.value>0?'+':'')+this.value+' mm'">
          <div class="row-between" style="margin-top:4px;"><span>↕️ Y轴抓点补偿 (+左/-右):</span><span class="mono" id="val-yoff">0.0 mm</span></div>
          <input type="range" id="slider-yoff" style="width:100%;" min="-15" max="15" step="0.5" value="0.0" onchange="updateGripperParams()" oninput="document.getElementById('val-yoff').innerText=(this.value>0?'+':'')+this.value+' mm'">
          <div class="row-between" style="margin-top:4px;"><span>🤏 柔顺抓持力矩 / 闭合行程:</span><span class="mono" id="val-ew">350 mN·m / 26.5 mm</span></div>
          <div style="display:flex; gap:8px;">
            <input type="range" id="slider-effort" style="flex:1;" min="200" max="1000" step="50" value="350" onchange="updateGripperParams()" oninput="updEWLabel()">
            <input type="range" id="slider-width" style="flex:1;" min="20" max="29" step="0.5" value="26.5" onchange="updateGripperParams()" oninput="updEWLabel()">
          </div>
        </div>

        <button class="btn-green" id="btn-return" onclick="postApi('/api/pick_and_return')">🤖 一键原地抓取并放回 (全自动任意位置)</button>
        <button class="btn-blue" id="btn-place" onclick="postApi('/api/pick_and_place')">📍 确认抓取并搬运到点击位置</button>
        <button class="btn-slate" id="btn-home" onclick="postApi('/api/retract_home')">🏠 机械臂安全回原点</button>
      </div>

      <!-- TAB 2: 在线手眼标定 -->
      <div id="tab-calib" class="tab-content">
        <div class="box" style="border-color:#2563eb;">
          <div style="font-weight:700; color:#60a5fa; margin-bottom:6px;">第一步：示教状态与对准模式</div>
          <select id="cal-touch-mode" style="width:100%; margin-bottom:8px;" onchange="syncCalibSel()">
            <option value="tip_touch">指尖闭合轻点 Tag 中心 (推荐最准)</option>
            <option value="cube_grasp">指尖张开居中包络方块</option>
          </select>
          <div style="display:flex; gap:8px;">
            <button class="btn-amber" style="flex:1;" onclick="setTeachMode(true)">🔓 开启自由示教拖动</button>
            <button class="btn-slate" style="flex:1;" onclick="setTeachMode(false)">🔒 锁回 CAN 伺服</button>
          </div>
          <div style="font-size:11.5px; color:#94a3b8; margin-top:5px;" id="cal-arm-live">正在读取机械臂关节角度...</div>
        </div>

        <div class="box" style="border-color:#16a34a;">
          <div style="font-weight:700; color:#4ade80; margin-bottom:6px;">第二步：依次采集 4~6 个桌面分散点</div>
          <div style="display:flex; gap:8px;">
            <button class="btn-blue" style="flex:1;" onclick="postApi('/api/calib/capture_cam')">📸 1. 视觉锁定方块</button>
            <button class="btn-purple" style="flex:1;" onclick="postApi('/api/calib/record_arm')">📍 2. 记录臂尖位置</button>
          </div>
          <div style="margin-top:6px; font-size:12px;" id="cal-pending">请先移开机械臂，放好方块后点击【1. 视觉锁定方块】</div>
        </div>

        <div class="box" style="max-height:180px; overflow-y:auto;">
          <div class="row-between" style="margin-bottom:4px;">
            <span style="font-weight:700; color:#cbd5e1;">已采集标定点列表:</span>
            <span class="mono" id="cal-rmse">RMSE: --</span>
          </div>
          <table>
            <thead><tr><th>#</th><th>相机坐标(mm)</th><th>臂尖坐标(mm)</th><th>误差</th><th>操作</th></tr></thead>
            <tbody id="cal-tbody"></tbody>
          </table>
        </div>

        <div style="display:flex; gap:8px;">
          <button class="btn-green" style="flex:2;" onclick="postApi('/api/calib/solve_and_save')">💾 计算并保存外参立即生效</button>
          <button class="btn-red" style="flex:1;" onclick="postApi('/api/calib/clear_points')">🗑️ 清空点位</button>
        </div>
      </div>

      <!-- 底部共享日志 -->
      <div style="padding:0 14px 14px 14px;">
        <div style="font-size:11.5px; color:#94a3b8; font-weight:700; margin-bottom:4px;">任务执行实时日志:</div>
        <div class="log-box" id="log-box">等待任务指令...</div>
      </div>
    </div>
  </div>
</div>

<script>
  const canvas = document.getElementById('live-canvas');
  const ctx = canvas.getContext('2d');
  const fpsBadge = document.getElementById('fps-indicator');
  let isFetching = false, frameCount = 0, lastFpsTime = performance.now();
  let paramsLoaded = false;

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
    const rect = canvas.getBoundingClientRect();
    const u = Math.round((e.clientX - rect.left) * (640 / rect.width));
    const v = Math.round((e.clientY - rect.top) * (480 / rect.height));
    postApi('/api/select_point', {u, v});
  });

  function switchTab(id, btn) {
    document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
    document.querySelectorAll('.tab-btn').forEach(el => el.classList.remove('active'));
    document.getElementById(id).classList.add('active');
    btn.classList.add('active');
  }

  function postApi(url, body = {}) {
    return fetch(url, {
      method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)
    }).then(r => r.json()).then(d => {
      if (d.status === 'error') alert(d.msg);
      updateStatus();
      return d;
    });
  }

  function startCamera() { postApi('/api/start_camera'); }
  function clearPoint() { postApi('/api/clear_point'); }
  function setTeachMode(enable) { postApi('/api/arm_teach_mode', {enable}); }
  function syncCalibSel() { postApi('/api/calib/select_arm', {touch_mode: document.getElementById('cal-touch-mode').value}); }

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
      y_offset_mm: document.getElementById('slider-yoff').value
    });
  }

  let winCamOn = false;
  function toggleWinCam() {
    winCamOn = !winCamOn;
    const box = document.getElementById('win-cam-box'), img = document.getElementById('win-cam-img'), btn = document.getElementById('btn-wincam');
    if (winCamOn) { box.style.display='block'; img.src='http://localhost:8082/stream?t='+Date.now(); btn.innerText='关闭副视角'; btn.style.background='#dc2626'; }
    else { box.style.display='none'; img.src=''; btn.innerText='开启副视角'; btn.style.background='#475569'; }
  }

  function updateStatus() {
    fetch('/api/status?t=' + Date.now()).then(r => r.json()).then(d => {
      const bCam = document.getElementById('badge-cam');
      bCam.innerText = d.camera_online ? '📷 相机: 在线' : '📷 相机: 未启动';
      bCam.className = 'pill ' + (d.camera_online ? 'ok' : 'err');

      const aid = d.primary_arm || 'arm_a';
      const arm = d.arms[aid], rt = arm.runtime;
      const bArm = document.getElementById('badge-arm');
      const modeTxt = rt.ctrl_mode === 2 ? '示教模式' : (arm.calibrated ? '已标定' : '未标定');
      bArm.innerText = `🦾 机械臂: ${rt.motors_alive ? '在线(' + modeTxt + ')' : '未连接'}`;
      bArm.className = 'pill ' + (rt.motors_alive ? 'ok' : 'err');

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
        updEWLabel();
      }

      const stTag = document.getElementById('st-tag');
      stTag.innerText = d.tag_detected ? `已锁定 (ID: ${d.tag_id})` : '未检测到方块';
      stTag.style.color = d.tag_detected ? '#22c55e' : '#f97316';
      const armPos = (aid === 'arm_a') ? d.tag_arm_a_mm : d.tag_arm_b_mm;
      document.getElementById('st-tag-pos').innerText = armPos
        ? `[${armPos.join(', ')}] mm`
        : (d.tag_cam_mm ? `未标定 (相机系 [${d.tag_cam_mm.join(', ')}])` : '--');
      document.getElementById('st-grip').innerText = rt.gripper_mm + ' mm';

      const stClick = document.getElementById('st-click');
      if (d.clicked_eval && d.clicked_eval[aid]) {
        const ev = d.clicked_eval[aid];
        stClick.innerText = (ev.valid ? '✅ ' : '❌ ') + ev.msg;
        stClick.style.color = ev.valid ? '#4ade80' : '#f87171';
      } else {
        stClick.innerText = '请点击左侧桌面空白位置';
        stClick.style.color = '#facc15';
      }

      document.getElementById('cal-arm-live').innerText =
        `状态=${rt.motors_alive ? (rt.ctrl_mode===2 ? '🔓自由拖动示教中' : '🔒CAN伺服锁定') : '🔴未连接'} | 关节(deg)=[${rt.joints_deg.join(', ')}]`;
      document.getElementById('cal-pending').innerHTML = d.calib.pending_cam_mm
        ? `<span style="color:#4ade80;">✅ 已锁存视觉坐标 [${d.calib.pending_cam_mm.join(', ')}] mm → 请拖动指尖轻点 Tag 中心后点击【2. 记录臂尖位置】</span>`
        : '请先拿开手臂，放好方块并点击【1. 视觉锁定方块】';
      document.getElementById('cal-rmse').innerText = d.calib.preview_rmse_mm !== null ? `实时预览 RMSE: ${d.calib.preview_rmse_mm} mm` : `至少需4点 (当前 ${d.calib.points.length})`;

      document.getElementById('cal-tbody').innerHTML = d.calib.points.map((p, idx) => `<tr>
        <td>C${p.idx}</td>
        <td>${p.p_cam_mm.join(', ')}</td>
        <td>${p.p_base_mm.join(', ')}</td>
        <td>${p.err_mm !== null ? p.err_mm + 'mm' : '-'}</td>
        <td><button class="btn-red btn-sm" style="padding:2px 6px;" onclick="postApi('/api/calib/delete_point',{idx:${idx}})">删</button></td>
      </tr>`).join('');

      document.getElementById('btn-return').disabled = d.task_busy || !d.tag_detected;
      document.getElementById('btn-place').disabled = d.task_busy || !d.tag_detected || !d.clicked_uv;
      document.getElementById('btn-home').disabled = d.task_busy;

      const logBox = document.getElementById('log-box');
      if (d.logs && d.logs.length > 0) {
        logBox.innerText = d.logs.join('\n');
        logBox.scrollTop = logBox.scrollHeight;
      }
    }).catch(() => {});
  }
  setInterval(updateStatus, 400);
</script>
</body>
</html>
'''
