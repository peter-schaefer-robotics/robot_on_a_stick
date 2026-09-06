/*
 * Drawing of the scene, the time histories and the planned input sequence.
 * All colours come from the CSS variables so light/dark follows automatically.
 * Drawing coordinates: metres -> CSS pixels.
 */
(function (root) {
  'use strict';
  var IPM = root.IPM = root.IPM || {};

  var colors = {};

  // The robot head is loaded once from the inlined SVG. Until it is decoded we
  // fall back to a plain disc, so nothing pops or flickers on first paint.
  var robotImg = null, robotReady = false;
  function ensureRobot() {
    if (robotImg || !IPM.robot) return;
    robotImg = new Image();
    robotImg.onload = function () { robotReady = true; };
    robotImg.src = IPM.robot.dataUri;
  }

  function readColors() {
    var cs = getComputedStyle(document.documentElement);
    var pick = function (n, fb) { return (cs.getPropertyValue(n) || fb).trim(); };
    colors = {
      fg: pick('--fg', '#e7eaf0'),
      dim: pick('--fg-dim', '#98a2b6'),
      faint: pick('--fg-faint', '#6b7488'),
      border: pick('--border', '#262d3b'),
      panel: pick('--panel', '#161a23'),
      panel2: pick('--panel-2', '#1c2130'),
      accent: pick('--accent', '#4fc3d9'),
      accent2: pick('--accent-2', '#e0a33e'),
      danger: pick('--danger', '#e2626b'),
      ok: pick('--ok', '#7cc47f'),
      violet: pick('--violet', '#a98cf0'),
      bobBg: pick('--bob-bg', '#eef2f7')
    };
    return colors;
  }

  /** Scale the canvas to device pixels; returns the size in CSS pixels. */
  function fit(canvas) {
    var dpr = Math.min(root.devicePixelRatio || 1, 2.5);
    var rect = canvas.getBoundingClientRect();
    var w = Math.max(1, Math.round(rect.width));
    var h = Math.max(1, Math.round(rect.height));
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    var ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { ctx: ctx, w: w, h: h };
  }

  function withAlpha(hex, a) {
    hex = (hex || '').trim();
    if (hex.charAt(0) === '#') {
      var s = hex.slice(1);
      if (s.length === 3) s = s[0] + s[0] + s[1] + s[1] + s[2] + s[2];
      var r = parseInt(s.slice(0, 2), 16), g = parseInt(s.slice(2, 4), 16), b = parseInt(s.slice(4, 6), 16);
      if (isFinite(r) && isFinite(g) && isFinite(b)) return 'rgba(' + r + ',' + g + ',' + b + ',' + a + ')';
    }
    return hex;
  }

  function arrow(ctx, x0, y0, x1, y1, color, width) {
    var dx = x1 - x0, dy = y1 - y0;
    var len = Math.hypot(dx, dy);
    if (len < 1.5) return;
    var ux = dx / len, uy = dy / len;
    var head = Math.min(11, len * 0.45);
    ctx.save();
    ctx.strokeStyle = color; ctx.fillStyle = color;
    ctx.lineWidth = width || 2.5; ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(x0, y0);
    ctx.lineTo(x1 - ux * head * 0.9, y1 - uy * head * 0.9);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x1 - ux * head - uy * head * 0.42, y1 - uy * head + ux * head * 0.42);
    ctx.lineTo(x1 - ux * head + uy * head * 0.42, y1 - uy * head - ux * head * 0.42);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  function roundRect(ctx, x, y, w, h, r) {
    ctx.beginPath();
    if (ctx.roundRect) { ctx.roundRect(x, y, w, h, r); return; }
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
  }

  /**
   * Main scene.
   * v = {
   *   state, plant, u, fd, xref, mouse:{x,y,active,radius},
   *   pred: Float64Array|null, showPred, showForces, trace:[{x,y}], camX, umax
   * }
   * Returns the transform in use so the app can convert pointer coordinates
   * back into metres.
   */
  function drawScene(canvas, v) {
    var f = fit(canvas), ctx = f.ctx, w = f.w, h = f.h;
    var C = colors;
    var viewW = Math.max(2.2, 2.6 * Math.max(0.7, Math.min(1.4, w / 900)));
    var scale = w / viewW;
    var groundY = h * 0.76;
    var camX = v.camX || 0;
    var X = function (x) { return w / 2 + (x - camX) * scale; };
    var Y = function (y) { return groundY - y * scale; };

    ctx.clearRect(0, 0, w, h);

    // background
    ctx.fillStyle = C.panel2;
    roundRect(ctx, 0, 0, w, h, 9); ctx.fill();

    // scale: ticks every 0.5 m
    ctx.save();
    ctx.strokeStyle = withAlpha(C.faint, 0.28);
    ctx.fillStyle = withAlpha(C.faint, 0.85);
    ctx.lineWidth = 1;
    ctx.font = '11.5px ui-monospace, Menlo, monospace';
    ctx.textAlign = 'center';
    var m0 = Math.ceil((camX - viewW / 2) / 0.5) * 0.5;
    for (var mx = m0; mx <= camX + viewW / 2; mx += 0.5) {
      var px = X(mx);
      ctx.beginPath();
      ctx.moveTo(px, groundY + 4);
      ctx.lineTo(px, groundY + (Math.abs(mx) < 1e-9 ? 14 : 9));
      ctx.stroke();
      if (Math.abs(mx % 1) < 1e-9) ctx.fillText(mx.toFixed(0) + ' m', px, groundY + 26);
    }
    ctx.restore();

    // rail
    ctx.strokeStyle = withAlpha(C.faint, 0.5);
    ctx.lineWidth = 3; ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(0, groundY + 1.5);
    ctx.lineTo(w, groundY + 1.5);
    ctx.stroke();

    // target position
    var xr = X(v.xref[0]);
    ctx.save();
    ctx.strokeStyle = withAlpha(C.accent, 0.55);
    ctx.setLineDash([4, 4]);
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(xr, groundY - h * 0.62);
    ctx.lineTo(xr, groundY + 3);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = withAlpha(C.accent, 0.8);
    ctx.beginPath();
    ctx.moveTo(xr, groundY - 4);
    ctx.lineTo(xr - 5, groundY - 13);
    ctx.lineTo(xr + 5, groundY - 13);
    ctx.closePath(); ctx.fill();
    ctx.restore();

    var p = v.state[0], th = v.state[2], l = v.plant.l;
    var cartW = 0.30 * scale, cartH = 0.16 * scale;
    var pivotX = X(p), pivotY = groundY - cartH * 0.82;
    var bobX = X(p + l * Math.sin(th));
    var bobY = Y(l * Math.cos(th)) - cartH * 0.82;
    var bobR = Math.max(11, 0.07 * scale);   // big enough to show the robot

    // trace of the ball
    if (v.trace && v.trace.length > 1) {
      ctx.save();
      ctx.strokeStyle = withAlpha(C.accent2, 0.32);
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      for (var i = 0; i < v.trace.length; i++) {
        var tx = X(v.trace[i].x), ty = Y(v.trace[i].y) - cartH * 0.82;
        if (i === 0) ctx.moveTo(tx, ty); else ctx.lineTo(tx, ty);
      }
      ctx.stroke();
      ctx.restore();
    }

    // prediction (ghost pendulums plus the path of the ball)
    if (v.showPred && v.pred && v.pred.length >= 4) {
      var n = v.pred.length / 4;
      ctx.save();
      ctx.strokeStyle = withAlpha(C.accent, 0.42);
      ctx.lineWidth = 1.5;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      for (var k = 0; k < n; k++) {
        var pk = v.pred[k * 4], tk = v.pred[k * 4 + 2];
        var gx = X(pk + l * Math.sin(tk)), gy = Y(l * Math.cos(tk)) - cartH * 0.82;
        if (k === 0) ctx.moveTo(bobX, bobY);
        ctx.lineTo(gx, gy);
      }
      ctx.stroke();
      ctx.setLineDash([]);

      var everyK = Math.max(1, Math.round(n / 7));
      for (k = everyK - 1; k < n; k += everyK) {
        var a = 0.30 * (1 - k / n) + 0.06;
        var pk2 = v.pred[k * 4], tk2 = v.pred[k * 4 + 2];
        var gx2 = X(pk2), gy2 = groundY - cartH * 0.82;
        var bx2 = X(pk2 + l * Math.sin(tk2)), by2 = Y(l * Math.cos(tk2)) - cartH * 0.82;
        ctx.strokeStyle = withAlpha(C.accent, a);
        ctx.lineWidth = 2;
        ctx.beginPath(); ctx.moveTo(gx2, gy2); ctx.lineTo(bx2, by2); ctx.stroke();
        ctx.fillStyle = withAlpha(C.accent, a * 0.8);
        ctx.beginPath(); ctx.arc(bx2, by2, bobR * 0.5, 0, 6.2832); ctx.fill();
        ctx.strokeStyle = withAlpha(C.accent, a * 0.65);
        ctx.lineWidth = 1.2;
        ctx.strokeRect(gx2 - cartW / 2, groundY - cartH, cartW, cartH * 0.92);
      }
      ctx.restore();
    }

    // cart
    ctx.save();
    ctx.fillStyle = C.panel;
    ctx.strokeStyle = withAlpha(C.accent, 0.85);
    ctx.lineWidth = 2;
    roundRect(ctx, pivotX - cartW / 2, groundY - cartH, cartW, cartH, 4);
    ctx.fill(); ctx.stroke();
    ctx.fillStyle = withAlpha(C.faint, 0.75);
    var wr = Math.max(3, cartH * 0.19);
    ctx.beginPath(); ctx.arc(pivotX - cartW * 0.28, groundY, wr, 0, 6.2832); ctx.fill();
    ctx.beginPath(); ctx.arc(pivotX + cartW * 0.28, groundY, wr, 0, 6.2832); ctx.fill();
    ctx.restore();

    // rod, pivot and the robot head at the tip
    ctx.save();
    ctx.strokeStyle = C.fg;
    ctx.lineWidth = Math.max(3, 0.016 * scale);
    ctx.lineCap = 'round';
    ctx.beginPath(); ctx.moveTo(pivotX, pivotY); ctx.lineTo(bobX, bobY); ctx.stroke();
    ctx.fillStyle = withAlpha(C.fg, 0.9);
    ctx.beginPath(); ctx.arc(pivotX, pivotY, Math.max(2.5, bobR * 0.22), 0, 6.2832); ctx.fill();
    ctx.restore();

    ensureRobot();
    ctx.save();
    ctx.beginPath(); ctx.arc(bobX, bobY, bobR, 0, 6.2832);
    ctx.fillStyle = C.bobBg;
    ctx.fill();
    if (robotReady) {
      ctx.save();
      ctx.clip();
      // Scale the artwork (not the viewBox) to fill the disc and centre it.
      var bx = IPM.robot.box;
      var sc = 1.94 * bobR / bx.h;
      ctx.drawImage(robotImg, bobX - bx.cx * sc, bobY - bx.cy * sc, 200 * sc, 200 * sc);
      ctx.restore();
    } else {
      ctx.fillStyle = withAlpha(C.accent2, 0.9);
      ctx.fill();
    }
    ctx.strokeStyle = withAlpha(C.fg, 0.55);
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.restore();

    // forces
    if (v.showForces) {
      var uLen = (v.u / Math.max(1e-6, v.umax)) * 0.62 * scale;
      if (Math.abs(uLen) > 2) {
        arrow(ctx, pivotX, groundY - cartH * 0.5, pivotX + uLen, groundY - cartH * 0.5,
              withAlpha(C.violet, 0.95), 3);
        ctx.save();
        ctx.fillStyle = withAlpha(C.violet, 0.9);
        ctx.font = '12.5px ui-monospace, Menlo, monospace';
        ctx.textAlign = uLen > 0 ? 'left' : 'right';
        ctx.fillText('u = ' + v.u.toFixed(1) + ' N',
                     pivotX + uLen + (uLen > 0 ? 6 : -6), groundY - cartH * 0.5 - 8);
        ctx.restore();
      }
      if (Math.abs(v.fd) > 0.05) {
        var fLen = Math.sign(v.fd) * Math.min(0.5, Math.abs(v.fd) / 25 * 0.5) * scale;
        arrow(ctx, bobX - fLen, bobY, bobX - Math.sign(fLen) * bobR * 1.15, bobY,
              withAlpha(C.danger, 0.95), 3);
      }
    }

    // the rubber band while the user drags the cart
    if (v.drag && v.drag.active) {
      var dxp = X(v.drag.x), dyp = groundY - cartH * 0.5;
      ctx.save();
      ctx.strokeStyle = withAlpha(C.accent, 0.75);
      ctx.lineWidth = 2;
      ctx.setLineDash([5, 4]);
      ctx.beginPath(); ctx.moveTo(pivotX, dyp); ctx.lineTo(dxp, dyp); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = withAlpha(C.accent, 0.9);
      ctx.beginPath(); ctx.arc(dxp, dyp, 5, 0, 6.2832); ctx.fill();
      ctx.restore();
    }

    // radius of influence of the pointer
    if (v.mouse && v.mouse.active) {
      ctx.save();
      var mx2 = X(v.mouse.x), my2 = Y(v.mouse.y) - cartH * 0.82;
      var rad = v.mouse.radius * scale;
      ctx.strokeStyle = withAlpha(C.danger, 0.28);
      ctx.setLineDash([3, 5]);
      ctx.lineWidth = 1.2;
      ctx.beginPath(); ctx.arc(mx2, my2, rad, 0, 6.2832); ctx.stroke();
      ctx.restore();
    }

    // angle arc
    ctx.save();
    ctx.strokeStyle = withAlpha(C.dim, 0.45);
    ctx.lineWidth = 1;
    ctx.setLineDash([2, 3]);
    ctx.beginPath();
    ctx.moveTo(pivotX, pivotY);
    ctx.lineTo(pivotX, pivotY - l * scale * 1.02);
    ctx.stroke();
    ctx.setLineDash([]);
    if (Math.abs(th) > 0.02) {
      ctx.beginPath();
      ctx.arc(pivotX, pivotY, l * scale * 0.34, -Math.PI / 2, -Math.PI / 2 + th, th < 0);
      ctx.stroke();
      ctx.fillStyle = withAlpha(C.dim, 0.9);
      ctx.font = '12.5px ui-sans-serif, system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('θ', pivotX + Math.sin(th / 2) * l * scale * 0.46,
                   pivotY - Math.cos(th / 2) * l * scale * 0.46 + 4);
    }
    ctx.restore();

    return { X: X, Y: Y, scale: scale, groundY: groundY, cartOffset: cartH * 0.82, camX: camX };
  }

  /** Time histories: three traces, each with its own scaling. */
  function drawScope(canvas, hist, tNow, span, scales) {
    var f = fit(canvas), ctx = f.ctx, w = f.w, h = f.h;
    var C = colors;
    var padL = 8, padR = 46, padT = 8, padB = 14;
    var x0 = padL, x1 = w - padR, y0 = padT, y1 = h - padB;

    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = C.panel2;
    roundRect(ctx, 0, 0, w, h, 8); ctx.fill();

    // zero line and time grid
    ctx.save();
    ctx.strokeStyle = withAlpha(C.faint, 0.25);
    ctx.lineWidth = 1;
    var ymid = (y0 + y1) / 2;
    ctx.beginPath(); ctx.moveTo(x0, ymid); ctx.lineTo(x1, ymid); ctx.stroke();
    ctx.strokeStyle = withAlpha(C.faint, 0.14);
    for (var s = Math.ceil(tNow - span); s <= tNow; s += 2) {
      var px = x0 + (s - (tNow - span)) / span * (x1 - x0);
      ctx.beginPath(); ctx.moveTo(px, y0); ctx.lineTo(px, y1); ctx.stroke();
    }
    ctx.restore();

    if (!hist.t.length) return;

    var series = [
      { key: 'th', color: C.accent2, scale: scales.th, unit: '°' },
      { key: 'p', color: C.accent, scale: scales.p, unit: 'm' },
      { key: 'u', color: C.violet, scale: scales.u, unit: 'N' }
    ];

    for (var si = 0; si < series.length; si++) {
      var ser = series[si], arr = hist[ser.key];
      ctx.save();
      ctx.strokeStyle = ser.color;
      ctx.lineWidth = 1.6;
      ctx.lineJoin = 'round';
      ctx.beginPath();
      var started = false;
      for (var i = 0; i < hist.t.length; i++) {
        var tt = hist.t[i];
        if (tt < tNow - span) continue;
        var px2 = x0 + (tt - (tNow - span)) / span * (x1 - x0);
        var vv = arr[i] / ser.scale;
        vv = vv > 1 ? 1 : (vv < -1 ? -1 : vv);
        var py = ymid - vv * (y1 - y0) / 2 * 0.94;
        if (!started) { ctx.moveTo(px2, py); started = true; } else ctx.lineTo(px2, py);
      }
      ctx.stroke();
      ctx.restore();

      // scale annotation on the right
      ctx.save();
      ctx.fillStyle = withAlpha(ser.color, 0.9);
      ctx.font = '11.5px ui-monospace, Menlo, monospace';
      ctx.textAlign = 'left';
      ctx.fillText('±' + (ser.scale >= 10 ? ser.scale.toFixed(0) : ser.scale.toFixed(1)) + ser.unit,
                   x1 + 5, y0 + 11 + si * 12);
      ctx.restore();
    }
  }

  /** Planned input sequence as a bar chart. */
  function drawUPlan(canvas, useq, umax, Ts) {
    var f = fit(canvas), ctx = f.ctx, w = f.w, h = f.h;
    var C = colors;
    var padL = 8, padR = 34, padT = 8, padB = 16;
    var x0 = padL, x1 = w - padR, y0 = padT, y1 = h - padB;
    var ymid = (y0 + y1) / 2;

    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = C.panel2;
    roundRect(ctx, 0, 0, w, h, 8); ctx.fill();

    ctx.save();
    ctx.strokeStyle = withAlpha(C.danger, 0.35);
    ctx.setLineDash([4, 4]); ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x0, y0 + 2); ctx.lineTo(x1, y0 + 2); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(x0, y1 - 2); ctx.lineTo(x1, y1 - 2); ctx.stroke();
    ctx.setLineDash([]);
    ctx.strokeStyle = withAlpha(C.faint, 0.3);
    ctx.beginPath(); ctx.moveTo(x0, ymid); ctx.lineTo(x1, ymid); ctx.stroke();
    ctx.fillStyle = withAlpha(C.danger, 0.75);
    ctx.font = '11.5px ui-monospace, Menlo, monospace';
    ctx.textAlign = 'left';
    ctx.fillText('+' + umax.toFixed(0) + 'N', x1 + 4, y0 + 8);
    ctx.fillText('-' + umax.toFixed(0) + 'N', x1 + 4, y1 + 1);
    ctx.restore();

    if (!useq || !useq.length) return;
    var N = useq.length;
    var bw = (x1 - x0) / N;
    for (var k = 0; k < N; k++) {
      var v = useq[k] / umax;
      v = v > 1 ? 1 : (v < -1 ? -1 : v);
      var bh = v * (ymid - y0 - 2);
      var sat = Math.abs(useq[k]) >= umax - 1e-6;
      ctx.fillStyle = k === 0
        ? withAlpha(C.accent2, 0.95)
        : withAlpha(sat ? C.danger : C.violet, 0.32 + 0.4 * (1 - k / N));
      var bx = x0 + k * bw;
      ctx.fillRect(bx + bw * 0.12, bh > 0 ? ymid - bh : ymid, Math.max(1, bw * 0.76), Math.abs(bh));
    }
    ctx.save();
    ctx.fillStyle = withAlpha(C.faint, 0.85);
    ctx.font = '11.5px ui-monospace, Menlo, monospace';
    ctx.textAlign = 'left';
    ctx.fillText('k=0', x0, y1 + 12);
    ctx.textAlign = 'right';
    ctx.fillText('k=' + (N - 1) + '  (' + (N * Ts).toFixed(2) + ' s)', x1, y1 + 12);
    ctx.restore();
  }

  IPM.render = {
    readColors: readColors,
    drawScene: drawScene,
    drawScope: drawScope,
    drawUPlan: drawUPlan,
    colors: function () { return colors; }
  };
})(typeof window !== 'undefined' ? window : globalThis);
