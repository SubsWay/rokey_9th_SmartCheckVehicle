// SCV server page — 3-3차 (login · 모니터(순찰 시작 · 중단) · 이력 · 정보)
import { html, createRoot, useState, useEffect, useRef, useCallback } from '/static/vendor/react-htm.mjs';

// ---------------------------------------------------------------- API
async function api(path, opts = {}) {
  const o = { credentials: 'same-origin', ...opts };
  if (o.json !== undefined) {
    o.method = o.method || 'POST';
    o.headers = { 'Content-Type': 'application/json' };
    o.body = JSON.stringify(o.json);
    delete o.json;
  }
  const r = await fetch(path, o);
  let data = null;
  try { data = await r.json(); } catch { /* 본문 없음 */ }
  if (!r.ok) {
    const d = data && data.detail;
    const err = new Error(typeof d === 'string' ? d : (d ? JSON.stringify(d) : `오류 (${r.status})`));
    err.status = r.status;
    throw err;
  }
  return data;
}

const TABS = [
  { id: 'monitor', label: '모니터' },
  { id: 'history', label: '이력' },
  { id: 'master', label: '정보' },
];
const pageFromPath = () => (location.pathname.match(/^\/server\/(\w+)/) || [])[1] || 'login';

// ---------------------------------------------------------------- 표시 도우미
const fmtTime = (v) => {
  if (!v) return '—';
  const d = new Date(v);
  const p = (n) => String(n).padStart(2, '0');
  return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())} ${p(d.getHours())}:${p(d.getMinutes())}`;
};
const fmtNum = (v) => (v === null || v === undefined ? '—' : (Number.isInteger(v) ? String(v) : Number(v).toFixed(3).replace(/\.?0+$/, '')));

function showValue(col, v) {
  if (v === null || v === undefined || v === '') return html`<span class="no">—</span>`;
  switch (col.type) {
    case 'bool': return v ? html`<span class="yes">예</span>` : html`<span class="no">아니요</span>`;
    case 'datetime': return html`<span class="nowrap">${fmtTime(v)}</span>`;
    case 'real': case 'int': return html`<span class="num">${fmtNum(v)}</span>`;
    case 'pose': return html`<span class="num">${fmtNum(v.x)}, ${fmtNum(v.y)}, ${fmtNum(v.yaw)}</span>`;
    case 'docks': {
      const ks = Object.keys(v);
      return ks.length ? html`<span class="num">${ks.map((k) => `${k} (${fmtNum(v[k].x)}, ${fmtNum(v[k].y)})`).join(' · ')}</span>`
        : html`<span class="no">—</span>`;
    }
    case 'select': {
      const o = (col.options || []).find((x) => String(x.value) === String(v));
      return o ? o.label : String(v);
    }
    default: return String(v);
  }
}

// 자세 입력칸 안내 글자 — 지도 좌표(m)와 방향(rad) 단위를 함께 보여 줌 (도 °로 잘못 넣지 않게)
const POSE_PH = { x: 'x (m)', y: 'y (m)', yaw: 'yaw (rad)' };

function Input({ col, value, onChange }) {
  const set = (v) => onChange(col.name, v);
  switch (col.type) {
    case 'bool':
      return html`<input type="checkbox" checked=${!!value} onChange=${(e) => set(e.target.checked)} />`;
    case 'int': case 'real':
      return html`<input type="number" step="any" value=${value ?? ''} onInput=${(e) => set(e.target.value)} />`;
    case 'password':
      return html`<input type="password" value=${value ?? ''} onInput=${(e) => set(e.target.value)} />`;
    case 'select':
      return html`<select value=${value ?? ''} onChange=${(e) => set(e.target.value)}>
        <option value="">선택</option>
        ${(col.options || []).map((o) => html`<option key=${o.value} value=${o.value}>${o.label}</option>`)}
      </select>`;
    case 'pose': {                                      // 관측 자세 — x · y는 지도 좌표 m, yaw는 rad (안내 글자에 단위)
      const p = value || { x: '', y: '', yaw: '' };
      const part = (k, ph) => html`<input type="number" step="any" placeholder=${ph} value=${p[k] ?? ''}
        onInput=${(e) => set({ ...p, [k]: e.target.value })} />`;
      return html`<div class="pose">${part('x', POSE_PH.x)}${part('y', POSE_PH.y)}${part('yaw', POSE_PH.yaw)}</div>`;
    }
    case 'docks': {                                     // 로봇마다 x · y(m) · yaw(rad) (비우면 그 로봇 도크 없음)
      const v = value || {};
      const part = (rid, k) => html`<input type="number" step="any" placeholder=${POSE_PH[k]} value=${(v[rid] || {})[k] ?? ''}
        onInput=${(e) => set({ ...v, [rid]: { ...(v[rid] || {}), [k]: e.target.value } })} />`;
      return html`<div class="docks">${(col.options || []).map((rid) => html`<div class="pose" key=${rid}>
        <span class="rid">${rid}</span>${part(rid, 'x')}${part(rid, 'y')}${part(rid, 'yaw')}</div>`)}
        <span class="help">도크 바로 앞에서 도크를 바라보는 자세 · 비우면 도크 없음</span></div>`;
    }
    default:
      return html`<input type="text" value=${value ?? ''} onInput=${(e) => set(e.target.value)} />`;
  }
}

// 쪽 번호 — 최신 순 10행씩. 쪽이 많으면 처음 · 끝 · 현재 앞뒤 2쪽만 보이고 나머지는 …
function Pager({ page, pages, onPage, disabled }) {
  if (!pages || pages <= 1) return null;
  const nums = [];
  for (let i = 1; i <= pages; i++) {
    if (i === 1 || i === pages || Math.abs(i - page) <= 2) nums.push(i);
    else if (nums[nums.length - 1] !== '…') nums.push('…');
  }
  return html`<div class="pager">
    <button class="pg" disabled=${disabled || page <= 1} onClick=${() => onPage(page - 1)} title="이전 쪽">‹</button>
    ${nums.map((n, i) => n === '…' ? html`<span key=${'e' + i} class="pg gap">…</span>`
      : html`<button key=${n} class=${'pg' + (n === page ? ' on' : '')} disabled=${disabled}
          onClick=${() => n !== page && onPage(n)}>${n}</button>`)}
    <button class="pg" disabled=${disabled || page >= pages} onClick=${() => onPage(page + 1)} title="다음 쪽">›</button>
  </div>`;
}

// ---------------------------------------------------------------- 로그인
function LoginPage({ onLogin }) {
  const [username, setU] = useState('');
  const [password, setP] = useState('');
  const [msg, setMsg] = useState('');
  const [busy, setBusy] = useState(false);
  const submit = async (e) => {
    e.preventDefault();
    setBusy(true); setMsg('');
    try {
      onLogin(await api('/api/login', { json: { username, password } }));
    } catch (err) { setMsg(err.message); }
    setBusy(false);
  };
  return html`<form class="login" onSubmit=${submit}>
    <h2>관리자 로그인</h2>
    <p class="sub">로그인하면 모니터 · 이력 · 정보 탭을 쓸 수 있습니다.</p>
    <div class="field"><label>아이디</label>
      <input autofocus value=${username} onInput=${(e) => setU(e.target.value)} autocomplete="username" /></div>
    <div class="field"><label>비밀번호</label>
      <input type="password" value=${password} onInput=${(e) => setP(e.target.value)} autocomplete="current-password" /></div>
    <button class="btn primary" disabled=${busy}>로그인</button>
    ${msg && html`<div class="msg">${msg}</div>`}
  </form>`;
}

// ---------------------------------------------------------------- monitor
const ROBOT_COLORS = { robot3: '#c2410c', robot4: '#6d28d9' };
const robotColor = (rid) => ROBOT_COLORS[rid] || '#2f5d7c';

// 로봇 상태 분류 — 정상 / 연결 끊김(E3 대기) / 응답 없음 / 비활성
function robotPhase(r) {
  if (r.state && r.connected) return ['ok', '정상'];
  if (r.state) return ['wait', `연결 끊김 — ${r.state.ttl_s}초 뒤 응답 없음`];
  if (r.is_active || r.connected) return ['bad', '응답 없음'];
  return ['off', '비활성'];
}

// 섹터 색 (3-2차) — 모니터 격자 · 범례가 같이 씀
const SECTOR_COLOR = { 1: '#2563eb', 2: '#d97706' };

// ---------------------------------------------------------------- 지도 정렬 (10/15 UI 시안)
//   원본: 지도 그대로(+x 오른쪽, +y 위) · 가로: +y 오른쪽, +x 아래 — 세로로 긴 testbed 를 화면 폭에 맞춤.
//   지도 이미지 · 격자 · 로봇 · 경로 · 표본이 모두 같은 변환 P(x, y) → [lat, lng] 을 쓴다. 로봇 좌표 자체는 바꾸지 않는다.
const ORIENT_KEY = 'scv_orient';
const readOrient = () => { try { return localStorage.getItem(ORIENT_KEY) || 'h'; } catch { return 'h'; } };
function useOrient() {                                 // 모니터 · 이력이 같은 정렬을 씀 (탭을 옮겨도 유지)
  const [o, setO] = useState(readOrient);
  useEffect(() => {
    const f = () => setO(readOrient());
    addEventListener('scv-orient', f);
    return () => removeEventListener('scv-orient', f);
  }, []);
  const set = (v) => { try { localStorage.setItem(ORIENT_KEY, v); } catch { /* 저장 못 해도 이 화면은 바뀜 */ } setO(v); dispatchEvent(new Event('scv-orient')); };
  return [o, set];
}
const PLAN_COLOR = '#eab308';                          // 계획 경로 — 두 섹터 모두 노랑 (10/15 PM)
const projector = (o) => (o === 'h' ? (x, y) => [-x, y] : (x, y) => [y, x]);

// 지도 범위(지도 좌표 x0..x1, y0..y1) — 이미지 크기를 알면 이미지 전체, 모르면(파일 없음) 격자 · 도크 · AP · 설비가 들어가는 범위
function mapExtent(m) {
  if (m.width && m.height) {
    return { x0: m.origin_x, y0: m.origin_y, x1: m.origin_x + m.width * m.resolution_m, y1: m.origin_y + m.height * m.resolution_m };
  }
  const g = m.grid_size_m || 1, xs = [], ys = [];
  const add = (x, y) => { if (Number.isFinite(x) && Number.isFinite(y)) { xs.push(x); ys.push(y); } };
  (m.cells || []).forEach(([, ix, iy]) => { add(m.origin_x + ix * g, m.origin_y + iy * g); add(m.origin_x + (ix + 1) * g, m.origin_y + (iy + 1) * g); });
  Object.values(m.dock_poses || {}).forEach((p) => add(p.x, p.y));
  (m.aps || []).forEach((a) => add(a.x, a.y));
  (m.facilities || []).forEach((f) => { add(f.expected_x, f.expected_y); if (f.view_pose) add(f.view_pose.x, f.view_pose.y); });
  if (!xs.length) return { x0: m.origin_x, y0: m.origin_y, x1: m.origin_x + 10, y1: m.origin_y + 10 };
  const pad = 0.5;
  return { x0: Math.min(...xs) - pad, y0: Math.min(...ys) - pad, x1: Math.max(...xs) + pad, y1: Math.max(...ys) + pad };
}
const boundsOf = (e, P) => {                          // 지도 좌표 사각형 → Leaflet 범위 (90° 돌려도 축에 맞는 사각형)
  const a = P(e.x0, e.y0), b = P(e.x1, e.y1);
  return [[Math.min(a[0], b[0]), Math.min(a[1], b[1])], [Math.max(a[0], b[0]), Math.max(a[1], b[1])]];
};
// 가로 정렬용 지도 이미지 — 시계 방향 90°: (X, Y) → (H − Y, X). 픽셀 그대로(부드럽게 하지 않음)
const rotatedImage = (url) => new Promise((ok, bad) => {
  const img = new Image();
  img.onload = () => {
    const c = document.createElement('canvas');
    c.width = img.height; c.height = img.width;
    const ctx = c.getContext('2d');
    ctx.imageSmoothingEnabled = false;
    ctx.setTransform(0, 1, -1, 0, img.height, 0);
    ctx.drawImage(img, 0, 0);
    ok(c.toDataURL());
  };
  img.onerror = bad;
  img.src = url;
});

// RSSI 색 — 약함 파랑 → 강함 빨강 (10/15 PM), −85 ~ −35 dBm 을 파랑(240°) → 청록 → 초록 → 노랑 → 빨강(0°) 으로 또렷하게
const RSSI_LO = -85, RSSI_HI = -35;
const rssiGrad = (v) => {
  if (v === null || v === undefined || !Number.isFinite(v)) return '#94a3b8';
  const t = Math.max(0, Math.min(1, (v - RSSI_LO) / (RSSI_HI - RSSI_LO)));
  return `hsl(${Math.round(240 - t * 240)}, 90%, ${Math.round(48 + Math.sin(t * Math.PI) * 4)}%)`;
};
const rssiColor = (v, thr) => (v < thr ? '#dc2626' : v < thr + 10 ? '#f59e0b' : '#16a34a');

// 레이어 (시안 11개 + 모니터용). 히트맵은 늘 켬
const LAYERS = [
  ['raw', '원본 지도'], ['cells', '셀 원본'], ['plan', '계획 경로'], ['track', '방문 궤적'], ['samples', 'NetSample'], ['unmeasured', '미측정'],
  ['failed', '도달 실패'], ['keepout', '회피 구역'], ['fac', '설비'], ['dock', '관측·Dock'], ['zone', '구역'], ['grid', '격자·값'],
];
const MONITOR_LAYERS = ['plan', 'track', 'unmeasured', 'cells', 'fac', 'dock', 'zone', 'grid', 'raw'];
const MONITOR_DEFAULT = { plan: true, track: true, unmeasured: true, fac: true, dock: true };
const HISTORY_DEFAULT = { cells: true, track: true, samples: true, unmeasured: true, failed: true, fac: true };
const ALL_ON = Object.fromEntries(LAYERS.map(([k]) => [k, true]));
function useLayers(key, def) {                         // 화면마다 켠 레이어를 기억 (브라우저 저장, 못 쓰면 기본값)
  const [v, setV] = useState(() => { try { return JSON.parse(localStorage.getItem(key)) || def; } catch { return def; } });
  const put = (n) => { setV(n); try { localStorage.setItem(key, JSON.stringify(n)); } catch { /* 무시 */ } };
  return [v, put];
}
function LayerBar({ keys, on, setOn, disabled = {}, extra }) {
  return html`<div class="layerbar">
    <span class="fixed">히트맵 · 기본 표시</span>
    ${LAYERS.filter(([k]) => keys.includes(k)).map(([k, label]) => html`<button key=${k}
      class=${'lyr' + (on[k] ? ' on' : '')} disabled=${!!disabled[k]} title=${disabled[k] || ''}
      onClick=${() => setOn({ ...on, [k]: !on[k] })}>${label}</button>`)}
    ${extra}
  </div>`;
}
function OrientSelect() {
  const [o, setO] = useOrient();
  return html`<label class="orient">정렬 <select value=${o} onChange=${(e) => setO(e.target.value)}>
    <option value="h">가로 정렬</option><option value="o">원본 방향</option></select></label>`;
}
function MapLegend({ opacity, setOpacity, history }) {
  return html`<div class="maplegend">
    <span class="lk"><i class="ln dash" style=${{ borderTopColor: PLAN_COLOR }}></i>계획 경로</span><span class="lk"><i class="ln"></i>방문 궤적</span>
    <span class="lk grad">RSSI <small>약(나쁨) −85</small><i class="bar"></i><small>−35 dBm 강(좋음)</small></span>
    <span class="lk"><i class="um"></i>미측정 (지나갔지만 신호 없음)</span>
    <label class="lk">농도 <input type="range" min="0.15" max="0.9" step="0.05" value=${opacity} onInput=${(e) => setOpacity(Number(e.target.value))} /></label>
    ${history && html`<span class="lk"><i class="ff">×</i>도달 실패 · 실행 사유 있음</span>
      <span class="lk"><i class="sp"></i>NetSample (속 빈 점 = 위치 불확실)</span>`}
  </div>`;
}

// 웹 표시용 도면 — 지도 이미지의 빈 공간(밝은 픽셀)이 많은 행 · 열 범위 = 방 안쪽 직사각형 (지도 좌표).
//   벽 픽셀이 들쭉날쭉하고 조금 기울어 있어도 곧게 그림. 행 · 열마다 빈 칸이 가장 많은 줄의 30% 넘는 곳만 (구석 잡음 제외)
const freeRect = (m) => new Promise((ok) => {
  const img = new Image();
  img.onload = () => {
    const c = document.createElement('canvas'), w = img.width, h = img.height;
    c.width = w; c.height = h;
    const ctx = c.getContext('2d');
    ctx.drawImage(img, 0, 0);
    const px = ctx.getImageData(0, 0, w, h).data, rows = new Array(h).fill(0), cols = new Array(w).fill(0);
    for (let j = 0; j < h; j++) for (let i = 0; i < w; i++) if (px[(j * w + i) * 4] > 230) { rows[j] += 1; cols[i] += 1; }
    const span = (a) => { const lim = Math.max(...a) * 0.3, idx = a.map((v, k) => (v > lim ? k : -1)).filter((k) => k >= 0); return idx.length ? [idx[0], idx[idx.length - 1]] : null; };
    const cr = span(cols), rr = span(rows), r = m.resolution_m;
    if (!cr || !rr) { ok(null); return; }
    // 안쪽 벽 — 방 안에서 어두운 픽셀이 0.4 m 넘게 이어진 세로 · 가로 줄. 이웃한 줄(기울어진 벽)은 하나로 묶어 곧게
    const dark = (i, j) => px[(j * w + i) * 4] < 120;
    const runs = (n, m2, at) => {                      // 줄 k 마다 가장 긴 어두운 구간 [k, a, b]
      const out = [];
      for (let k = n[0]; k <= n[1]; k++) {
        let best = null, a = -1;
        for (let t = m2[0]; t <= m2[1] + 1; t++) {
          if (t <= m2[1] && at(k, t)) { if (a < 0) a = t; } else if (a >= 0) { if (!best || t - a > best[1] - best[0]) best = [a, t - 1]; a = -1; }
        }
        if (best && best[1] - best[0] + 1 >= 8) out.push([k, best[0], best[1]]);
      }
      const segs = [];                                 // 이웃(2줄 안) 묶기 → [가운데 줄, 시작, 끝]
      out.forEach((q) => {
        const g = segs.length && segs[segs.length - 1];
        if (g && q[0] - g.k1 <= 2 && q[1] <= g.b + 2 && q[2] >= g.a - 2) { g.k1 = q[0]; g.ks.push(q[0]); g.a = Math.min(g.a, q[1]); g.b = Math.max(g.b, q[2]); }
        else segs.push({ k1: q[0], ks: [q[0]], a: q[1], b: q[2] });
      });
      return segs.map((g) => [g.ks.reduce((x, y) => x + y, 0) / g.ks.length, g.a, g.b]);
    };
    const X = (i) => m.origin_x + (i + 0.5) * r, Y = (j) => m.origin_y + (h - j - 0.5) * r;
    const walls = [
      ...runs(cr, rr, (i, j) => dark(i, j)).map(([i, a, b]) => [[X(i), Y(b) - r / 2], [X(i), Y(a) + r / 2]]),     // 세로 벽
      ...runs(rr, cr, (j, i) => dark(i, j)).map(([j, a, b]) => [[X(a) - r / 2, Y(j)], [X(b) + r / 2, Y(j)]]),     // 가로 벽
    ];
    const box = { x0: m.origin_x + cr[0] * r, x1: m.origin_x + (cr[1] + 1) * r, y0: m.origin_y + (h - 1 - rr[1]) * r, y1: m.origin_y + (h - rr[0]) * r };
    const near = 0.3;                                  // 외벽과 나란히 붙은 줄(기울어진 외벽이 안으로 들어온 것)은 빼고 칸막이만
    const snap = (v, lo, hi) => (v - lo < 0.4 ? lo : hi - v < 0.4 ? hi : v);   // 외벽 0.4 m 안에서 끝나는 칸막이는 외벽에 붙임
    box.walls = walls.filter(([a, b]) => (a[0] === b[0]
      ? Math.min(a[0] - box.x0, box.x1 - a[0]) > near
      : Math.min(a[1] - box.y0, box.y1 - a[1]) > near))
      .map(([a, b]) => (a[0] === b[0]
        ? [[a[0], snap(a[1], box.y0, box.y1)], [b[0], snap(b[1], box.y0, box.y1)]]
        : [[snap(a[0], box.x0, box.x1), a[1]], [snap(b[0], box.x0, box.x1), b[1]]]));
    ok(box);
  };
  img.onerror = () => ok(null);
  img.src = m.image_url;
});

// 도달 실패 — FAILED task 의 reason "BLOCKED skipped=[i…] shortened=[j…]" → 그 경로점 자리 (manager blocked_cells 와 같은 읽기)
function failedSpots(tasks) {
  const out = [];
  (tasks || []).forEach((t) => {
    const r = t.reason || '';
    if (!r.startsWith('BLOCKED') || !(t.waypoints || []).length) return;
    const idx = (k) => { const mm = r.match(new RegExp(k + '=\\[([^\\]]*)\\]')); return mm ? mm[1].split(',').map((s) => Number(s.trim())).filter(Number.isFinite) : []; };
    [...idx('skipped'), ...idx('shortened')].forEach((i) => {
      const w = t.waypoints[i];
      if (w) out.push({ x: w.x, y: w.y, task_id: t.task_id, robot_id: t.robot_id, reason: r, kind: idx('skipped').includes(i) ? '건너뜀' : '줄임' });
    });
  });
  return out;
}

function MapView({ m, robots, overlay, mini, plans, layers, opacity = 0.55 }) {
  const [orient] = useOrient();
  const P = projector(orient);
  const ref = useRef(null);
  const mapRef = useRef(null);
  const L1 = useRef({});                               // 레이어 그룹 — base(이미지) · info · overlay · plan · robot
  const markersRef = useRef({});
  const lay = layers || (mini ? { fac: true, dock: true, cells: true } : { fac: true, dock: true });   // 「자세히」 작은 지도는 칸 값 · 설비 · 도크
  useEffect(() => {
    const L = window.L;
    const map = L.map(ref.current, {
      crs: L.CRS.Simple, minZoom: -3, maxZoom: 9, zoomSnap: 0.25, attributionControl: false,
    });
    const ext = mapExtent(m), bounds = boundsOf(ext, P);
    map.createPane('floor').style.zIndex = 250;         // 지도 이미지 · 도면 — 히트맵 아래
    map.createPane('heat').style.zIndex = 380;          // 히트맵 — 부드럽게 (흐림), 지도 위 · 표시 아래
    map.getPane('heat').classList.add('heatpane');
    L1.current.base = L.layerGroup();                  // 원본 지도 이미지 (「원본 지도」 레이어)
    L1.current.plate = L.layerGroup();                 // 웹 표시용 도면 — 빈 공간 범위의 곧은 직사각형 (표시 전용, 좌표는 그대로)
    if (m.width && m.height) {
      if (orient === 'h') rotatedImage(m.image_url).then((u) => mapRef.current === map && L.imageOverlay(u, bounds, { className: 'mapimg', pane: 'floor' }).addTo(L1.current.base)).catch(() => {});
      else L.imageOverlay(m.image_url, bounds, { className: 'mapimg', pane: 'floor' }).addTo(L1.current.base);
      freeRect(m).then((r) => {
        if (mapRef.current !== map || !r) return;
        L.rectangle([P(r.x0, r.y0), P(r.x1, r.y1)], { pane: 'floor', color: '#9aa5ae', weight: 10, fillColor: '#ffffff', fillOpacity: 1, interactive: false })
          .addTo(L1.current.plate);
        (r.walls || []).forEach(([a, b]) => L.polyline([P(...a), P(...b)], { pane: 'floor', color: '#9aa5ae', weight: 10, lineCap: 'square', interactive: false })
          .addTo(L1.current.plate));
      }).catch(() => {});
    } else {                                           // 지도 이미지 파일이 없음 — 격자 · 등록 정보 범위로 맞추고 알림
      const Note = L.Control.extend({ onAdd: () => {
        const d = L.DomUtil.create('div', 'map-note');
        d.textContent = `지도 이미지 파일 없음 (${m.image_key}) — 격자 · 등록 정보만 표시`;
        return d;
      } });
      new Note({ position: 'bottomleft' }).addTo(map);
    }
    // 0.5 m 축척 — 실제 길이 (지도 해상도와 같은 변환)
    const Scale = L.Control.extend({ onAdd: () => {
      const d = L.DomUtil.create('div', 'map-scale');
      const draw = () => { const px = (m.grid_size_m || 0.5) * Math.pow(2, map.getZoom()); d.innerHTML = `<i style="width:${px}px"></i><span>${m.grid_size_m || 0.5} m</span>`; };
      map.on('zoomend', draw); draw();
      return d;
    } });
    new Scale({ position: 'bottomleft' }).addTo(map);
    const container = ref.current;
    let userMoved = false;
    const fit = () => { map.invalidateSize(); map.fitBounds(bounds, { padding: [8, 8], animate: false }); };
    container.addEventListener('pointerdown', () => { userMoved = true; });
    container.addEventListener('wheel', () => { userMoved = true; }, { passive: true });
    const FitAll = L.Control.extend({ onAdd: () => {
      const b = L.DomUtil.create('a', 'leaflet-bar map-fit');
      b.href = '#'; b.title = '전체 보기'; b.textContent = '⤢';
      L.DomEvent.on(b, 'click', (e) => { L.DomEvent.stop(e); fit(); userMoved = false; });
      L.DomEvent.disableClickPropagation(b);
      return b;
    } });
    new FitAll({ position: 'topleft' }).addTo(map);
    fit();
    let raf = requestAnimationFrame(fit), lastW = -1;  // 너비가 바뀔 때만, 다음 그리기 때 (ResizeObserver loop 방지). 첫 크기 잡힘 때 한 번 맞춤
    const ro = new ResizeObserver(() => {
      if (container.clientWidth === lastW) return;
      lastW = container.clientWidth;
      cancelAnimationFrame(raf);
      raf = requestAnimationFrame(() => { if (userMoved) map.invalidateSize(); else fit(); });
    });
    ro.observe(container);
    (lay.raw ? L1.current.base : L1.current.plate).addTo(map);
    L1.current.info = L.layerGroup().addTo(map);
    L1.current.overlay = L.layerGroup().addTo(map);
    L1.current.plan = L.layerGroup().addTo(map);
    L1.current.robot = L.layerGroup().addTo(map);       // 로봇 표시는 맨 위
    markersRef.current = {};
    mapRef.current = map;
    const onResize = () => map.invalidateSize();
    addEventListener('resize', onResize);
    return () => { ro.disconnect(); cancelAnimationFrame(raf); removeEventListener('resize', onResize); mapRef.current = null; map.remove(); };
  }, [m.map_id, m.image_url, orient]);

  // 등록 정보 (격자 · 구역 · 설비 · 도크 · AP) — 레이어 · 정렬이 바뀔 때 다시
  const layKey = JSON.stringify(lay);
  useEffect(() => {                                    // 원본 지도 ↔ 도면
    const map = mapRef.current, { base, plate } = L1.current;
    if (!map || !base) return;
    const [show, hide] = lay.raw ? [base, plate] : [plate, base];
    if (map.hasLayer(hide)) map.removeLayer(hide);
    if (!map.hasLayer(show)) show.addTo(map);          // 둘 다 floor pane 이라 순서를 바꾸지 않음 (bringToBack 하면 칸막이가 흰 바닥 뒤로 숨음)
  }, [lay.raw, orient]);
  useEffect(() => {
    const L = window.L, info = L1.current.info;
    if (!info) return;
    info.clearLayers();
    const g = m.grid_size_m;
    const rect = (x0, y0, o) => L.rectangle([P(x0, y0), P(x0 + g, y0 + g)], o);
    if (lay.zone || lay.grid) {
      (m.cells || []).forEach(([id, ix, iy, sector]) => {
        const col = SECTOR_COLOR[sector] || '#0b7a84';
        rect(m.origin_x + ix * g, m.origin_y + iy * g, { weight: lay.grid ? 1 : 0, color: lay.grid ? '#94a3b8' : col, opacity: 0.7,
          fillColor: col, fillOpacity: lay.zone ? 0.12 : 0, interactive: false }).addTo(info);
      });
    }
    const label = (cls) => ({ permanent: true, direction: 'right', offset: [8, 0], className: 'map-tip ' + cls });
    if (lay.dock) {
      Object.entries(m.dock_poses || {}).forEach(([rid, p]) => {
        // 출발 위치만 알림 — 작은 점 + 글자 (10/15 PM: 큰 사각형 · 테두리는 설비처럼 보여 헷갈림)
        const c = robotColor(rid);
        L.circleMarker(P(p.x, p.y), { radius: 4, weight: 0, fillColor: c, fillOpacity: 0.45 })
          .bindTooltip(`R${rid.replace('robot', '')} 출발`, { permanent: true, direction: 'bottom', offset: [0, 4], className: 'start-tip' }).addTo(info);
      });
    }
    if (lay.fac) {
      (m.aps || []).filter((a) => a.x !== null && a.y !== null).forEach((a) => {
        L.marker(P(a.x, a.y), { icon: L.divIcon({ className: 'ap-icon', iconSize: [18, 18], html: '<b>📶</b>' }) })
          .bindTooltip(`${a.is_target ? '측정 AP · ' : ''}${a.ssid}${a.is_active ? '' : ' (비활성)'}`, { direction: 'top' }).addTo(info);
      });
      const FAC = { extinguisher: ['소', '#dc2626'], shutter: ['셔', '#7c3aed'], exit_light: ['유', '#16a34a'], rack: ['랙', '#b45309'] };
      (m.facilities || []).forEach((f) => {
        if (!f.expected_x && !f.expected_y) return;      // 좌표 미등록 설비는 원점에 그리지 않음
        const [ch, col] = FAC[f.facility_type] || ['?', '#555'];
        const c = f.is_active ? col : '#9aa3aa', v = f.view_pose || {};
        if (lay.dock && v.x !== undefined) {             // 관측 자세 → 설비 (점선)
          L.polyline([P(v.x, v.y), P(f.expected_x, f.expected_y)], { color: c, weight: 1.5, dashArray: '2 3', interactive: false }).addTo(info);
          L.circleMarker(P(v.x, v.y), { radius: 3, color: c, weight: 1, fillColor: c, fillOpacity: 0.6, interactive: false }).addTo(info);
        }
        L.marker(P(f.expected_x, f.expected_y), { icon: L.divIcon({ className: 'fac-icon', iconSize: [20, 20],
          html: `<b style="background:${c}">${ch}</b>` }) })
          .bindTooltip(`${f.facility_id}${f.has_gauge ? ' · 압력계' : ''}${f.is_active ? '' : ' (비활성)'}`, label('fac')).addTo(info);
      });
    }
  }, [m.map_id, orient, layKey, !!L1.current.info]);

  // 로봇 위치 — /ws/ui로 1초마다 받은 값으로 마커를 옮김 (지도·마커는 다시 만들지 않음)
  useEffect(() => {
    const L = window.L, layer = L1.current.robot, mk = markersRef.current;
    if (!layer) return;
    const seen = new Set();
    (robots || []).forEach((r) => {
      const st = r.state;
      if (!st || st.x === null || st.y === null) return;
      seen.add(r.robot_id);
      const live = r.connected;
      const color = live ? robotColor(r.robot_id) : '#8a969e';
      // 정면 화살표 — 몸통(폭 7 cm) + 머리(폭 26 cm), 로봇 중심에서 0.55 m (지도 좌표로 만든 뒤 같은 변환)
      // 받은 yaw 그대로 (도크 위에서도 라이다로 AMCL 이 실제 방향을 잡음 — 뒤집으면 반대가 됨, 10/15 PM 확인)
      const yaw = st.yaw || 0, c = Math.cos(yaw), sn = Math.sin(yaw);
      const at = (f, s2) => P(st.x + c * f - sn * s2, st.y + sn * f + c * s2);
      const head = [at(0, 0.018), at(0.18, 0.018), at(0.18, 0.065), at(0.28, 0), at(0.18, -0.065), at(0.18, -0.018), at(0, -0.018)];
      const tip = `${r.robot_id}${live ? '' : ' (마지막 위치)'}`;
      let mm = mk[r.robot_id];
      if (!mm || !layer.hasLayer(mm.dot)) {
        mm = mk[r.robot_id] = {
          line: L.polygon(head, { color: '#fff', weight: 1.5, fillColor: color, fillOpacity: 1, interactive: false }).addTo(layer),
          dot: L.circleMarker(P(st.x, st.y), { radius: 9, color: '#fff', weight: 2, fillColor: color, fillOpacity: 0.95 })
            .bindTooltip(tip, { permanent: true, direction: 'top', offset: [0, -8], className: 'robot-tip' }).addTo(layer),
        };
      } else {
        mm.line.setLatLngs(head).setStyle({ fillColor: color });
        mm.dot.setLatLng(P(st.x, st.y)).setStyle({ fillColor: color, fillOpacity: live ? 0.95 : 0.6 });
        mm.dot.setTooltipContent(tip);
      }
    });
    Object.keys(mk).forEach((rid) => {                 // 상태가 없어진 로봇은 지움
      if (!seen.has(rid)) { layer.removeLayer(mk[rid].line); layer.removeLayer(mk[rid].dot); delete mk[rid]; }
    });
  }, [robots, orient, !!L1.current.robot]);

  // Nav2 전역 경로 (모니터) — 로봇 색 점선 + 점 (RViz 처럼), 1초마다 다시 그림
  useEffect(() => {
    const L = window.L, layer = L1.current.plan;
    if (!layer) return;
    layer.clearLayers();
    if (!lay.plan) return;
    Object.entries(plans || {}).forEach(([rid, pts]) => {
      const col = PLAN_COLOR, ll = (pts || []).map(([x, y]) => P(x, y));
      if (ll.length > 1) L.polyline(ll, { color: col, weight: 2.5, opacity: 0.9, dashArray: '6 5', interactive: false }).addTo(layer);
      ll.forEach((p) => L.circleMarker(p, { radius: 1.6, weight: 0, fillColor: col, fillOpacity: 0.9, interactive: false }).addTo(layer));
    });
  }, [plans, orient, layKey, !!L1.current.plan]);

  // 히트맵 · 경로 · 표본 · 후보 · 판정 — overlay 가 바뀔 때마다 다시 그림 (지도 · 격자는 그대로)
  useEffect(() => {
    const layer = L1.current.overlay;
    if (!layer) return;
    layer.clearLayers();
    if (overlay) drawOverlay(window.L, layer, m, overlay, P, lay, opacity);
  }, [overlay, m.map_id, orient, layKey, opacity, !!L1.current.overlay]);
  return mini ? html`<div class="map-mini" ref=${ref}></div>` : html`<div id="map" ref=${ref}></div>`;
}

// ---------------------------------------------------------------- 지도 표시 (히트맵 · 레이어)
const CAND_COLOR = { OPEN: '#ea580c', REINSPECTING: '#7c3aed', CONFIRMED: '#b91c1c', CLEARED: '#15803d', UNCONFIRMED: '#64748b' };
const FINAL_COLOR = { normal: '#16a34a', abnormal: '#dc2626', unconfirmed: '#64748b' };
function drawOverlay(L, layer, m, ov, P, lay = {}, opacity = 0.55) {
  const g = m.grid_size_m, byId = {};
  (m.cells || []).forEach(([id, ix, iy, sector]) => { byId[id] = [ix, iy, sector]; });
  const rect = (c, o) => L.rectangle([P(m.origin_x + c[0] * g, m.origin_y + c[1] * g), P(m.origin_x + (c[0] + 1) * g, m.origin_y + (c[1] + 1) * g)], o);
  const best = {};                                     // 칸 → [평균, 표본 수, AP] (선택한 AP, 또는 가장 센 AP)
  Object.entries(ov.heat || {}).forEach(([cid, byAp]) => {
    const vals = Object.entries(byAp).filter(([ap]) => !ov.ap || String(ap) === String(ov.ap));
    if (!vals.length) return;
    const [ap, [mean, n]] = vals.reduce((a, b) => (b[1][0] > a[1][0] ? b : a));
    best[cid] = [mean, n, ap];
  });
  // 히트맵 — 늘 켬. 흐림 pane 에 칸 색을 칠해 부드럽게 (원본 칸 수치는 「셀 원본」에서)
  Object.entries(best).forEach(([cid, [mean]]) => {
    const c = byId[cid];
    if (c) rect(c, { pane: 'heat', weight: 0, fillColor: rssiGrad(mean), fillOpacity: opacity, interactive: false }).addTo(layer);
  });
  if (lay.cells || ov.mini) {                          // 셀 원본 — 칸 대표값 (낮은 농도 · 칸 테두리 · 값)
    Object.entries(best).forEach(([cid, [mean, n, ap]]) => {
      const c = byId[cid];
      if (!c) return;
      rect(c, { weight: 1, color: rssiGrad(mean), opacity: 0.9, fillColor: rssiGrad(mean), fillOpacity: 0.12 })
        .bindTooltip(`칸 ${cid} (${c[0]},${c[1]}) · ${(ov.apName || {})[ap] || ap} ${mean == null ? '검출값 없음' : '평균 ' + mean + ' dBm'} · 표본 ${n}`, { direction: 'top' }).addTo(layer);
    });
  }
  if (lay.grid) {                                      // 격자 · 값 — 칸 번호 위에 대표값
    (m.cells || []).forEach(([id, ix, iy]) => {
      const v = best[id];
      L.marker(P(m.origin_x + (ix + 0.5) * g, m.origin_y + (iy + 0.5) * g), { interactive: false, keyboard: false,
        icon: L.divIcon({ className: 'cell-val', iconSize: null, html: v ? `<b>${Math.round(v[0])}</b>` : `<small>${id}</small>` }) }).addTo(layer);
    });
  }
  if (lay.unmeasured) {                                // 미측정 — 로봇이 지나간 칸(방문 궤적) 중 신호 표본이 없는 칸만 회색. 안 간 칸은 그대로 둠
    const visited = new Set();
    Object.values(ov.tracks || {}).forEach((pts) => pts.forEach(([, x, y]) => visited.add(`${Math.floor((x - m.origin_x) / g)},${Math.floor((y - m.origin_y) / g)}`)));
    (m.cells || []).forEach(([id, ix, iy]) => {
      if (!visited.has(`${ix},${iy}`) || best[id]) return;
      rect([ix, iy], { weight: 1, color: '#94a3b8', fillColor: '#cbd5e1', fillOpacity: 0.8, dashArray: null })
        .bindTooltip(`칸 ${id} (${ix},${iy}) · 미측정 — 지나갔지만 신호 표본 없음`, { direction: 'top' }).addTo(layer);
    });
  }
  if (lay.track) {                                     // 방문 궤적 — 실선, 5초 넘게 끊긴 곳은 잇지 않음
    Object.entries(ov.tracks || {}).forEach(([rid, pts]) => {
      const col = robotColor(rid);
      let seg = [];
      const flush = () => { if (seg.length > 1) L.polyline(seg, { color: col, weight: 3, opacity: 0.85, interactive: false }).addTo(layer); seg = []; };
      pts.forEach(([t, x, y], i) => { if (i && t - pts[i - 1][0] > 5) flush(); seg.push(P(x, y)); });
      flush();
    });
  }
  if (lay.plan) {                                      // 계획 경로 — task 경로점 (진행 중 진하게 · 대기 연하게 · 끝난 것은 이력에서만)
    (ov.tasks || []).forEach((t) => {
      if (!(t.waypoints || []).length) return;
      if (ov.live && !['ACTIVE', 'PENDING'].includes(t.status)) return;
      const col = PLAN_COLOR, pts = t.waypoints.map((p) => P(p.x, p.y)), act = t.status === 'ACTIVE';
      L.polyline(pts, { color: col, weight: act ? 2.5 : 1.5, opacity: act ? 0.75 : 0.4, dashArray: '5 6', interactive: false }).addTo(layer);
    });
  }
  if (lay.samples) {                                   // NetSample — 개별 표본 점 (위치 불확실 = 속 빈 회색)
    (ov.points || []).forEach((s) => {
      const okp = s.pose_ok;
      L.circleMarker(P(s.x, s.y), { radius: 3, weight: okp ? 0 : 1.2, color: '#64748b', fill: okp, fillColor: s.detected === false ? '#94a3b8' : rssiGrad(s.rssi_dbm), fillOpacity: 0.95 })
        .bindTooltip(`${s.robot_id} · seq ${s.seq} · ${hm(s.stamp)}<br>(${fmtNum(Math.round(s.x * 100) / 100)}, ${fmtNum(Math.round(s.y * 100) / 100)}) · ${s.detected === false ? 'AP 미검출' : s.rssi_dbm + ' dBm'}${okp ? '' : ' · 위치 불확실(칸 집계 제외)'}${s.task_id ? '<br>' + s.task_id : ''}`, { direction: 'top' }).addTo(layer);
    });
  }
  if (lay.failed) {                                    // 도달 실패 — 붉은 테두리 칸 + ×, 실행 사유
    const seen = {};
    failedSpots(ov.tasks).forEach((f) => {
      const ix = Math.floor((f.x - m.origin_x) / g), iy = Math.floor((f.y - m.origin_y) / g), k = `${ix},${iy}`;
      (seen[k] = seen[k] || []).push(f);
    });
    Object.entries(seen).forEach(([k, fs]) => {
      const [ix, iy] = k.split(',').map(Number);
      rect([ix, iy], { weight: 2.5, color: '#dc2626', fillColor: '#fecaca', fillOpacity: 0.35 })
        .bindTooltip(`도달 실패 칸 (${k})<br>${fs.map((f) => `${f.robot_id} ${f.task_id} · 경로점 ${f.kind}`).join('<br>')}<br>${fs[0].reason}`, { direction: 'top' }).addTo(layer);
      L.marker(P(m.origin_x + (ix + 0.5) * g, m.origin_y + (iy + 0.5) * g), { interactive: false, keyboard: false,
        icon: L.divIcon({ className: 'fail-x', iconSize: [16, 16], html: '×' }) }).addTo(layer);
    });
  }
  if (ov.path && ov.path.pts.length) {                 // task 「자세히」 — 계획 경로점(점선 · 번호) · 실제로 지나간 길(실선)
    const pts = ov.path.pts.map((p) => P(p.x, p.y)), col = ov.path.color;
    L.polyline(pts, { color: col, weight: 1.5, opacity: 0.6, dashArray: '4 4', interactive: false }).addTo(layer);
    pts.forEach((p, i) => L.marker(p, { keyboard: false, icon: L.divIcon({ className: 'wp-num', iconSize: [16, 16],
      html: `<b style="border-color:${col};color:${col}">${i + 1}</b>` }) })
      .bindTooltip(`경로점 ${i + 1}`, { direction: 'top' }).addTo(layer));
  }
  if (ov.track && ov.track.pts.length) {
    const pts = ov.track.pts.map((p) => P(p.x, p.y)), col = ov.track.color;
    L.polyline(pts, { color: col, weight: 3.5, opacity: 0.9, interactive: false }).addTo(layer);
    L.circleMarker(pts[0], { radius: 5, color: col, weight: 2, fillColor: '#fff', fillOpacity: 1 })
      .bindTooltip('시작', { direction: 'top' }).addTo(layer);
    L.circleMarker(pts[pts.length - 1], { radius: 5, color: '#fff', weight: 2, fillColor: col, fillOpacity: 1 })
      .bindTooltip('끝', { direction: 'top' }).addTo(layer);
  }
  (ov.candidates || []).forEach((c) => {               // 후보 — 상태 · 결론 색
    if (c.target_x === null) return;
    const col = CAND_COLOR[c.outcome || c.status] || '#ea580c';
    L.circleMarker(P(c.target_x, c.target_y), { radius: 7, color: '#fff', weight: 2, fillColor: col, fillOpacity: 0.95 })
      .bindTooltip(`${c.candidate_id} · 칸 ${c.ref} · ${c.outcome || c.status} · ${fmtNum(c.score)} dBm`, { direction: 'top' }).addTo(layer);
  });
  (ov.inspections || []).forEach((i) => {             // 설비 판정 — 설비 위치에 결과 색 테두리
    L.circleMarker(P(i.expected_x, i.expected_y), { radius: 13, color: FINAL_COLOR[i.final] || '#64748b', weight: 3, fill: false })
      .bindTooltip(`${i.facility_id} · ${i.detail} · ${i.final}`, { direction: 'bottom' }).addTo(layer);
  });
}

// /ws/ui — 로봇 상태 1초마다. 끊기면 2초 뒤 다시 접속, 로그인이 풀렸으면(4401) 로그인 화면으로
function useUiSocket(onAuthLost) {
  const [data, setData] = useState(null);
  const [linked, setLinked] = useState(false);
  useEffect(() => {
    let ws, timer, alive = true;
    const open = () => {
      ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws/ui`);
      ws.onopen = () => setLinked(true);
      ws.onmessage = (e) => { try { const d = JSON.parse(e.data); if (d.type === 'robots') setData(d); } catch { /* 무시 */ } };
      ws.onclose = (e) => {
        setLinked(false);
        if (!alive) return;
        if (e.code === 4401) { onAuthLost(); return; }
        timer = setTimeout(open, 2000);
      };
    };
    open();
    return () => { alive = false; clearTimeout(timer); ws && ws.close(); };
  }, []);
  return [data, linked];
}

const ago = (iso) => {
  if (!iso) return '—';
  const s = Math.max(0, (Date.now() - new Date(iso).getTime()) / 1000);
  return s < 60 ? `${s.toFixed(0)}초 전` : `${(s / 60).toFixed(0)}분 전`;
};

// 로봇 카드 (10/15 UI 시안) — 지도 위 가로 카드: 담당 구역 · 현재 작업 · 진행률 · 배터리 · 대기 사유. 미수신 값은 — 로
// 로봇 카드 설비 점검 (10/15): 이번 회차에서 이 로봇의 가장 최근 판정 — 새 결과가 오면 12초 동안 강조(팝업)
const FINAL_LABEL = { normal: '정상', abnormal: '이상', unconfirmed: '판정 못 함' };
const EXPIRY_LABEL = { VALID: '유효', EXPIRED: '만료', UNKNOWN: '미확인' };
// 소화기 ArUco 제조 정보 한 줄씩: ID · 제조년월 · 제조번호 · 만료 상태 (inspections.aruco)
const arucoLines = (list) => (list || []).map((a) => html`<small key=${a.marker_id} class="aruco">
  ArUco ${a.marker_id} · 제조 ${(a.manufacture_date || '—').slice(0, 7)} · ${a.manufacture_number || '—'} · <b class=${'exp ' + (a.expiry_status || '')}>${EXPIRY_LABEL[a.expiry_status] || a.expiry_status || '—'}</b></small>`);
function InspectionPop({ list }) {
  const last = list.length ? list[list.length - 1] : null;
  const [fresh, setFresh] = useState(false);
  const seen = useRef(null);
  useEffect(() => {
    if (!last || seen.current === last.inspection_id) return;
    const first = seen.current === null;
    seen.current = last.inspection_id;
    if (first && Date.now() - new Date(last.inspected_at) > 15000) return;   // 화면을 연 때 이미 있던 결과는 강조하지 않음
    setFresh(true);
    const h = setTimeout(() => setFresh(false), 12000);
    return () => clearTimeout(h);
  }, [last && last.inspection_id]);
  if (!last) return html`<div class="rinsp none"><span>설비 점검 결과가 여기에 뜹니다</span></div>`;
  const photo = (last.photos || [])[0];
  return html`<div class=${'rinsp ' + (fresh ? 'fresh' : '')} style=${{ borderColor: FINAL_COLOR[last.final] }}>
    ${photo ? html`<a href=${'/media/' + photo} target="_blank"><img src=${'/media/' + photo} alt="" /></a>` : html`<div class="noimg">사진 없음</div>`}
    <div class="rinsp-t">
      <b>${last.facility_id}</b> <small>${FAC_LABEL[last.facility_type] || last.facility_type || ''}</small>
      <span class="cand" style=${{ background: FINAL_COLOR[last.final] }}>${FINAL_LABEL[last.final] || last.final}</span>
      <small class="no">${last.detail} · ${fmtTime(last.inspected_at)}</small>
      ${arucoLines(last.aruco)}
      <small class="no">이번 순찰 점검 ${list.length}건 · 이상 ${list.filter((i) => i.final === 'abnormal').length}</small>
    </div></div>`;
}

function RobotCard({ r, sector, why, timeout, insp }) {
  const [k, label] = robotPhase(r), st = r.state;
  const prog = st && st.progress !== null && st.progress !== undefined ? Math.round(st.progress * 100) : null;
  const bat = st && st.battery_pct !== null && st.battery_pct !== undefined ? st.battery_pct : null;
  const col = robotColor(r.robot_id);
  return html`<div class="rcard">
    <div class="rch"><span class="badge" style=${{ background: col + '1a', color: col }}>R${r.robot_id.replace('robot', '')}</span>
      <div><b>${r.robot_id}</b><small>${sector ? `섹터 ${sector}` : '배정 없음'}</small></div>
      <span class=${'chip ' + k}>${label}</span></div>
    ${st ? html`<div class="rcb">
        <div class="rcl"><span>${st.nav_state || '—'}${st.is_docked ? ' · 도킹됨' : ''}${st.estop ? ' · 정지' : ''}</span>
          <span class="no">${st.task_id || (st.run_id ? `run ${st.run_id}` : '작업 없음')}</span></div>
        <div class="rcbar"><span>작업 진행</span><i><b style=${{ width: (prog || 0) + '%', background: col }}></b></i><span class="num">${prog === null ? '—' : prog + '%'}</span></div>
        <div class="rcbar"><span>배터리</span><i><b style=${{ width: (bat || 0) + '%', background: bat !== null && bat <= 30 ? '#dc2626' : '#0f766e' }}></b></i><span class="num">${bat === null ? '—' : bat.toFixed(0) + '%'}</span></div>
        <div class="rcl small"><span>위치 ${fmtNum(st.x)}, ${fmtNum(st.y)}${st.yaw === null ? '' : ' · ' + (st.yaw * 180 / Math.PI).toFixed(0) + '°'}</span>
          <span class="no">${r.connected ? '마지막 보고 ' + ago(st.ts) : '마지막 위치 · ' + ago(st.ts)}</span></div>
        <${InspectionPop} list=${insp || []} />
        ${why && html`<div class="rcwhy">${why}</div>`}
        ${r.is_active && st.raw && html`<details class="raw"><summary>관제 메모리 상태 · E3까지 ${st.ttl_s}초</summary>
          <div class="rkv mono">${Object.entries(st.raw).map(([kk, v]) => html`<span key=${kk}>${kk}</span><span>${v === '' ? '—' : v}</span>`)}</div></details>`}
      </div>`
      : html`<p class="no">${r.last_e3 ? `${fmtTime(r.last_e3)} 응답 없음(E3) 기록` : `미수신 — ${timeout || '?'}초 안에 상태가 오지 않음`}</p>`}
  </div>`;
}

function MonitorPage({ onAuthLost }) {
  const [state, setState] = useState({ loading: true });
  const [ui, linked] = useUiSocket(onAuthLost);
  const [starting, setStarting] = useState(false);
  const [msg, setMsg] = useState('');
  const [run, setRun] = useState(null);               // 진행 중 회차 — 히트맵 · 방문 궤적 · task 경로 (2초)
  const [lay, setLay] = useLayers('scv_layers_monitor', MONITOR_DEFAULT);
  const [opacity, setOpacity] = useState(0.55);
  const abort = async (runId) => {
    const ros = ui && ui.run_owner === 'ros';          // ROS 관제: 중단 = 비상정지(두 로봇 정지, 「재개」로 이어 감)
    if (!confirm(ros ? `순찰 run ${runId}을 비상정지할까요? 두 로봇이 멈추고 「재개」를 기다립니다.`
      : `순찰 run ${runId}을 중단할까요? 로봇은 하던 일을 멈추고 도크로 돌아갑니다.`)) return;
    try { await api(`/api/runs/${runId}/abort`, { method: 'POST' }); setMsg(''); } catch (e) { setMsg(e.message); }
  };
  const stop = async (runId) => {                     // 순찰 종료 — 회차를 aborted 로 닫고 로봇은 도크로 귀환 (비상정지 = 그 자리 정지)
    if (!confirm(`순찰 run ${runId}을 종료할까요? 남은 작업은 취소되고 회차가 「중단」으로 닫힙니다. 로봇은 도크로 돌아갑니다.`)) return;
    try { await api(`/api/runs/${runId}/stop`, { method: 'POST' }); setMsg(''); } catch (e) { setMsg(e.message); }
  };
  const resume = async () => {                        // ROS 관제(SCV_RUN_OWNER=ros) — 비상정지 뒤 /inspection/resume
    try { await api('/api/runs/resume', { method: 'POST' }); setMsg(''); } catch (e) { setMsg(e.message); }
  };
  useEffect(() => {
    api('/api/maps/active')
      .then((d) => setState({ map: d.map }))
      .catch((e) => (e.status === 401 ? onAuthLost() : setState({ error: e.message })));
  }, []);
  const runId = ui && ui.run_id;
  useEffect(() => {
    if (!runId) { setRun(null); return; }
    let alive = true;
    const tick = () => api(`/api/runs/${runId}?lite=1`).then((x) => alive && setRun(x)).catch(() => {});
    tick();
    const t = setInterval(tick, 2000);
    return () => { alive = false; clearInterval(t); };
  }, [runId]);
  const m = state.map;
  const st = ui && ui.start;
  const ready = st ? st.robots.filter((r) => r.ready).map((r) => r.robot_id) : [];
  const why = !st ? '' : runId ? '' : st.problems.length ? st.problems.join(' · ')
    : !ready.length ? st.robots.map((r) => `${r.robot_id}: ${r.why.join(' · ')}`).join(' / ') : '';
  const whyOf = Object.fromEntries(((st && st.robots) || []).map((r) => [r.robot_id, runId ? '' : (r.why || []).join(' · ')]));
  const sectorOf = {};                                 // 로봇 → 섹터 (진행 중 회차 배정, 없으면 기본)
  Object.entries((run && run.run && run.run.sector_robots) || {}).forEach(([s, rid]) => { if (rid) sectorOf[rid] = s; });
  ((m && m.sectors) || []).forEach((x) => { if (x.robot && !sectorOf[x.robot]) sectorOf[x.robot] = x.sector; });
  const target = run ? ((m && m.cells) || []).filter((c) => Object.keys(run.run.sector_robots || {}).includes(String(c[3]))).length : 0;
  const measured = run ? Object.keys(run.heat || {}).length : 0;
  const apName = {};
  ((run && run.aps) || []).forEach((a) => { apName[a.ap_id] = a.ssid; });
  const overlay = run ? { heat: run.heat, apName, thr: run.thr, tasks: run.tasks, tracks: run.tracks, live: true,
    candidates: run.candidates, inspections: run.inspections } : null;
  const ros = ui && ui.run_owner === 'ros';
  return html`<main class="mon">
    <div class="pagehead">
      <div><h2>실시간 관제</h2><p class="sub">두 로봇의 현재 작업과 수집 중인 신호를 한 지도에서 확인합니다.</p></div>
      <div class="ctl">
        <span class=${'live ' + (linked ? 'on' : '')}>${linked ? '● 실시간' : '연결 중…'}</span>
        ${runId ? html`<button class="btn danger" disabled=${ui.estopped} onClick=${() => abort(runId)}>${ros ? (ui.estopped ? '비상정지 중' : '비상정지') : '순찰 중단'}</button>
          ${ros && ui.estopped && html`<button class="btn primary" onClick=${resume}>재개</button>`}
          ${ros && html`<button class="btn" onClick=${() => stop(runId)}>순찰 종료</button>`}`
          : html`<button class="btn primary" disabled=${!st || !st.can_start} title=${why} onClick=${() => setStarting(true)}>순찰 시작</button>`}
      </div>
    </div>
    ${!runId && why && html`<p class="startwhy">순찰 시작 불가 — ${why}</p>`}
    ${msg && html`<div class="msg">${msg}</div>`}
    <div class="sumbar">
      <span>지도 <b>${m ? m.name : '—'}</b></span>
      <span>회차 <b>${runId ? `#${runId} ${ui.estopped ? '비상정지 중' : '진행 중'}` : '없음'}</b></span>
      <span>측정 대상 <b>${runId ? `${measured} / ${target}칸` : '—'}</b></span>
      <span>표본 <b>${run ? run.samples : '—'}</b></span>
      ${st && st.target_ap && html`<span>측정 AP <b>${st.target_ap.ssid}</b>${st.target_ap.active ? '' : ' (미수신)'}</span>`}
    </div>
    <h3 class="sect">로봇 현황 · ${ui ? ui.robots.length : 0}대</h3>
    <div class="rcards">${ui && ui.robots.length ? ui.robots.map((r) => html`<${RobotCard} key=${r.robot_id} r=${r}
      sector=${sectorOf[r.robot_id]} why=${whyOf[r.robot_id]} timeout=${ui.timeout_s}
      insp=${((run && run.inspections) || []).filter((i) => i.robot_id === r.robot_id)} />`) : html`<p class="no">등록된 로봇이 없습니다.</p>`}</div>
    <div class="mapbox">
      <div class="head">
        <b>현재 점검 지도</b>
        ${m && html`<span>해상도 ${fmtNum(m.resolution_m)} m/px · 격자 ${fmtNum(m.grid_size_m)} m · ${m.cell_count}칸</span>`}
        <${OrientSelect} />
      </div>
      <${LayerBar} keys=${MONITOR_LAYERS} on=${lay} setOn=${setLay} />
      ${state.loading ? html`<div class="empty">불러오는 중…</div>`
        : state.error ? html`<div class="empty">${state.error}</div>`
        : !m ? html`<div class="empty">현재 지도가 없습니다. 정보 › 지도에서 지도를 등록하고 「현재 지도로」를 누르세요.</div>`
        : html`<${MapView} m=${m} robots=${ui ? ui.robots : []} plans=${ui ? ui.plans : null} overlay=${overlay} layers=${lay} opacity=${opacity} />`}
      <${MapLegend} opacity=${opacity} setOpacity=${setOpacity} />
    </div>
    <div class="cols2">
      <div class="panel"><h3>이번 순찰 이벤트</h3>${runId ? html`<${EventList} events=${ui.events} max=${12} />`
        : html`<p class="no">순찰 중일 때 그 순찰의 이벤트만 보입니다.</p>`}</div>
      <div class="panel"><h3>수집 · 저장 상태</h3>
        ${runId ? html`<dl class="kv">
          <dt>측정 칸</dt><dd>${measured} / ${target}칸 (표본이 붙은 칸 / 배정 섹터 칸)</dd>
          <dt>저장된 표본</dt><dd>${run ? run.samples : '—'}건 (DB net_samples)</dd>
          <dt>로봇별 작업</dt><dd>${Object.entries(((run && run.tasks) || []).reduce((a, t) => { const b = a[t.robot_id] = a[t.robot_id] || [0, 0]; b[0] += 1; if (['DONE', 'FAILED', 'CANCELED'].includes(t.status)) b[1] += 1; return a; }, {}))
            .map(([rid, [n, d]]) => `${rid} ${d}/${n}`).join(' · ') || '—'}</dd>
        </dl>` : html`<p class="no">진행 중인 순찰 없음 — 위 「순찰 시작」으로 시작합니다.</p>`}
        <p class="no">${ui ? ui.timeout_s : '—'}초 동안 상태가 오지 않으면 응답 없음(E3) · 수신과 저장 확인은 따로 표시합니다.</p></div>
    </div>
    ${starting && html`<${StartModal} onClose=${() => setStarting(false)} onStarted=${() => setStarting(false)} />`}
  </main>`;
}


// ---------------------------------------------------------------- 순찰 시작 창 · 이력 (3-3차)
const TYPE_LABEL = { SURVEY_SEGMENT: '측정', REINSPECTION: '정밀측정', FACILITY_CHECK: '탐지', DOCK: '복귀' };
const TASK_FILTERS = [['', '전체'], ['SURVEY_SEGMENT', '측정'], ['REINSPECTION', '정밀측정'], ['FACILITY_CHECK', '탐지'], ['DOCK', '복귀']];
const STATUS_CHIP = { running: 'ok', done: 'ok', partial: 'wait', aborted: 'bad',
  ACTIVE: 'ok', PENDING: 'off', DONE: 'ok', FAILED: 'bad', CANCELED: 'off' };
const RUN_TYPE_LABEL = { patrol: '순찰' };          // 순찰 종류는 순찰 하나 (DB의 이전 demo · remeasure 회차는 값 그대로 표시)
const RUN_LABEL = { running: '진행 중', done: '완료', partial: '일부 섹터 실패', aborted: '중단 · 실패' };
const pct = (v) => (v === null || v === undefined ? '—' : `${Math.round(v * 100)}%`);
const hm = (v) => {
  if (!v) return '—';
  const d = new Date(v), p = (n) => String(n).padStart(2, '0');
  return `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
};

function EventList({ events, max = 12 }) {
  const ev = (events || []).slice(-max).reverse();
  if (!ev.length) return html`<p class="no">이벤트 없음</p>`;
  return html`<ul class="evlist">${ev.map((e) => html`<li key=${e.event_id} class=${'lv-' + e.level}>
    <span class="t">${hm(e.ts)}</span><b>${e.code}</b><span>${e.robot_id}${e.task_id ? ' · ' + e.task_id : ''}</span>
    <small>${e.detail ? Object.entries(e.detail).filter(([k]) => k !== 'medians').map(([k, v]) => `${k} ${typeof v === 'object' ? JSON.stringify(v) : v}`).join(' · ') : ''}</small>
  </li>`)}</ul>`;
}

function StartModal({ onClose, onStarted }) {
  const [form, setForm] = useState(null);
  const [sel, setSel] = useState({});
  const [msg, setMsg] = useState('');
  const [busy, setBusy] = useState(false);
  useEffect(() => {
    api('/api/runs/form').then((f) => { setForm(f); setSel(f.default || {}); }).catch((e) => setMsg(e.message));
  }, []);
  const submit = async () => {
    setBusy(true); setMsg('');
    try { const r = await api('/api/runs', { json: { sector_robots: sel } }); onStarted(r.run_id); }
    catch (e) { setMsg(e.message); }
    setBusy(false);
  };
  return html`<div class="backdrop"><div class="modal narrow">
    <div class="mh"><b>순찰 시작</b><code>${form && form.map ? form.map.name : ''}</code>
      <button class="btn small x" onClick=${onClose}>닫기</button></div>
    <div class="mdesc">섹터마다 순찰할 로봇을 고릅니다. 정상 · 도킹 중인 로봇만 고를 수 있고, 로봇 1대 = 섹터 1개입니다.
      현재 지도 · 격자와 활성 측정 AP가 있어야 합니다.
      로봇은 측정 → 후보 칸 정밀측정 → 설비 탐지(1회 판정) → 복귀를 차례로 합니다.</div>
    <div class="body">${!form ? html`<p>불러오는 중…</p>` : html`
      <table class="grid"><thead><tr><th>섹터</th><th>칸</th><th>로봇</th></tr></thead><tbody>
        ${form.sectors.map((x) => html`<tr key=${x.sector}>
          <td><i class="sec" style=${{ background: SECTOR_COLOR[x.sector] }}></i> 섹터 ${x.sector}</td><td>${x.cells}칸</td>
          <td><select value=${sel[x.sector] || ''} onChange=${(e) => setSel({ ...sel, [x.sector]: e.target.value || null })}>
            <option value="">순찰 안 함</option>
            ${form.robots.map((r) => html`<option key=${r.robot_id} value=${r.robot_id} disabled=${!r.ready}>
              ${r.robot_id}${r.ready ? '' : ` (${r.why.join(' · ') || '불가'})`}</option>`)}
          </select></td></tr>`)}
      </tbody></table>
      ${form.target_ap && html`<p class="no">측정 AP ${form.target_ap.ssid} — ${form.target_ap.active ? '활성' : '비활성'}</p>`}
      ${form.problems.length > 0 && html`<p class="lockmsg">순찰 시작 불가 — ${form.problems.join(' · ')}</p>`}
      ${form.running && html`<p class="lockmsg">순찰 run ${form.running}이 진행 중입니다.</p>`}`}
    </div>
    ${msg && html`<div class="msg">${msg}</div>`}
    <div class="tools"><span class="sp"></span>
      <button class="btn primary" disabled=${busy || !form || !form.can_start} onClick=${submit}>순찰 시작</button></div>
  </div></div>`;
}

// 이력 › 블랙박스 (10/15): 주행 중 로봇 카메라 1 Hz 프레임 (비전 로봇은 검출을 그린 화면). 로봇 골라 넘겨 보기 · 재생(4배속)
function Recording({ runId }) {
  const [rec, setRec] = useState(null);
  const [rid, setRid] = useState('');
  const [i, setI] = useState(0);
  const [play, setPlay] = useState(false);
  useEffect(() => {
    setRec(null); setI(0); setPlay(false);
    if (runId) api(`/api/runs/${runId}/recording`).then((x) => { setRec(x); setRid(Object.keys(x)[0] || ''); }).catch(() => setRec({}));
  }, [runId]);
  const frames = (rec && rec[rid]) || [];
  useEffect(() => {
    if (!play) return;
    const h = setInterval(() => setI((v) => (v + 1 < frames.length ? v + 1 : (setPlay(false), v))), 250);
    return () => clearInterval(h);
  }, [play, frames.length]);
  const f = frames[Math.min(i, frames.length - 1)];
  return html`<div class="panel"><div class="ttl"><b>주행 영상 기록 (블랙박스)</b><code>recordings</code><span class="n">${frames.length}</span></div>
    ${!rec ? html`<p class="no">불러오는 중…</p>` : !Object.keys(rec).length ? html`<p class="no">이 순찰의 영상 기록이 없습니다 (주행 중 1초에 한 장 저장).</p>` : html`
      <div class="recbar">
        ${Object.keys(rec).map((k) => html`<button key=${k} class=${'btn ' + (k === rid ? 'primary' : '')} onClick=${() => { setRid(k); setI(0); setPlay(false); }}>${k} · ${rec[k].length}장</button>`)}
        <button class="btn" disabled=${!frames.length} onClick=${() => setPlay(!play)}>${play ? '멈춤' : '재생 ×4'}</button>
        <span class="mono">${f ? new Date(f[0]).toLocaleTimeString('ko-KR', { hour12: false }) : '—'} · ${frames.length ? i + 1 : 0} / ${frames.length}</span>
      </div>
      <input class="recslider" type="range" min="0" max=${Math.max(0, frames.length - 1)} value=${i} onInput=${(e) => { setPlay(false); setI(+e.target.value); }} />
      ${f && html`<div class="recview"><img src=${f[1]} alt="" /></div>`}`}
  </div>`;
}

function HistoryPage({ onAuthLost }) {
  const [m, setM] = useState(null);
  const [events, setEvents] = useState([]);
  const [runs, setRuns] = useState({ runs: [], current: null });
  const [runId, setRunId] = useState(null);
  const [d, setD] = useState(null);
  const [filter, setFilter] = useState('');
  const [ap, setAp] = useState('');
  const [msg, setMsg] = useState('');
  const [detail, setDetail] = useState(null);       // 「자세히」 task_id
  const [lay, setLay] = useLayers('scv_layers_history', HISTORY_DEFAULT);
  const [opacity, setOpacity] = useState(0.55);
  const fail = (e) => (e.status === 401 ? onAuthLost() : setMsg(e.message));
  const loadRuns = () => api('/api/runs').then((r) => { setRuns(r); return r; }).catch(fail);
  useEffect(() => {                                  // 현재 지도의 순찰 목록 — 진행 중 순찰이 생기거나 끝나면 5초 안에 반영
    api('/api/maps/active').then((x) => setM(x.map)).catch(fail);
    const f = () => loadRuns().then((r) => r && setRunId((v) => v ?? r.current ?? (r.runs[0] && r.runs[0].run_id) ?? null));
    f();
    const t = setInterval(f, 5000);
    return () => clearInterval(t);
  }, []);
  useEffect(() => {
    if (!runId) { setD(null); setEvents([]); return; }
    let alive = true;
    const tick = () => {
      api(`/api/runs/${runId}`).then((x) => alive && setD(x)).catch(fail);
      api(`/api/runs/${runId}/events`).then((x) => alive && setEvents(x.events)).catch(fail);
    };
    tick();
    const t = setInterval(tick, 2000);
    return () => { alive = false; clearInterval(t); };
  }, [runId]);
  const run = d && d.run;
  const apName = {};
  ((d && d.aps) || []).forEach((a) => { apName[a.ap_id] = a.ssid; });
  const sameMap = m && run && m.map_id === run.map_id;
  const overlay = d && sameMap ? { heat: d.heat, ap, apName, thr: d.thr, tasks: d.tasks, candidates: d.candidates, inspections: d.inspections,
    tracks: d.tracks, points: d.points, sectors: Object.entries(run.sector_robots || {}).filter(([, v]) => v).map(([k]) => k) } : null;
  const nFail = d ? failedSpots(d.tasks).length : 0;
  const tasks = ((d && d.tasks) || []).filter((t) => !filter || t.type === filter);
  const byRobot = {};
  ((d && d.tasks) || []).forEach((t) => {
    const b = byRobot[t.robot_id] = byRobot[t.robot_id] || { n: 0, done: 0, active: null };
    b.n += 1; if (['DONE', 'FAILED', 'CANCELED'].includes(t.status)) b.done += 1; if (t.status === 'ACTIVE') b.active = t;
  });
  return html`<main>
    <h2>이력</h2>
    <p class="sub">현재 지도(${m ? m.name : '없음'})에서 수행된 순찰의 결과를 봅니다 — task(측정 · 정밀측정 · 탐지 · 복귀) · RSSI 히트맵 · 후보 · 설비 판정 · 이벤트. 순찰 시작 · 중단은 모니터에서 합니다.</p>
    <div class="runbar panel">
      <select value=${runId || ''} onChange=${(e) => setRunId(Number(e.target.value) || null)}>
        ${!runs.runs.length && html`<option value="">이 지도의 순찰 기록 없음</option>`}
        ${runs.runs.map((r) => html`<option key=${r.run_id} value=${r.run_id}>run ${r.run_id} · ${RUN_LABEL[r.status] || r.status} · ${fmtTime(r.started_at)}</option>`)}
      </select>
      ${run && html`<span class=${'chip ' + (STATUS_CHIP[run.status] || 'off')}>${RUN_LABEL[run.status] || run.status}</span>
        <span>${run.map_name} · ${RUN_TYPE_LABEL[run.run_type] || run.run_type} · 배정 ${Object.entries(run.sector_robots).map(([k, v]) => `섹터 ${k} = ${v}`).join(', ')}</span>
        <span class="no">시작 ${fmtTime(run.started_at)}${run.started_by_name ? ' · ' + run.started_by_name : ''}${run.ended_at ? ' · 종료 ' + fmtTime(run.ended_at) : ''}</span>
        ${run.is_baseline_ok && html`<span class="chip ok">기준선 재료</span>`}`}
    </div>
    ${msg && html`<div class="msg">${msg}</div>`}
    <div class="mapbox">
      <div class="head">
        ${run ? html`<b>run ${run.run_id}</b>` : html`<b>${m ? m.name : '지도 없음'}</b>`}
        ${Object.entries(byRobot).map(([rid, b]) => html`<span key=${rid}><i class="dot" style=${{ background: robotColor(rid) }}></i>
          ${rid} ${b.done}/${b.n}${b.active ? ` · ${TYPE_LABEL[b.active.type]} ${b.active.task_id} ${pct(b.active.progress)}` : ''}</span>`)}
        ${d && html`<span>표본 ${d.samples}${(d.points || []).length < d.samples ? ` (지도 ${d.points.length})` : ''} · 도달 실패 ${nFail}점</span>`}
        <span class="lg">히트맵 AP <select class="mini" value=${ap} onChange=${(e) => setAp(e.target.value)}>
          <option value="">가장 센 AP</option>${((d && d.aps) || []).map((a) => html`<option key=${a.ap_id} value=${a.ap_id}>${a.ssid}</option>`)}
        </select></span>
        <${OrientSelect} />
      </div>
      <${LayerBar} keys=${LAYERS.map(([k]) => k)} on=${lay} setOn=${setLay}
        disabled=${{ keepout: '회피 구역 기록이 아직 DB에 없습니다 (manager keepout 마스크는 저장하지 않음)' }}
        extra=${html`<span class="lyrbtns"><button class="btn small" onClick=${() => setLay({ ...ALL_ON, keepout: false, raw: false })}>전체 표시</button>
          <button class="btn small" onClick=${() => setLay(HISTORY_DEFAULT)}>기본 보기</button></span>`} />
      ${!m ? html`<div class="empty">현재 지도가 없습니다.</div>`
        : html`<${MapView} m=${m} robots=${[]} overlay=${overlay} layers=${lay} opacity=${opacity} />`}
      ${run && !sameMap && m && html`<div class="empty">이 run은 다른 지도(${run.map_name})라 지도 위 표시는 생략합니다.</div>`}
      <${MapLegend} opacity=${opacity} setOpacity=${setOpacity} history=${true} />
      <div class="maplegend"><span class="lk">● 후보 (주황 열림 · 보라 정밀측정 중 · 빨강 확정 · 초록 해제)</span>
        <span class="lk">◯ 설비 판정 (초록 정상 · 빨강 이상 · 회색 미확인)</span>
        <span class="lk">후보 기준 ${d ? d.thr : -72} dBm</span></div>
    </div>
    <div class="cols2">
      <div class="panel"><h3>이 순찰의 이벤트 <span class="no">${events.length}건</span></h3><${EventList} events=${events} max=${200} /></div>
      <div class="panel"><h3>NetSample 원본 <span class="no">${d ? (d.points || []).length : 0}건 · 최근 순</span></h3>
        ${d && (d.points || []).length ? html`<div class="scrollbox"><table class="grid"><thead><tr><th>로봇</th><th>seq</th><th>시각</th><th>x, y</th><th>위치</th><th>RSSI</th><th>칸</th><th>task</th></tr></thead>
          <tbody>${d.points.slice(-300).reverse().map((p) => html`<tr key=${p.robot_id + p.seq}><td>${p.robot_id}</td><td class="mono">${p.seq}</td><td>${hm(p.stamp)}</td>
            <td class="num">${fmtNum(Math.round(p.x * 100) / 100)}, ${fmtNum(Math.round(p.y * 100) / 100)}</td><td>${p.pose_ok ? '유효' : html`<span class="no">불확실</span>`}</td>
            <td>${dbm(p.rssi_dbm, d.thr)}</td><td>${p.cell_id ?? '—'}</td><td class="mono">${p.task_id || '—'}</td></tr>`)}</tbody></table></div>`
          : html`<p class="no">표본 없음</p>`}</div>
    </div>
    ${d && html`<div class="panel">
      <div class="ttl"><b>task</b><code>tasks</code>
        <span class="filters">${TASK_FILTERS.map(([k, label]) => html`<button key=${k} class=${'pg' + (filter === k ? ' on' : '')}
          onClick=${() => setFilter(k)}>${label} ${(d.tasks || []).filter((t) => !k || t.type === k).length}</button>`)}</span></div>
      <table class="grid"><thead><tr><th>task</th><th>로봇</th><th>종류</th><th>대상</th><th>점</th><th>상태</th><th>진행률</th><th>사유</th><th>보낸 시각</th><th>끝난 시각</th><th></th></tr></thead>
      <tbody>${tasks.map((t) => html`<tr key=${t.task_id}>
        <td class="mono">${t.task_id}</td><td>${t.robot_id}</td><td>${TYPE_LABEL[t.type] || t.type}</td><td class="mono">${t.target_ref || '—'}</td>
        <td>${(t.waypoints || []).length}</td><td><span class=${'chip ' + (STATUS_CHIP[t.status] || 'off')}>${t.status}</span></td>
        <td>${pct(t.progress)}</td><td class="mono">${t.reason || ''}</td><td>${hm(t.assigned_at)}</td><td>${hm(t.ended_at)}</td>
        <td><button class="btn small" onClick=${() => setDetail(t.task_id)}>자세히</button></td></tr>`)}
      </tbody></table></div>
    <div class="cols2">
      <div class="panel"><div class="ttl"><b>후보 → 정밀측정 결론</b><code>candidates</code><span class="n">${d.candidates.length}</span></div>
        ${d.candidates.length ? html`<table class="grid"><thead><tr><th>후보</th><th>칸</th><th>측정 중앙값</th><th>정밀측정 task</th><th>정밀측정 중앙값</th><th>결론</th></tr></thead>
        <tbody>${d.candidates.map((c) => html`<tr key=${c.candidate_id}>
          <td class="mono">${c.candidate_id}</td><td>${c.ref} (${c.ix},${c.iy})</td><td>${fmtNum(c.score)} dBm</td>
          <td class="mono">${c.reinspect_task_id || '—'}</td><td>${c.precise_best === null ? '—' : fmtNum(c.precise_best) + ' dBm'}</td>
          <td><span class="cand" style=${{ background: CAND_COLOR[c.outcome || c.status] }}>${c.outcome || c.status}</span>${c.finding_id ? ' · 확정 이상 ' + c.finding_id : ''}</td></tr>`)}
        </tbody></table>` : html`<p class="no">후보 없음 — 칸 중앙값(가장 센 AP)이 ${d.thr} dBm 미만인 칸이 나오면 생깁니다.</p>`}</div>
      <${Recording} runId=${runId} />
      <div class="panel"><div class="ttl"><b>설비 탐지 (1회 판정)</b><code>inspections</code><span class="n">${d.inspections.length}</span></div>
        ${d.inspections.length ? html`<table class="grid"><thead><tr><th>설비</th><th>판정</th><th>최종</th><th>정면</th><th>보정</th><th>압력계</th><th>ArUco 제조 정보</th><th>사진</th></tr></thead>
        <tbody>${d.inspections.map((i) => html`<tr key=${i.inspection_id}>
          <td class="mono">${i.facility_id}</td><td>${i.detail}</td>
          <td><span class="cand" style=${{ background: FINAL_COLOR[i.final] }}>${i.final}</span></td>
          <td>${i.frontal ? '예' : '아니요'}</td><td>${i.pose_retry}</td><td>${i.gauge_state || '—'}</td>
          <td>${i.aruco && i.aruco.length ? html`<div class="arucol">${arucoLines(i.aruco)}</div>` : '—'}</td>
          <td>${(i.photos || []).map((k) => html`<a key=${k} href=${'/media/' + k} target="_blank"><img class="thumb" src=${'/media/' + k} /></a>`)}</td></tr>`)}
        </tbody></table>` : html`<p class="no">판정 없음</p>`}</div>
    </div>`}
    ${detail && html`<${TaskDetail} taskId=${detail} m=${sameMap ? m : null} onClose=${() => setDetail(null)} onAuthLost=${onAuthLost} />`}
  </main>`;
}

// ---------------------------------------------------------------- 이력 › task 「자세히」 (3-3차)
//   측정: 거친 칸 · 칸별 AP 측정값 · 이 구간 후보 / 정밀측정: 후보 · 정밀측정 · 결론 / 탐지: 설비 · 판정 · 사진 / 공통: 경로 · 이벤트
const FAC_LABEL = { extinguisher: '소화기', shutter: '셔터', exit_light: '유도등', rack: '랙' };   // 「자세히」 설비 종류 이름
const dur = (a, b) => {
  if (!a) return '—';
  const s = Math.max(0, Math.round(((b ? new Date(b) : new Date()) - new Date(a)) / 1000));
  return s >= 60 ? `${Math.floor(s / 60)}분 ${s % 60}초` : `${s}초`;
};
const trackLen = (tr) => tr.slice(1).reduce((s, p, i) => s + Math.hypot(p.x - tr[i].x, p.y - tr[i].y), 0).toFixed(1);
const dbm = (v, thr) => (v === null || v === undefined ? '—'
  : html`<span class="dbm" style=${{ color: thr === undefined ? 'inherit' : rssiColor(v, thr) }}>${fmtNum(Math.round(v * 10) / 10)}</span>`);

function TaskDetail({ taskId, m, onClose, onAuthLost }) {
  const [d, setD] = useState(null);
  const [msg, setMsg] = useState('');
  const [big, setBig] = useState(null);              // 크게 보는 사진 object_key
  useEffect(() => {
    let alive = true, timer = null;
    const load = () => api(`/api/tasks/${encodeURIComponent(taskId)}`)
      .then((x) => {
        if (!alive) return;
        setD(x);
        if (['ACTIVE', 'PENDING'].includes(x.task.status)) timer = setTimeout(load, 2000);   // 진행 중이면 계속 갱신
      })
      .catch((e) => (e.status === 401 ? onAuthLost() : setMsg(e.message)));
    load();
    return () => { alive = false; clearTimeout(timer); };
  }, [taskId]);
  useEffect(() => {                                  // Esc — 큰 사진이 열려 있으면 사진만, 아니면 창을 닫음
    const esc = (e) => { if (e.key === 'Escape') (big ? setBig(null) : onClose()); };
    addEventListener('keydown', esc);
    return () => removeEventListener('keydown', esc);
  }, [big]);
  const t = d && d.task;
  const apName = {};
  ((d && d.aps) || []).forEach((a) => { apName[a.ap_id] = a.ssid; });
  const thr = d ? d.thr : -72;
  // 지도 표시 — 이 task의 경로 + 측정 칸(칸별 가장 센 AP 중앙값) + 후보 + 판정
  let overlay = null;
  if (d && m) {
    const heat = {};
    ((d.survey && d.survey.cells) || []).forEach((c) => {
      heat[c.cell_id] = Object.fromEntries(Object.entries(c.by_ap).map(([ap, v]) => [ap, [v.median, v.n_detected ?? v.n]]));
    });
    const cands = d.candidates || (d.candidate ? [d.candidate] : []);
    const ins = (d.inspections || []).map((i) => ({ ...i, facility_id: d.facility.facility_id,
      expected_x: d.facility.expected_x, expected_y: d.facility.expected_y }));
    overlay = { heat, ap: '', apName, thr, path: { pts: t.waypoints || [], color: robotColor(t.robot_id) },
      track: { pts: d.track || [], color: robotColor(t.robot_id) }, candidates: cands, inspections: ins };
  }
  const events = ((d && d.events) || []).map((e) => ({ ...e, robot_id: t.robot_id, task_id: null }));
  // 칸 순서 — 계획(경로점마다 칸) · 실제(지난 칸, 다시 지나면 따로). 계획 밖 칸 · 다시 지난 칸 · 못 지난 계획 칸
  const plan = (d && d.plan_cells) || [], visits = (d && d.visits) || [];
  const planIdx = {};
  plan.forEach((p) => { if (p.cell_id !== null) (planIdx[p.cell_id] = planIdx[p.cell_id] || []).push(p.i); });
  const seen = new Set(), visitTags = visits.map((v) => {
    const tag = !planIdx[v.cell_id] ? 'extra' : seen.has(v.cell_id) ? 'revisit' : '';
    seen.add(v.cell_id);
    return tag;
  });
  const missed = plan.filter((p) => p.cell_id !== null && !seen.has(p.cell_id));
  const pathRange = plan.length && plan[0].path_i !== null ? ` (섹터 경로 ${plan[0].path_i}~${plan[plan.length - 1].path_i}번)` : '';
  const seenAp = new Set(((d && d.survey && d.survey.cells) || []).flatMap((c) => Object.keys(c.by_ap).map(Number)));
  const apIds = ((d && d.aps) || []).map((a) => a.ap_id).filter((a) => seenAp.has(a));   // 이 task에서 잰 AP만 열로
  return html`<div class="backdrop" onClick=${(e) => e.target === e.currentTarget && onClose()}><div class="modal taskd">
    <div class="mh"><b>task ${taskId}</b>
      ${t && html`<span>${TYPE_LABEL[t.type] || t.type} · ${t.robot_id}</span>
        <span class=${'chip ' + (STATUS_CHIP[t.status] || 'off')}>${t.status}</span>`}
      <button class="btn small x" onClick=${onClose}>닫기</button></div>
    ${msg && html`<div class="msg">${msg}</div>`}
    <div class="body">${!d ? html`<p>불러오는 중…</p>` : html`
      <div class="tdtop">
        <div class="tdmap">${m ? html`<${MapView} m=${m} robots=${[]} overlay=${overlay} mini=${true} />`
          : html`<div class="empty">이 run은 현재 지도가 아니라 지도 표시는 생략합니다.</div>`}</div>
        <div class="tdsum">
          <dl class="kv">
            <dt>대상</dt><dd class="mono">${t.target_ref || '—'}</dd>
            <dt>경로점</dt><dd>${(t.waypoints || []).length}점</dd>
            <dt>진행률</dt><dd>${pct(t.progress)}</dd>
            <dt>보낸 시각</dt><dd>${fmtTime(t.assigned_at)}</dd>
            <dt>끝난 시각</dt><dd>${fmtTime(t.ended_at)}</dd>
            <dt>걸린 시간</dt><dd>${dur(t.assigned_at, t.ended_at)}${t.ended_at ? '' : (t.assigned_at ? ' (진행 중)' : '')}</dd>
            ${(d.track || []).length > 0 && html`<dt>지나간 길</dt><dd>${trackLen(d.track)} m · 위치 ${d.track.length}개 (제어 PC status, 지도 실선)</dd>
              <dt>배터리</dt><dd>${Math.round(d.track[0].battery_pct)}% → ${Math.round(d.track[d.track.length - 1].battery_pct)}%</dd>`}
            ${t.reason && html`<dt>사유</dt><dd class="mono">${t.reason}</dd>`}
            ${d.survey && html`<dt>표본</dt><dd>${d.survey.totals.n}개 · 칸 ${d.survey.cells.length}개
              ${d.survey.totals.pose_bad ? ` · 위치 불확실 ${d.survey.totals.pose_bad}` : ''}${d.survey.totals.off_grid ? ` · 격자 밖 ${d.survey.totals.off_grid}` : ''}</dd>
              <dt>후보</dt><dd>${(d.candidates || []).length}개 (칸 중앙값 · 가장 센 AP ${'<'} ${thr} dBm)</dd>`}
          </dl>
          ${d.survey && d.survey.basis === 'none' && html`<p class="no">이 task의 표본이 없습니다.</p>`}
          ${d.survey && d.survey.basis === 'time' && html`<p class="no">이 task의 표본에는 task 기록이 없어(3-3차 이전) 같은 로봇 · run에서 이 task 수행 시간 안의 표본으로 보여 줍니다.</p>`}
          <h4>이 task의 이벤트 <span class="no">${events.length}건</span></h4>
          <${EventList} events=${events} max=${100} />
        </div>
      </div>

      ${(plan.length > 0 || visits.length > 0) && html`<h4>칸 순서 <span class="no">칸 번호(ix,iy)</span></h4>
        <div class="seq"><span class="sl">계획</span><span class="sv">${plan.length ? plan.map((p) => html`<span key=${p.i} class="cc plan"
          title=${`경로점 ${p.i}${p.path_i !== null ? ' · 섹터 경로 ' + p.path_i + '번' : ''}`}><i>${p.i}</i>${p.cell_id === null ? '격자 밖' : `${p.cell_id}(${p.ix},${p.iy})`}</span>`)
          : html`<span class="no">경로점 없음</span>`}</span></div>
        <div class="seq"><span class="sl">실제</span><span class="sv">${visits.length ? visits.map((v, i) => html`<span key=${i} class=${'cc ' + visitTags[i]}
          title=${`${hm(v.enter)} ~ ${hm(v.leave)} · ${d.visits_source === 'samples' ? '표본' : '위치'} ${v.n}개${visitTags[i] === 'extra' ? ' · 계획 밖' : visitTags[i] === 'revisit' ? ' · 다시 지남' : ''}`}>
          <i>${i + 1}</i>${v.cell_id}(${v.ix},${v.iy})</span>`) : html`<span class="no">기록 없음${['ACTIVE', 'PENDING'].includes(t.status) ? ' (아직 시작 전 · 진행 중)' : ''}</span>`}</span></div>
        <p class="no">계획 칸 ${Object.keys(planIdx).length}개${pathRange} 중 지난 칸 ${Object.keys(planIdx).length - new Set(missed.map((p) => p.cell_id)).size}개
          ${missed.length ? html` · <span class=${'cc ' + (d.visits_source === 'samples' ? 'missed' : 'nolog')}>${d.visits_source === 'samples' ? '못 지난 계획 칸' : '위치 기록에 없는 계획 칸(1초 간격이라 짧게 머문 칸은 빠짐)'} ${[...new Set(missed.map((p) => `${p.cell_id}(${p.ix},${p.iy})`))].join(', ')}</span>` : ''}
          ${visitTags.includes('extra') ? html` · <span class="cc extra">계획 밖 ${visitTags.filter((x) => x === 'extra').length}칸</span>` : ''}
          ${visitTags.includes('revisit') ? html` · <span class="cc revisit">다시 지남 ${visitTags.filter((x) => x === 'revisit').length}번</span>` : ''}
          ${' · '}실제 순서는 ${d.visits_source === 'samples' ? '측정 표본(rssi 주기) 위치' : d.visits_source === 'status' ? '제어 PC status 위치(1초마다)' : '—'} 기준 — 짧게 스친 칸은 빠지고, 칸 경계에서는 앞뒤 칸이 섞일 수 있음</p>`}

      ${d.survey && html`<h4>지난 칸 · 칸별 측정값 <span class="no">지난 순서 · 중앙값 dBm (표본 수, 최소 ~ 최대)</span></h4>
        ${d.survey.cells.length ? html`<table class="grid"><thead><tr><th>#</th><th>계획 #</th><th>칸</th><th>섹터</th>
          ${apIds.map((a) => html`<th key=${a}>${apName[a]}</th>`)}${apIds.length > 1 && html`<th>가장 센 AP</th>`}<th>후보</th></tr></thead>
          <tbody>${d.survey.cells.map((c, i) => {
            const vals = Object.values(c.by_ap).map((v) => v.median);
            const best = vals.length ? Math.max(...vals) : null;
            const cand = (d.candidates || []).find((x) => String(x.ref) === String(c.cell_id));
            return html`<tr key=${c.cell_id}><td>${i + 1}</td>
              <td>${planIdx[c.cell_id] ? planIdx[c.cell_id].join(', ') : html`<span class="cc extra">계획 밖</span>`}</td>
              <td class="mono">${c.cell_id} (${c.ix},${c.iy})</td><td>${c.sector}</td>
              ${apIds.map((a) => { const v = c.by_ap[a]; return html`<td key=${a}>${v ? html`${dbm(v.median, thr)} <small class="no">(${v.n}, ${v.min} ~ ${v.max})</small>` : '—'}</td>`; })}
              ${apIds.length > 1 && html`<td>${dbm(best, thr)}</td>`}
              <td>${cand ? html`<span class="cand" style=${{ background: CAND_COLOR[cand.outcome || cand.status] }}>${cand.candidate_id} · ${cand.outcome || cand.status}</span>`
                : best !== null && best < thr ? html`<small class="no">기준 미만 (표본 부족)</small>` : ''}</td></tr>`;
          })}</tbody></table>` : html`<p class="no">위치가 확실한 표본이 없습니다.</p>`}`}

      ${t.type === 'REINSPECTION' && html`<h4>후보 → 정밀측정 → 결론</h4>
        ${d.candidate ? html`<dl class="kv wide">
          <dt>후보</dt><dd class="mono">${d.candidate.candidate_id} · ${d.candidate.source} · ${d.candidate.kind}</dd>
          <dt>칸 · 지점</dt><dd>${d.candidate.ref}${d.candidate.ix !== null && d.candidate.ix !== undefined ? ` (${d.candidate.ix},${d.candidate.iy})` : ''} · (${fmtNum(d.candidate.target_x)}, ${fmtNum(d.candidate.target_y)})</dd>
          <dt>발견</dt><dd class="mono">${d.candidate.detect_task_id} · ${fmtTime(d.candidate.detected_at)} · 점수 ${fmtNum(d.candidate.score)}${d.candidate.source === 'NETWORK' ? ' dBm' : ''}</dd>
          <dt>측정 때 칸 중앙값</dt><dd>${(d.survey_cell || []).length ? d.survey_cell.map((v) => html`<span key=${v.ap_id} class="apv">${apName[v.ap_id]} ${dbm(v.median, thr)} <small class="no">(${v.n})</small></span>`) : '—'}</dd>
          <dt>결론</dt><dd><span class="cand" style=${{ background: CAND_COLOR[d.candidate.outcome || d.candidate.status] }}>${d.candidate.outcome || d.candidate.status}</span>
            ${d.candidate.finding_id ? ` 확정 이상 ${d.candidate.finding_id} (${d.candidate.finding_status})` : ''}${d.candidate.decided_at ? ' · ' + fmtTime(d.candidate.decided_at) : ''}</dd>
        </dl>` : html`<p class="no">후보 정보가 없습니다.</p>`}
        ${(d.precise || []).length ? html`<table class="grid"><thead><tr><th>AP</th><th>측정 시간</th><th>표본</th><th>중앙값</th><th>IQR</th><th>잰 시각</th></tr></thead>
          <tbody>${d.precise.map((p) => html`<tr key=${p.ap_id}><td>${p.ssid}</td><td>${p.duration_s}초</td><td>${p.n_samples}</td>
            <td>${dbm(p.rssi_median, thr)} dBm</td><td>${fmtNum(p.rssi_iqr)} dB</td><td>${fmtTime(p.measured_at)}</td></tr>`)}</tbody></table>`
          : html`<p class="no">정밀측정 기록 없음</p>`}`}

      ${t.type === 'FACILITY_CHECK' && html`<h4>설비 · 판정 · 사진</h4>
        ${d.facility && html`<dl class="kv wide">
          <dt>설비</dt><dd class="mono">${d.facility.facility_id} · ${FAC_LABEL[d.facility.facility_type] || d.facility.facility_type}${d.facility.has_gauge ? ' · 압력계' : ''}${d.facility.is_active ? '' : ' (비활성)'}</dd>
          <dt>등록 위치</dt><dd>(${fmtNum(d.facility.expected_x)} m, ${fmtNum(d.facility.expected_y)} m) · 관측 자세 (${fmtNum(d.facility.view_pose.x)} m, ${fmtNum(d.facility.view_pose.y)} m, ${fmtNum(d.facility.view_pose.yaw)} rad)</dd>
        </dl>`}
        ${(d.inspections || []).length ? d.inspections.map((i) => html`<div class="insp" key=${i.inspection_id}>
          <dl class="kv wide">
            <dt>판정</dt><dd>${i.detail} · <span class="cand" style=${{ background: FINAL_COLOR[i.final] }}>${i.final}</span></dd>
            <dt>정면 · 보정</dt><dd>${i.frontal ? '정면' : '정면 아님'} · 자세 보정 ${i.pose_retry}회</dd>
            <dt>압력계 · 신뢰도</dt><dd>${i.gauge_state || '—'} · ${i.confidence === null ? '—' : Math.round(i.confidence * 100) + '%'}</dd>
            <dt>bbox · 모델</dt><dd class="mono">${i.bbox ? JSON.stringify(i.bbox) : '—'} · ${i.model_version}</dd>
            <dt>판정 시각</dt><dd>${fmtTime(i.inspected_at)}</dd>
          </dl>
          <div class="photos">${(i.media || []).length ? i.media.map((md) => html`<figure key=${md.object_key}>
            <img src=${'/media/' + md.object_key} onClick=${() => setBig(md.object_key)} title="크게 보기" />
            <figcaption>${md.kind} · ${Math.round(md.size_bytes / 1024)} KB · ${hm(md.captured_at)}</figcaption></figure>`)
            : html`<p class="no">사진 없음</p>`}</div>
        </div>`) : html`<p class="no">판정 기록 없음${['ACTIVE', 'PENDING'].includes(t.status) ? ' (아직 판정 전)' : ''}</p>`}`}
    `}</div>
    ${big && html`<div class="photobig" onClick=${() => setBig(null)}><img src=${'/media/' + big} /><span>${big} — 누르면 닫힘</span></div>`}
  </div></div>`;
}

// ---------------------------------------------------------------- 정보 (master)
//   오른쪽 세로 탭에서 표를 누를 때마다 선택·해제. 선택한 표는 선택한 순서대로 왼쪽에 세로로 쌓임
function MasterPage({ onAuthLost, selected, setSelected }) {
  const [ui] = useUiSocket(onAuthLost);              // AP 활성 · 측정 AP 실시간 (3-3차)
  const apSig = ui ? JSON.stringify(ui.aps || []) : '';
  const [cards, setCards] = useState(null);
  const [err, setErr] = useState('');
  const [open, setOpen] = useState(null);
  const load = useCallback(() => {
    api('/api/master/summary')
      .then((d) => setCards(d.cards))
      .catch((e) => (e.status === 401 ? onAuthLost() : setErr(e.message)));
  }, []);
  useEffect(load, []);
  const turnPage = (t, page) => api(`/api/master/summary/${t}?page=${page}`)
    .then((c) => setCards((cs) => cs.map((x) => (x.table === t ? c : x))))
    .catch((e) => (e.status === 401 ? onAuthLost() : setErr(e.message)));
  const toggle = (t) => setSelected((s) => (s.includes(t) ? s.filter((x) => x !== t) : [...s, t]));
  useEffect(() => {                                  // AP 활성이 바뀌면 AP 패널을 다시 읽음 (지금 쪽 그대로)
    if (!apSig || !cards) return;
    const c = cards.find((x) => x.table === 'aps');
    if (c) turnPage('aps', c.page || 1);
  }, [apSig]);
  const byTable = Object.fromEntries((cards || []).map((c) => [c.table, c]));
  const shown = selected.filter((t) => byTable[t]);

  return html`<main>
    <h2>정보</h2>
    <p class="sub">오른쪽 탭에서 표를 선택하면 아래로 쌓여 보입니다(다시 누르면 해제). 「목록」에서 행을 추가·삭제하고 값을 바꿉니다.</p>
    ${err && html`<div class="msg">${err}</div>`}
    <div class="info">
      <div class="stack">
        ${cards && shown.length === 0 && html`<div class="panel empty">오른쪽 탭에서 볼 표를 선택하세요.</div>`}
        ${shown.map((t) => {
          const c = byTable[t];
          return html`<section key=${t} class="panel">
            <div class="ttl"><b>${c.title}</b><code>${c.table}</code><span class="n">${c.count}행</span>
              <button class="btn" onClick=${() => setOpen(c.table)}>목록</button></div>
            <div class="desc">${c.desc}</div>
            ${c.items.length
              ? html`<div class=${'kvgrid' + (c.items.some((x) => x[2]) ? ' sub' : '')}>
                  ${c.items.map(([k, v, sub], i) => html`<div key=${i} class="kvi">
                    <span>${k}${sub ? html`<em>${sub}</em>` : ''}</span><span>${v}</span></div>`)}
                </div>`
              : html`<div class="desc">등록된 행이 없습니다.</div>`}
            <${Pager} page=${c.page} pages=${c.pages} onPage=${(p) => turnPage(t, p)} />
          </section>`;
        })}
      </div>
      <nav class="vtabs">
        ${(cards || []).map((c) => {
          const on = selected.includes(c.table);
          return html`<button key=${c.table} class=${'vtab' + (on ? ' on' : '')} onClick=${() => toggle(c.table)}>
            <span class="chk">${on ? selected.indexOf(c.table) + 1 : ''}</span>
            <span class="vt"><b>${c.title}</b><code>${c.table}</code></span>
            <span class="n">${c.count}</span>
          </button>`;
        })}
      </nav>
    </div>
    ${open && html`<${ListModal} table=${open} onAuthLost=${onAuthLost} live=${open === 'aps' ? apSig : ''}
      onClose=${() => { setOpen(null); load(); }} />`}
  </main>`;
}

function ListModal({ table, onClose, onAuthLost, live }) {
  const [data, setData] = useState(null);
  const [msg, setMsg] = useState(null);           // { text, ok }
  const [adding, setAdding] = useState(false);
  const [newRow, setNewRow] = useState({});
  const [editKey, setEditKey] = useState(null);
  const [editRow, setEditRow] = useState({});
  const [mapId, setMapId] = useState(null);       // 지도에 속한 표(격자 칸 · AP · 설비)의 조회 지도
  const [page, setPage] = useState(1);
  const [busy, setBusy] = useState(false);

  const fail = (e) => (e.status === 401 ? onAuthLost() : setMsg({ text: e.message }));
  const load = useCallback((mid, pg = 1) => {
    const q = new URLSearchParams({ page: pg });
    if (mid) q.set('map_id', mid);
    return api(`/api/master/${table}?${q}`).then((d) => {
      setData(d);
      setPage(d.page);
      if (d.maps) setMapId(d.map_id);
    }).catch(fail);
  }, [table]);
  useEffect(() => { load(); }, [load]);
  useEffect(() => {                                  // AP 목록 — 활성 · 측정 AP가 바뀌면 다시 읽음 (입력 중이면 기다림)
    if (live && data && !adding && editKey === null) load(mapId, page);
  }, [live]);
  useEffect(() => {
    const k = (e) => e.key === 'Escape' && onClose();
    addEventListener('keydown', k);
    return () => removeEventListener('keydown', k);
  }, []);

  const run = async (fn, okText, toPage = page) => {
    setBusy(true); setMsg(null);
    try {
      const r = await fn();
      setMsg({ text: typeof okText === 'function' ? okText(r) : okText, ok: true });
      await load(mapId, toPage);
      return true;
    } catch (e) { fail(e); return false; } finally { setBusy(false); }
  };

  if (!data) {
    return html`<div class="backdrop"><div class="modal"><div class="mh"><b>불러오는 중…</b>
      <button class="btn x" onClick=${onClose}>닫기</button></div>
      ${msg && html`<div class="msg">${msg.text}</div>`}</div></div>`;
  }
  const { meta, rows } = data;
  const ops = meta.ops;
  const shown = meta.columns.filter((c) => (c.type !== 'password' || c.edit) && !c.hide);
  const addCols = meta.columns.filter((c) => c.add);
  const pk = meta.pk;
  const byMap = !!data.maps;                       // 지도에 속한 표 — 목록 위 지도 선택
  const locked = byMap && data.running_run;        // 그 지도로 순찰 중 → 추가·수정·삭제 잠금
  const mq = meta.map_scoped && mapId ? `?map_id=${mapId}` : '';

  const startAdd = () => {
    const init = {};
    addCols.forEach((c) => { init[c.name] = c.default ?? (c.type === 'pose' ? { x: '', y: '', yaw: '' } : ''); });
    setNewRow(init); setAdding(true); setEditKey(null);
  };
  const saveAdd = async () => {
    const body = meta.map_scoped ? { ...newRow, map_id: mapId } : newRow;    // 고른 지도에 추가
    if (await run(() => api(`/api/master/${table}`, { json: body }), '추가했습니다', 1)) setAdding(false);   // 최신 순이라 1쪽
  };
  const startEdit = (r) => {
    const init = {};
    meta.columns.filter((c) => c.edit && canEdit(r, c)).forEach((c) => { init[c.name] = c.type === 'password' ? '' : r[c.name]; });
    setEditRow(init); setEditKey(r[pk]); setAdding(false);
  };
  const saveEdit = async () => {
    if (await run(() => api(`/api/master/${table}/${encodeURIComponent(editKey)}${mq}`, { method: 'PATCH', json: editRow }),
      '값을 바꿨습니다')) setEditKey(null);
  };
  const del = (r) => {
    if (!confirm(`${meta.title} 「${r[pk]}」 행을 삭제할까요?`)) return;
    if (table === 'maps' && !confirm('이 지도의 격자 칸 · AP · 설비도 함께 지워집니다. 계속할까요?')) return;
    const path = table === 'maps' ? `/api/master/maps/${r.map_id}` : `/api/master/${table}/${encodeURIComponent(r[pk])}${mq}`;
    run(() => api(path, { method: 'DELETE' }), '삭제했습니다');
  };
  const target = (r) => run(() => api(`/api/aps/${r.ap_id}/target`, { method: 'POST' }),
    `「${r.ssid}」을(를) 측정 AP로 지정했습니다 — 순찰은 이 SSID만 측정합니다`);
  const activate = (r) => run(() => api(`/api/maps/${r.map_id}/activate`, { method: 'POST' }),
    (d) => `「${r.name}」을(를) 현재 지도로 바꿨습니다` + (d.created ? ` — 격자 ${d.created}칸을 만들었습니다` : ` (격자 ${d.cell_count}칸)`));

  const hasAct = ops.some((o) => ['edit', 'delete', 'activate', 'target'].includes(o));
  const canEdit = (r, c) => !r._edit_only || r._edit_only.includes(c.name);   // 수퍼 계정은 비밀번호만

  return html`<div class="backdrop" onMouseDown=${(e) => e.target === e.currentTarget && onClose()}>
    <div class="modal">
      <div class="mh"><b>${meta.title}</b><code>${table}</code>
        <span class="sub" style=${{ margin: 0 }}>${data.total}${table === 'grid_cells' ? '칸' : '행'} · 최신 순</span>
        <button class="btn x" onClick=${onClose}>닫기</button></div>
      <div class="mdesc">${meta.desc}</div>

      <div class="tools">
        ${ops.includes('add') && html`<button class="btn primary" disabled=${adding || busy || locked || (byMap && !mapId)}
          title=${locked ? '순찰 중인 지도는 바꿀 수 없습니다' : ''} onClick=${startAdd}>추가</button>`}
        ${ops.includes('upload') && html`<button class="btn primary" disabled=${adding || busy} onClick=${() => setAdding(true)}>지도 등록</button>`}
        ${byMap && html`
          <label class="sub" style=${{ margin: 0 }}>지도</label>
          <select value=${mapId ?? ''} disabled=${adding || editKey !== null} onChange=${(e) => { setMsg(null); load(e.target.value, 1); }}>
            ${data.maps.length === 0 && html`<option value="">등록된 지도 없음</option>`}
            ${data.maps.map((o) => html`<option key=${o.value} value=${o.value}>${o.label}</option>`)}
          </select>
          ${locked && html`<span class="lockmsg">순찰 run ${data.running_run} 진행 중 — 이 지도는 잠김</span>`}`}
        ${table === 'grid_cells' && html`
          <span class="sp"></span>
          <button class="btn" disabled=${!mapId || busy} onClick=${() => confirm('이 지도의 격자를 지우고 다시 만들까요?') &&
            run(() => api(`/api/maps/${mapId}/grid`, { method: 'POST' }), (d) => `격자 ${d.cell_count}칸을 다시 만들었습니다`)}>격자 다시 생성</button>
          <button class="btn danger" disabled=${!mapId || busy} onClick=${() => confirm('이 지도의 격자를 모두 삭제할까요?') &&
            run(() => api(`/api/maps/${mapId}/grid`, { method: 'DELETE' }), (d) => `격자 ${d.deleted}칸을 삭제했습니다`)}>격자 삭제</button>`}
      </div>

      ${table === 'maps' && adding && html`<${MapUpload} busy=${busy}
        onCancel=${() => setAdding(false)}
        onSubmit=${async (fd) => {
          if (await run(() => api('/api/maps/upload', { method: 'POST', body: fd }),
            (d) => `지도를 등록했습니다 (${d.width} × ${d.height} px). 「현재 지도로」를 누르면 격자가 생성됩니다`)) setAdding(false);
        }} />`}

      ${msg && html`<div class=${'msg' + (msg.ok ? ' ok' : '')}>${msg.text}</div>`}

      <div class="body">
        <table class="grid">
          <thead><tr>
            ${(adding && table !== 'maps' ? meta.columns : shown).map((c) => html`<th key=${c.name}>${c.label}</th>`)}
            ${hasAct && html`<th></th>`}
          </tr></thead>
          <tbody>
            ${adding && table !== 'maps' && html`<tr class="new">
              ${meta.columns.map((c) => html`<td key=${c.name}>
                ${c.add ? html`<${Input} col=${c} value=${newRow[c.name]}
                  onChange=${(k, v) => setNewRow((p) => ({ ...p, [k]: v }))} />` : html`<span class="no">자동</span>`}
                ${c.add && c.help && html`<span class="help">${c.help}</span>`}
              </td>`)}
              <td class="act">
                <button class="btn small primary" disabled=${busy} onClick=${saveAdd}>저장</button>
                <button class="btn small" onClick=${() => setAdding(false)}>취소</button>
              </td>
            </tr>`}
            ${rows.map((r) => {
              const editing = editKey !== null && String(editKey) === String(r[pk]);
              const cols = adding && table !== 'maps' ? meta.columns : shown;
              return html`<tr key=${r[pk]} class=${editing ? 'edit' : ''}>
                ${cols.map((c) => html`<td key=${c.name}>
                  ${editing && c.edit && canEdit(r, c)
                    ? html`<${Input} col=${c} value=${editRow[c.name]} onChange=${(k, v) => setEditRow((p) => ({ ...p, [k]: v }))} />
                      ${c.type === 'password' && html`<span class="help">바꿀 때만 입력</span>`}`
                    : c.type === 'password' ? html`<span class="no">••••</span>`
                    : c.name === 'is_target' ? (r.is_target ? html`<span class="chip ok">측정 AP</span>` : html`<span class="no">—</span>`)
                    : showValue(c, r[c.name])}
                </td>`)}
                ${hasAct && html`<td class="act">
                  ${editing ? html`
                    <button class="btn small primary" disabled=${busy} onClick=${saveEdit}>저장</button>
                    <button class="btn small" onClick=${() => setEditKey(null)}>취소</button>` : html`
                    ${ops.includes('activate') && !r.is_active && html`<button class="btn small" disabled=${busy} onClick=${() => activate(r)}>현재 지도로</button>`}
                    ${ops.includes('target') && !r.is_target && html`<button class="btn small" disabled=${busy || locked || !r.is_active}
                      title=${r.is_active ? '' : '비활성 AP는 고를 수 없습니다 (client가 이 SSID 값을 받을 때 활성)'} onClick=${() => target(r)}>측정 AP로</button>`}
                    ${ops.includes('edit') && html`<button class="btn small" disabled=${busy || adding || locked} onClick=${() => startEdit(r)}>수정</button>`}
                    ${ops.includes('delete') && !r._no_delete && html`<button class="btn small danger" disabled=${busy || locked} onClick=${() => del(r)}>삭제</button>`}`}
                </td>`}
              </tr>`;
            })}
            ${rows.length === 0 && !adding && html`<tr><td colspan=${shown.length + 1} class="no">등록된 행이 없습니다.</td></tr>`}
          </tbody>
        </table>
        <${Pager} page=${page} pages=${data.pages} disabled=${busy || adding || editKey !== null}
          onPage=${(p) => { setMsg(null); load(mapId, p); }} />
      </div>
    </div>
  </div>`;
}

function MapUpload({ busy, onSubmit, onCancel }) {
  const ref = useRef(null);
  const submit = (e) => { e.preventDefault(); onSubmit(new FormData(ref.current)); };
  return html`<form class="upload" ref=${ref} onSubmit=${submit}>
    <div class="field"><label>이름</label><input name="name" required placeholder="testbed_20261010" /></div>
    <div class="field"><label>제어 PC 지도 경로 (yaml)</label>
      <input name="robot_map_path" required placeholder="/opt/scv/maps/testbed_20261010.yaml" /></div>
    <div class="field"><label>격자 크기 (m)</label><input name="grid_size_m" type="number" step="any" min="0.1" defaultValue="1.0" /></div>
    <div class="field"><label>yaml 파일</label><input name="yaml_file" type="file" accept=".yaml,.yml" required /></div>
    <div class="field"><label>pgm 파일</label><input name="pgm_file" type="file" accept=".pgm,.png" required /></div>
    <div style=${{ display: 'flex', gap: '6px' }}>
      <button class="btn primary" disabled=${busy}>업로드</button>
      <button class="btn" type="button" onClick=${onCancel}>취소</button>
    </div>
  </form>`;
}

// ---------------------------------------------------------------- 앱
function App() {
  const [me, setMe] = useState(undefined);        // undefined = 확인 중, null = 로그인 안 됨
  const [page, setPage] = useState(pageFromPath());
  const [infoSel, setInfoSel] = useState([]);    // 정보 탭에서 선택한 표 (다른 탭에 다녀와도 유지)
  const go = useCallback((p) => {
    if (location.pathname !== `/server/${p}`) history.pushState({}, '', `/server/${p}`);
    setPage(p);
  }, []);
  useEffect(() => {
    const f = () => setPage(pageFromPath());
    addEventListener('popstate', f);
    return () => removeEventListener('popstate', f);
  }, []);
  useEffect(() => { api('/api/me').then((d) => setMe(d.user)).catch(() => setMe(null)); }, []);
  useEffect(() => {
    if (me === null && page !== 'login') go('login');
    if (me && (page === 'login' || !TABS.some((t) => t.id === page))) go('monitor');
  }, [me, page]);
  const logout = async () => { await api('/api/logout', { method: 'POST' }).catch(() => {}); setMe(null); };
  const onAuthLost = () => setMe(null);

  return html`
    <header class="top">
      <div class="brand">SCV 관제<small>server page</small></div>
      <nav class="tabs">
        ${TABS.map((t) => html`<button key=${t.id} class=${'tab' + (page === t.id ? ' on' : '')}
          disabled=${!me} title=${me ? '' : '로그인 후 사용할 수 있습니다'} onClick=${() => go(t.id)}>${t.label}</button>`)}
      </nav>
      <div class="who">${me
        ? html`<span>${me.username}</span><button class="btn small ghost" onClick=${logout}>로그아웃</button>`
        : html`<span>로그인 필요</span>`}</div>
    </header>
    ${me === undefined ? html`<main><p class="sub">확인 중…</p></main>`
      : !me ? html`<${LoginPage} onLogin=${(u) => setMe(u)} />`
      : page === 'master' ? html`<${MasterPage} onAuthLost=${onAuthLost} selected=${infoSel} setSelected=${setInfoSel} />`
      : page === 'history' ? html`<${HistoryPage} onAuthLost=${onAuthLost} />`
      : html`<${MonitorPage} onAuthLost=${onAuthLost} />`}
  `;
}

createRoot(document.getElementById('root')).render(html`<${App} />`);
