// robot3d.js — URDF · STL 읽기 (공용: services/dashboard · apps/gripper_pick/xr_view · apps/act_viewer). three.js (전역 THREE) 다음에 불러옴.
//   Robot3D.parseSTL(buf, cutX)  → BufferGeometry (이진 · 글자 STL, 법선 계산). cutX: 메시 x 가 이보다 큰 삼각형은 버림 (H2 손목 메시의 기본 손)
//   Robot3D.parseURDF(text)      → {links, joints, materials}
//       links[이름]  = {name, visuals: [{fn, sc, origin: {xyz, rpy}, matName, rgba}]}   rgba = visual 안 색 (없으면 null)
//       joints[이름] = {name, type, parent, child, origin, axis, limit: {lower, upper, effort}}
//       materials[이름] = 0xRRGGBB (robot 아래 material 색)
//   Robot3D.setOrigin(obj, origin) → 위치 + URDF rpy (고정축 roll → pitch → yaw = Rz·Ry·Rx)
//   Robot3D.basename(path)        → 파일 이름 (소문자)
(function () {
  "use strict";
  const nums = s => (s || "").trim().split(/\s+/).filter(Boolean).map(Number);

  function parseSTL(buf, cutX) {
    const dv = new DataView(buf), geo = new THREE.BufferGeometry();
    const n = buf.byteLength >= 84 ? dv.getUint32(80, true) : 0;
    let pos;
    if (Math.abs(buf.byteLength - (84 + n * 50)) <= 4 && n > 0) {
      pos = new Float32Array(n * 9);
      let m = 0;
      for (let i = 0; i < n; i++) {
        const o = 84 + i * 50 + 12;
        if (cutX != null && Math.max(dv.getFloat32(o, true), dv.getFloat32(o + 12, true), dv.getFloat32(o + 24, true)) > cutX) continue;
        for (let k = 0; k < 9; k++) pos[m * 9 + k] = dv.getFloat32(o + k * 4, true);
        m++;
      }
      pos = pos.subarray(0, m * 9);
    } else {
      const txt = new TextDecoder().decode(buf), v = [], re = /vertex\s+(\S+)\s+(\S+)\s+(\S+)/g;
      let m;
      while ((m = re.exec(txt))) v.push(+m[1], +m[2], +m[3]);
      if (cutX != null) {
        const w = [];
        for (let i = 0; i + 8 < v.length; i += 9) if (Math.max(v[i], v[i + 3], v[i + 6]) <= cutX) for (let k = 0; k < 9; k++) w.push(v[i + k]);
        pos = new Float32Array(w);
      } else pos = new Float32Array(v);
    }
    geo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
    geo.computeVertexNormals();
    return geo;
  }

  function origin(el) {
    if (!el) return { xyz: [0, 0, 0], rpy: [0, 0, 0] };
    return { xyz: nums(el.getAttribute("xyz") || "0 0 0"), rpy: nums(el.getAttribute("rpy") || "0 0 0") };
  }

  function parseURDF(text) {
    const doc = new DOMParser().parseFromString(text, "text/xml"), links = {}, joints = {}, materials = {};
    doc.querySelectorAll("robot > material").forEach(m => {
      const c = m.querySelector("color")?.getAttribute("rgba");
      if (c) { const [r, g, b] = nums(c); materials[m.getAttribute("name")] = (Math.round(r * 255) << 16) | (Math.round(g * 255) << 8) | Math.round(b * 255); }
    });
    doc.querySelectorAll("robot > link").forEach(el => {
      const name = el.getAttribute("name"); links[name] = { name, visuals: [] };
      el.querySelectorAll(":scope > visual").forEach(v => {
        const me = v.querySelector("geometry mesh"); if (!me) return;
        const mat = v.querySelector("material"), col = mat?.querySelector("color")?.getAttribute("rgba");
        links[name].visuals.push({ fn: me.getAttribute("filename") || "", sc: nums(me.getAttribute("scale") || "1 1 1"),
          origin: origin(v.querySelector(":scope > origin")), matName: mat?.getAttribute("name") || "", rgba: col ? nums(col) : null });
      });
    });
    doc.querySelectorAll("robot > joint").forEach(el => {
      const name = el.getAttribute("name"), lim = el.querySelector("limit");
      joints[name] = { name, type: el.getAttribute("type") || "fixed",
        parent: el.querySelector("parent")?.getAttribute("link") || "", child: el.querySelector("child")?.getAttribute("link") || "",
        origin: origin(el.querySelector(":scope > origin")), axis: nums(el.querySelector("axis")?.getAttribute("xyz") || "1 0 0"),
        limit: { lower: lim ? +lim.getAttribute("lower") : -3.14, upper: lim ? +lim.getAttribute("upper") : 3.14,
                 effort: lim && lim.getAttribute("effort") ? +lim.getAttribute("effort") : null } };
    });
    return { links, joints, materials };
  }

  function setOrigin(obj, o) {
    obj.position.set(o.xyz[0], o.xyz[1], o.xyz[2]);
    obj.quaternion.setFromEuler(new THREE.Euler(o.rpy[0], o.rpy[1], o.rpy[2], "ZYX"));
  }

  const basename = s => s.split(/[/\\]/).pop().toLowerCase();
  window.Robot3D = { parseSTL, parseURDF, setOrigin, basename };
})();
