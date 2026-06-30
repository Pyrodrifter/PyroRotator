#pragma once
#include <pgmspace.h>

// PYRO ROTATOR web UI. Uses only existing endpoints:
//   GET /api/status -> {az,el,taz,tel,moving,homed,ip[,rssi,uptime,control]}
//   GET /api/goto?az=&el=   /api/jog?axis=&delta=   /api/stop /api/home /api/park
// Pure front-end; drop-in with no firmware change.

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>PYRO ROTATOR</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Chakra+Petch:wght@500;600;700&family=Fira+Code:wght@400;500&display=swap" rel="stylesheet">
<style>
:root{--bg:#0a0a0c;--panel:#131318;--panel2:#17171e;--line:#2a2a32;
  --org:#ff4a1c;--txt:#e8e8ea;--dim:#6f6f78;--ok:#3ad17a}
.accent-btn{position:relative;width:32px;height:32px;padding:0;border:1px solid var(--line);
  background:var(--panel);border-radius:8px;cursor:pointer;overflow:hidden}
.accent-btn-dot{width:14px;height:14px;border-radius:50%;background:var(--org);
  position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);pointer-events:none}
#accentPick{position:absolute;inset:0;opacity:0;cursor:pointer;width:100%;height:100%}
*{box-sizing:border-box}
html,body{margin:0}
body{background:var(--bg);
  background-image:linear-gradient(rgba(255,255,255,.018) 1px,transparent 1px),
    linear-gradient(90deg,rgba(255,255,255,.018) 1px,transparent 1px);
  background-size:26px 26px;color:var(--txt);
  font-family:'Fira Code',ui-monospace,monospace;-webkit-tap-highlight-color:transparent}
.wrap{max-width:560px;margin:0 auto;padding:16px 16px 40px;transition:opacity .3s}
.wrap.stale .tele,.wrap.stale .scopewrap{opacity:.4;transition:opacity .3s}
header{display:flex;align-items:center;gap:12px;padding:6px 2px 14px}
h1{font-family:'Chakra Petch',sans-serif;font-weight:700;font-size:21px;margin:0;
  letter-spacing:2px;color:var(--org)}
.call{color:var(--dim);font-size:12px;letter-spacing:1px}
.hgroup{margin-left:auto;display:flex;align-items:center;gap:8px}
.pill{font-size:11px;letter-spacing:1px;padding:5px 10px;border-radius:999px;
  border:1px solid var(--line);background:var(--panel);color:var(--dim);white-space:nowrap}
.pill b{color:var(--ok);font-weight:500}
.pill.lost b{color:var(--org)}
.topbtn{width:32px;height:32px;padding:0;border:1px solid var(--line);background:var(--panel);
  color:var(--dim);border-radius:8px;font-size:14px;line-height:1;cursor:pointer}
.topbtn:hover{color:var(--txt)}
.tele{display:flex;gap:12px;margin-bottom:14px}
.t{flex:1;background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:12px 14px}
.t .lbl{color:var(--dim);font-size:10px;letter-spacing:3px}
.t .v{font-family:'Chakra Petch',sans-serif;font-weight:700;font-size:40px;line-height:1.05;
  font-variant-numeric:tabular-nums}
.t .v .u{font-size:18px;color:var(--dim);font-weight:500;margin-left:2px}
.t .meta{font-size:11px;color:var(--dim);margin-top:3px}
.t .meta .d{color:var(--org)}
.scopewrap{position:relative;background:var(--panel);border:1px solid var(--line);
  border-radius:14px;padding:10px;margin-bottom:14px;transition:opacity .3s}
svg{display:block;width:100%;height:auto;touch-action:none}
.ring{fill:none;stroke:var(--line);stroke-width:1}
.ring.rim{stroke:#3a3a44}
.spoke{stroke:var(--line);stroke-width:1;opacity:.6}
.card{fill:var(--dim);font:600 12px 'Fira Code',monospace}
.elnum{fill:var(--dim);font:400 9px 'Fira Code',monospace}
.beam{stroke:var(--org);stroke-width:1.5;opacity:.5}
.lead{stroke:var(--org);stroke-width:1;stroke-dasharray:4 4;opacity:0;transition:opacity .3s}
.cur{fill:var(--org)}
.curglow{fill:var(--org);opacity:.18}
.tgt{fill:none;stroke:var(--org);stroke-width:1.5;stroke-dasharray:3 3;opacity:.8}
.aim{fill:none;stroke:#fff;stroke-width:1.5;stroke-dasharray:2 4;opacity:.9}
.traildot{fill:var(--org)}
.hint{position:absolute;top:14px;left:16px;font-size:10px;color:var(--dim)}
.sstat{display:block;text-align:center;font-size:11px;letter-spacing:1px;
  color:var(--org);min-height:16px;margin:2px 0}
.sstat.ok{color:var(--ok)}
.go-row{display:flex;gap:8px;margin-bottom:10px}
input,select,button{font-family:'Fira Code',monospace;font-size:15px;border-radius:9px;
  border:1px solid var(--line);background:var(--panel2);color:var(--txt);padding:11px}
input{width:100%;text-align:center}
input::placeholder{color:var(--dim)}
button{cursor:pointer}
button:focus-visible,input:focus-visible,select:focus-visible{outline:2px solid var(--org);outline-offset:1px}
.btn-go{flex:0 0 84px;background:var(--org);color:#0a0a0c;border-color:var(--org);font-weight:700;letter-spacing:1px}
.jog{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:10px}
.jog button{padding:14px 0;font-size:16px}
.jog .lbl{background:none;border:none}
.jog select{padding:8px}
.presets{display:grid;grid-template-columns:repeat(5,1fr);gap:8px;margin-bottom:14px}
.presets button{padding:10px 0;font-size:12px;color:var(--dim)}
.act{display:grid;grid-template-columns:2fr 1fr 1fr;gap:8px}
.btn-stop{background:#2a0d06;border-color:var(--org);color:var(--org);font-weight:700;letter-spacing:2px;padding:15px}
.foot{margin-top:16px;border-top:1px solid var(--line);padding-top:12px;
  display:flex;flex-wrap:wrap;gap:6px 16px;font-size:11px;color:var(--dim)}
.led{display:inline-flex;align-items:center;gap:6px}
.led i{width:8px;height:8px;border-radius:50%;background:#33333c;transition:background .2s}
.led.on i{background:var(--org);box-shadow:0 0 7px var(--org)}
.led.ok i{background:var(--ok);box-shadow:0 0 7px var(--ok)}
.kv b{color:var(--txt);font-weight:500}
.bars{display:inline-flex;align-items:flex-end;gap:2px;height:11px;vertical-align:-1px}
.bars i{width:3px;background:#33333c;border-radius:1px}
.bars i:nth-child(1){height:4px}.bars i:nth-child(2){height:6px}
.bars i:nth-child(3){height:8px}.bars i:nth-child(4){height:11px}
.bars i.on{background:var(--ok)}
#toast{position:fixed;left:50%;bottom:18px;transform:translateX(-50%) translateY(20px);
  background:var(--panel);border:1px solid var(--org);color:var(--txt);padding:10px 16px;
  border-radius:10px;font-size:13px;opacity:0;pointer-events:none;transition:.25s;z-index:9}
#toast.show{opacity:1;transform:translateX(-50%) translateY(0)}
.overlay{position:fixed;inset:0;background:rgba(0,0,0,.6);display:none;align-items:center;
  justify-content:center;z-index:20;padding:20px}
.overlay.show{display:flex}
.ovbox{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:20px;max-width:340px;width:100%}
.ovbox h3{font-family:'Chakra Petch',sans-serif;color:var(--org);margin:0 0 10px;font-size:13px;letter-spacing:1px}
.ovbox hr{border:none;border-top:1px solid var(--line);margin:14px 0}
.ovrow{display:flex;justify-content:space-between;gap:12px;font-size:13px;padding:4px 0;color:var(--dim)}
.ovrow b{color:var(--txt);font-weight:500}
.lg{display:inline-block;width:12px;height:12px;border-radius:50%;vertical-align:-2px;margin-right:5px}
.lg.cur{background:var(--org)}
.lg.tgt{border:1.5px dashed var(--org)}
.ovclose{width:100%;margin-top:14px;background:var(--panel2)}
body.kiosk .go-row,body.kiosk .jog,body.kiosk .presets,body.kiosk .act{display:none}
@keyframes pulse{0%,100%{opacity:.18;r:13}50%{opacity:.05;r:18}}
.curglow{animation:pulse 2.4s ease-in-out infinite}
@media (prefers-reduced-motion: reduce){*{transition:none!important;animation:none!important}}
</style></head><body>
<div class="wrap">
  <header>
    <h1>PYRO ROTATOR</h1><span class="call">PJ4</span>
    <div class="hgroup">
      <span class="pill" id="link"><b>&#9679; LIVE</b> &middot; <span id="mode">MANUAL</span></span>
      <button class="topbtn accent-btn" title="Accent color"><span class="accent-btn-dot"></span><input type="color" id="accentPick" value="#ff4a1c"></button>
      <button class="topbtn" id="proto" title="TCP 4533 protocol: rotctld / SuperRot" style="width:auto;padding:0 9px;font-size:11px;letter-spacing:1px">ROTCTLD</button>
      <button class="topbtn" id="elmode" title="Toggle El 90/180">90°</button>
      <button class="topbtn" id="help" title="Shortcuts">?</button>
      <button class="topbtn" id="kiosk" title="Kiosk view">&#9974;</button>
    </div>
  </header>

  <div class="tele">
    <div class="t"><div class="lbl">AZIMUTH</div>
      <div class="v"><span id="az">--</span><span class="u">&deg;</span></div>
      <div class="meta">tgt <span id="taz">--</span>&deg; &middot; <span class="d" id="daz">&Delta;0</span></div></div>
    <div class="t"><div class="lbl">ELEVATION</div>
      <div class="v"><span id="el">--</span><span class="u">&deg;</span></div>
      <div class="meta">tgt <span id="tel">--</span>&deg; &middot; <span class="d" id="del">&Delta;0</span></div></div>
  </div>

  <div class="scopewrap">
    <svg id="scope" viewBox="0 0 320 320" aria-label="Sky scope. Tap to aim.">
      <circle class="ring rim" cx="160" cy="160" r="140"/>
      <circle class="ring" cx="160" cy="160" r="93.3"/>
      <circle class="ring" cx="160" cy="160" r="46.7"/>
      <circle cx="160" cy="160" r="1.5" fill="#3a3a44"/>
      <g id="grid"></g>
      <text class="card" x="160" y="14" text-anchor="middle">N</text>
      <text class="card" x="312" y="164" text-anchor="middle">E</text>
      <text class="card" x="160" y="316" text-anchor="middle">S</text>
      <text class="card" x="8" y="164" text-anchor="middle">W</text>
      <text class="elnum" x="164" y="118">60</text>
      <text class="elnum" x="164" y="71">30</text>
      <g id="trail"></g>
      <line id="beam" class="beam" x1="160" y1="160" x2="160" y2="160"/>
      <line id="lead" class="lead" x1="160" y1="160" x2="160" y2="160"/>
      <circle id="tgt" class="tgt" cx="160" cy="160" r="6"/>
      <circle id="aim" class="aim" cx="160" cy="160" r="7" style="display:none"/>
      <circle id="curglow" class="curglow" cx="160" cy="160" r="13"/>
      <circle id="cur" class="cur" cx="160" cy="160" r="5"/>
    </svg>
    <span class="hint">tap / drag to aim</span>
    <span class="sstat" id="sstat"></span>
  </div>

  <div class="go-row">
    <input id="iaz" type="number" inputmode="decimal" placeholder="Az" min="0" max="360" step="0.1">
    <input id="iel" type="number" inputmode="decimal" placeholder="El" min="0" max="180" step="0.1">
    <button class="btn-go" id="go">GO</button>
  </div>

  <div class="jog">
    <span class="lbl"></span><button data-jog="el" data-d="1">El &#9650;</button><span class="lbl"></span>
    <button data-jog="az" data-d="-1">&#9664; Az</button>
    <select id="step" aria-label="Jog step"><option>1</option><option selected>5</option><option>10</option></select>
    <button data-jog="az" data-d="1">Az &#9654;</button>
    <span class="lbl"></span><button data-jog="el" data-d="-1">El &#9660;</button><span class="lbl"></span>
  </div>

  <div class="presets">
    <button data-goto="0,90">Zenith</button>
    <button data-goto="0,20">N</button>
    <button data-goto="90,20">E</button>
    <button data-goto="180,20">S</button>
    <button data-goto="270,20">W</button>
  </div>

  <div class="act">
    <button class="btn-stop" id="stop">&#9632; STOP</button>
    <button id="home">HOME</button>
    <button id="park">PARK</button>
  </div>

  <div class="foot">
    <span class="led" id="ledHome"><i></i>homed</span>
    <span class="led" id="ledMove"><i></i>slewing</span>
    <span class="kv">ip <b id="ip">--</b></span>
    <span class="kv">rotctld <b>:4533</b></span>
    <span class="kv" id="rssiKv" style="display:none"><span class="bars" id="bars"><i></i><i></i><i></i><i></i></span> <b id="rssi">--</b></span>
    <span class="kv" id="upKv" style="display:none">up <b id="uptime">--</b></span>
  </div>
</div>

<div class="overlay" id="ov">
  <div class="ovbox">
    <h3>CONTROLS</h3>
    <div class="ovrow"><span>Tap / drag scope</span><b>aim antenna</b></div>
    <div class="ovrow"><span>Arrow keys</span><b>jog az / el</b></div>
    <div class="ovrow"><span>Enter (in field)</span><b>go</b></div>
    <div class="ovrow"><span>S / H</span><b>stop / home</b></div>
    <hr>
    <h3>SCOPE</h3>
    <div class="ovrow"><span><i class="lg cur"></i>filled dot</span><b>current bearing</b></div>
    <div class="ovrow"><span><i class="lg tgt"></i>dashed ring</span><b>target</b></div>
    <button class="ovclose" id="ovclose">Close</button>
  </div>
</div>
<div id="toast"></div>

<script>
const $=s=>document.querySelector(s);
(function(){const c=localStorage.getItem("accent");if(c){document.documentElement.style.setProperty("--org",c);document.getElementById("accentPick").value=c;}})();
document.getElementById("accentPick").addEventListener("input",e=>{const c=e.target.value;document.documentElement.style.setProperty("--org",c);localStorage.setItem("accent",c);});
const R=140,CX=160,CY=160;let ELMAX=90;
const EASE=matchMedia("(prefers-reduced-motion: reduce)").matches?1:0.18;
const wrap=(d)=>((d+540)%360)-180;
const azelXY=(az,el)=>{const r=R*(1-Math.max(0,Math.min(ELMAX,el))/ELMAX),a=az*Math.PI/180;
  return [CX+r*Math.sin(a),CY-r*Math.cos(a)];};
const xyAzel=(x,y)=>{const dx=x-CX,dy=y-CY,r=Math.hypot(dx,dy);
  let el=ELMAX*(1-Math.min(r,R)/R),az=Math.atan2(dx,-dy)*180/Math.PI;if(az<0)az+=360;return [az,el];};

// build az spokes once
const NS="http://www.w3.org/2000/svg",grid=$("#grid");
for(let a=0;a<360;a+=30){const [x,y]=azelXY(a,0);const l=document.createElementNS(NS,"line");
  l.setAttribute("x1",CX);l.setAttribute("y1",CY);l.setAttribute("x2",x);l.setAttribute("y2",y);
  l.setAttribute("class","spoke");grid.appendChild(l);}

let toastT;
const toast=m=>{const t=$("#toast");t.textContent=m;t.classList.add("show");
  clearTimeout(toastT);toastT=setTimeout(()=>t.classList.remove("show"),1800);};
const send=(u,m)=>fetch(u).then(()=>m&&toast(m)).catch(()=>toast("Link lost"));
const stepv=()=>parseFloat($("#step").value);
const goTo=(az,el)=>send(`/api/goto?az=${az}&el=${el}`,`Slewing to ${Math.round(az)}\u00B0 / ${Math.round(el)}\u00B0`);

$("#go").addEventListener("click",()=>{const a=$("#iaz").value,e=$("#iel").value;
  if(a!==""&&e!=="")goTo(parseFloat(a),parseFloat(e));});
["#iaz","#iel"].forEach(id=>$(id).addEventListener("keydown",e=>{if(e.key==="Enter")$("#go").click();}));
document.querySelectorAll("[data-jog]").forEach(b=>b.addEventListener("click",()=>
  send(`/api/jog?axis=${b.dataset.jog}&delta=${(+b.dataset.d)*stepv()}`)));
document.querySelectorAll("[data-goto]").forEach(b=>b.addEventListener("click",()=>{
  const [a,e]=b.dataset.goto.split(",").map(Number);goTo(a,e);}));
$("#elmode").addEventListener("click",()=>fetch("/api/elmode").then(r=>r.json()).then(d=>{ELMAX=d.elMax;$("#elmode").textContent=ELMAX+"°";}));
const setProto=p=>{const b=$("#proto");b.textContent=p.toUpperCase();b.style.color=p==="superrot"?"var(--org)":"var(--dim)";b.style.borderColor=p==="superrot"?"var(--org)":"var(--line)";};
$("#proto").addEventListener("click",()=>fetch("/api/proto").then(r=>r.json()).then(d=>{setProto(d.proto);toast("TCP 4533 -> "+d.proto.toUpperCase());}));
$("#stop").addEventListener("click",()=>send("/api/stop","Stopped"));
$("#home").addEventListener("click",()=>send("/api/home","Homing"));
$("#park").addEventListener("click",()=>send("/api/park","Parking"));

// overlay + kiosk
$("#help").addEventListener("click",()=>$("#ov").classList.add("show"));
$("#ovclose").addEventListener("click",()=>$("#ov").classList.remove("show"));
$("#ov").addEventListener("click",e=>{if(e.target.id==="ov")$("#ov").classList.remove("show");});
$("#kiosk").addEventListener("click",()=>{
  if(document.fullscreenElement){document.exitFullscreen&&document.exitFullscreen();}
  else{const r=document.documentElement.requestFullscreen&&document.documentElement.requestFullscreen();
    r&&r.then?r.then(()=>document.body.classList.add("kiosk")).catch(()=>document.body.classList.toggle("kiosk"))
     :document.body.classList.toggle("kiosk");}});
document.addEventListener("fullscreenchange",()=>{if(!document.fullscreenElement)document.body.classList.remove("kiosk");});

addEventListener("keydown",e=>{
  if(e.key==="Escape"){$("#ov").classList.remove("show");return;}
  if(/input|select/i.test(e.target.tagName))return;
  const s=stepv(),k=e.key;
  if(k==="ArrowRight")send(`/api/jog?axis=az&delta=${s}`);
  else if(k==="ArrowLeft")send(`/api/jog?axis=az&delta=${-s}`);
  else if(k==="ArrowUp")send(`/api/jog?axis=el&delta=${s}`);
  else if(k==="ArrowDown")send(`/api/jog?axis=el&delta=${-s}`);
  else if(k==="s"||k==="S")send("/api/stop","Stopped");
  else if(k==="h"||k==="H")send("/api/home","Homing");
  else return;
  e.preventDefault();});

// tap / drag to aim
const svg=$("#scope"),aim=$("#aim");let dragging=false;
const evtAzel=ev=>{const r=svg.getBoundingClientRect();
  return xyAzel((ev.clientX-r.left)*320/r.width,(ev.clientY-r.top)*320/r.height);};
const showAim=(az,el)=>{const[x,y]=azelXY(az,el);aim.style.display="";
  aim.setAttribute("cx",x);aim.setAttribute("cy",y);
  $("#iaz").value=az.toFixed(1);$("#iel").value=el.toFixed(1);};
svg.addEventListener("pointerdown",e=>{dragging=true;svg.setPointerCapture(e.pointerId);
  const[a,el]=evtAzel(e);showAim(a,el);});
svg.addEventListener("pointermove",e=>{if(!dragging)return;const[a,el]=evtAzel(e);showAim(a,el);});
svg.addEventListener("pointerup",e=>{if(!dragging)return;dragging=false;
  const[a,el]=evtAzel(e);aim.style.display="none";goTo(a,el);});

// state: cur/tgt = latest real; disp/dispT = animated
let cur={az:0,el:0},tgt={az:0,el:0},disp={az:0,el:0},dispT={az:0,el:0},moving=false,inited=false;
const trail=[];let last=null;
const setLed=(id,on,cls)=>{const e=$(id);e.classList.toggle("on",on&&cls!=="ok");
  e.classList.toggle("ok",on&&cls==="ok");};
const drawTrail=()=>{const g=$("#trail");g.innerHTML="";
  trail.forEach((p,i)=>{const[x,y]=azelXY(p.az,p.el);const c=document.createElementNS(NS,"circle");
    c.setAttribute("cx",x);c.setAttribute("cy",y);c.setAttribute("r",2);c.setAttribute("class","traildot");
    c.setAttribute("opacity",(i/trail.length*0.5).toFixed(2));g.appendChild(c);});};

async function poll(){
  try{
    const s=await (await fetch("/api/status")).json();
    $(".wrap").classList.remove("stale");
    const lk=$("#link");lk.classList.remove("lost");lk.querySelector("b").innerHTML="&#9679; LIVE";
    $("#az").textContent=s.az.toFixed(1);$("#el").textContent=s.el.toFixed(1);
    $("#taz").textContent=s.taz.toFixed(1);$("#tel").textContent=s.tel.toFixed(1);
    const daz=wrap(s.taz-s.az),del=s.tel-s.el;
    $("#daz").textContent=(daz>=0?"\u0394+":"\u0394")+daz.toFixed(1);
    $("#del").textContent=(del>=0?"\u0394+":"\u0394")+del.toFixed(1);
    cur={az:s.az,el:s.el};tgt={az:s.taz,el:s.tel};moving=s.moving;
    if(s.elMax&&s.elMax!==ELMAX){ELMAX=s.elMax;$("#elmode").textContent=ELMAX+"°";}
    if(s.proto&&s.proto!==$("#proto").textContent.toLowerCase())setProto(s.proto);
    if(!inited){disp={...cur};dispT={...tgt};inited=true;}
    if(!last||Math.hypot(s.az-last.az,s.el-last.el)>0.3){trail.push({az:s.az,el:s.el});
      if(trail.length>60)trail.shift();drawTrail();last={az:s.az,el:s.el};}
    const dist=Math.max(Math.abs(daz),Math.abs(del)),ss=$("#sstat");
    if(s.moving){ss.className="sstat";ss.textContent="SLEWING \u00B7 "+dist.toFixed(1)+"\u00B0 to go";}
    else if(dist<0.6){ss.className="sstat ok";ss.textContent="ON TARGET";}
    else{ss.textContent="";}
    $("#mode").textContent=s.control?("rotctld "+s.control):"MANUAL";
    setLed("#ledHome",s.homed,"ok");setLed("#ledMove",s.moving);
    $("#ip").textContent=s.ip||"--";
    if(s.rssi!==undefined){$("#rssiKv").style.display="";$("#rssi").textContent=s.rssi;
      const lv=s.rssi>-55?4:s.rssi>-65?3:s.rssi>-72?2:s.rssi>-80?1:0;
      document.querySelectorAll("#bars i").forEach((b,i)=>b.classList.toggle("on",i<lv));}
    if(s.uptime!==undefined){$("#upKv").style.display="";$("#uptime").textContent=s.uptime;}
  }catch(e){
    const lk=$("#link");lk.classList.add("lost");lk.querySelector("b").innerHTML="&#9675; LINK LOST";
    $(".wrap").classList.add("stale");
  }
}

function raf(){
  disp.az=(disp.az+wrap(cur.az-disp.az)*EASE+360)%360;
  disp.el+=(cur.el-disp.el)*EASE;
  dispT.az=(dispT.az+wrap(tgt.az-dispT.az)*EASE+360)%360;
  dispT.el+=(tgt.el-dispT.el)*EASE;
  const[cx,cy]=azelXY(disp.az,disp.el),[tx,ty]=azelXY(dispT.az,dispT.el);
  $("#cur").setAttribute("cx",cx);$("#cur").setAttribute("cy",cy);
  $("#curglow").setAttribute("cx",cx);$("#curglow").setAttribute("cy",cy);
  $("#beam").setAttribute("x2",cx);$("#beam").setAttribute("y2",cy);
  $("#tgt").setAttribute("cx",tx);$("#tgt").setAttribute("cy",ty);
  const ld=$("#lead");ld.setAttribute("x1",cx);ld.setAttribute("y1",cy);
  ld.setAttribute("x2",tx);ld.setAttribute("y2",ty);ld.style.opacity=moving?0.5:0;
  requestAnimationFrame(raf);
}
requestAnimationFrame(raf);
setInterval(poll,300);poll();
</script></body></html>
)HTML";
