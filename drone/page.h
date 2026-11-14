#pragma once
#include <Arduino.h>

static const char PAGE[] PROGMEM = R"rawliteral(<!doctype html>
<meta name=viewport content="width=device-width,initial-scale=1">
<meta name=color-scheme content="light dark">
<style>
body{font-family:system-ui,sans-serif;margin:12px;font-size:18px}
button{font-size:20px;padding:14px;margin:4px;border-radius:8px;border:0;color:#fff}
#stop{background:#d32f2f;width:100%;font-size:34px;padding:26px}
#arm{background:#2e7d32}
input[type=range]{width:100%;height:44px}
pre{background:rgba(128,128,128,.2);padding:8px;border-radius:8px;font-size:15px}
</style>
<button id=stop onclick="stopNow()">STOP</button>
<div><button id=arm onclick="armNow()">ARM</button>
<label><input type=checkbox id=stab onchange="send()"> Stabilise</label></div>
<div>Motor <select id=sel onchange="send()">
<option value=0>ALL<option value=1>FL<option value=2>FR<option value=3>BL<option value=4>BR</select></div>
<div>Throttle <b id=tv>0</b>%</div>
<input type=range id=thr min=0 max=50 value=0 oninput="tv.textContent=this.value;send()">
<pre id=o>...</pre>
<script>
let pend=false;
function send(){if(pend)return;pend=true;setTimeout(()=>{pend=false;
fetch('/set?thr='+thr.value+'&sel='+sel.value+'&stab='+(stab.checked?1:0)).catch(()=>{})},100)}
function zero(){thr.value=0;tv.textContent=0}
async function stopNow(){zero();try{await fetch('/stop')}catch(e){}}
async function armNow(){zero();try{await fetch('/arm',{method:'POST'})}catch(e){}}
document.addEventListener('keydown',e=>{if(e.code=='Space'){e.preventDefault();stopNow()}});
async function poll(){try{const d=await (await fetch('/data')).json();
if(!d.armed)zero();
o.textContent=Object.keys(d).map(k=>k.padEnd(12)+d[k]).join('\n')}
catch(e){o.textContent='no connection - motors cut'}setTimeout(poll,150)}
poll()
</script>)rawliteral";