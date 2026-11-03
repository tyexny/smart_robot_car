"""Read-only browser heatmap, using the existing thermal monitor's frames."""
import secrets

PAGE = r'''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>MLX90640 实时热成像</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#101621;color:#edf3fa;font:17px system-ui,sans-serif}
main{max-width:1360px;margin:24px auto;padding:0 20px}h1{font-size:27px;margin:12px 0}.bar,.values{display:flex;gap:14px;align-items:center;flex-wrap:wrap}
a{color:#8dcaff}button,select{background:#263750;border:1px solid #607c9f;border-radius:8px;padding:9px 14px;color:white;font:inherit;cursor:pointer}
.view{position:relative;width:960px;max-width:100%;background:#05070b;border:1px solid #3b4b63;border-radius:12px;overflow:hidden;margin:16px 0}
canvas{display:block;width:100%;height:auto;aspect-ratio:4/3;image-rendering:auto;visibility:hidden}.wait{position:absolute;inset:0;display:grid;place-items:center;padding:25px;text-align:center;color:#cfdae8;background:#101621d9}
.values{padding:12px 0}.values strong{font-size:26px}.muted{color:#b4c2d5;line-height:1.65}.scale{height:15px;border-radius:5px;background:linear-gradient(90deg,#08000e,#301068,#264ad0,#00c9c9,#ffee6b,#fffdf4)}
.legend{display:flex;justify-content:space-between;font-size:15px;margin:5px 0 16px}#state[data-ok=false]{color:#ffb58c}#state[data-ok=true]{color:#9ee6ba}
@media(max-width:520px){main{margin-top:15px;padding:0 12px}h1{font-size:23px}body{font-size:15px}.values strong{font-size:22px}}
</style></head><body><main>
<div class="bar"><h1>MLX90640 实时热成像</h1><a id="back" href="#">返回任务页</a></div>
<p class="muted">32 × 24 原始热图 · 约 4 幅/秒 · 浏览器放大显示</p>
<div class="bar"><label for="displaySize">显示尺寸</label><select id="displaySize"><option value="640">640 × 480</option><option value="960" selected>960 × 720</option><option value="1280">1280 × 960</option></select><button id="smooth" type="button">切换像素显示</button><button id="rotate" type="button">旋转 180°</button><span id="state" role="status" data-ok="false">正在连接传感器…</span></div>
<div class="values"><span>最低 <strong id="minimum">--</strong></span><span>最高 <strong id="maximum">--</strong></span><span id="limit"></span></div>
<div class="view" id="heatmapView"><canvas id="image" width="32" height="24" aria-label="实时红外温度图"></canvas><div id="wait" class="wait">等待完整温度画面</div></div>
<div class="scale"></div><div class="legend"><span id="cold">低温</span><span>当前画面自动色标</span><span id="hot">高温</span></div>
<p class="muted">显示尺寸仅改变放大效果，原始采集仍为 32×24，不增加真实测温细节。窗口较小时自动适配。颜色按当前画面温度范围自动调整，报警以温度数值和阈值为准。</p>
<p class="muted" id="tip">此页只查看热图，不发送车辆运动指令。</p>
</main><script>
const token=decodeURIComponent(location.pathname.split('/')[1]||''),canvas=document.getElementById('image'),ctx=canvas.getContext('2d'),wait=document.getElementById('wait');
const byId=id=>document.getElementById(id);let smooth=true,rotated=false,lastFrame=-1;
byId('back').href='/'+encodeURIComponent(token);
if(document.body.dataset.standalone==='true'){byId('back').hidden=true;byId('tip').textContent='独立预览只查看热图，不发送车辆指令。正常任务启动后，请从任务网页点击“查看热成像”。前台预览可用 Ctrl+C 停止。'}
function displaySize(value){const width=[640,960,1280].includes(Number(value))?Number(value):960;byId('heatmapView').style.width=width+'px';byId('displaySize').value=String(width)}
let preferredSize=960;try{preferredSize=localStorage.getItem('mlx90640-preview-size')||960}catch(error){}displaySize(preferredSize);
byId('displaySize').onchange=()=>{displaySize(byId('displaySize').value);try{localStorage.setItem('mlx90640-preview-size',byId('displaySize').value)}catch(error){}};
byId('smooth').onclick=()=>{smooth=!smooth;canvas.style.imageRendering=smooth?'auto':'pixelated';byId('smooth').textContent=smooth?'切换像素显示':'切换平滑显示'};
byId('rotate').onclick=()=>{rotated=!rotated;canvas.style.transform=rotated?'rotate(180deg)':''};
const stops=[[8,0,14],[48,16,104],[38,74,208],[0,201,201],[255,238,107],[255,253,244]];
function color(value){const at=Math.max(0,Math.min(1,value))*(stops.length-1),i=Math.min(stops.length-2,Math.floor(at)),f=at-i;return stops[i].map((v,c)=>Math.round(v+(stops[i+1][c]-v)*f))}
function paint(frame){const values=frame.pixels;if(!Array.isArray(values)||values.length!==768||values.some(v=>!Number.isFinite(v)))throw Error('热图数据不完整');
const minimum=Math.min(...values),maximum=Math.max(...values),span=Math.max(1,maximum-minimum),image=ctx.createImageData(32,24);
values.forEach((value,i)=>{const rgb=color((value-minimum)/span);image.data.set([...rgb,255],i*4)});ctx.putImageData(image,0,0);
byId('minimum').textContent=minimum.toFixed(2)+'°C';byId('maximum').textContent=maximum.toFixed(2)+'°C';byId('cold').textContent=minimum.toFixed(1)+'°C';byId('hot').textContent=(minimum+span).toFixed(1)+'°C';
canvas.style.visibility='visible';wait.style.display='none'}
async function refresh(){const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),2500);try{
const response=await fetch('/api/thermal/frame',{headers:{'X-Task-Token':token},cache:'no-store',signal:controller.signal}),frame=await response.json();
if(!response.ok||!frame.frame_available)throw Error(frame.error||'传感器暂未就绪');
if(frame.frame_id!==lastFrame){paint(frame);lastFrame=frame.frame_id}
byId('limit').textContent='报警阈值 '+frame.threshold_c+'°C';byId('state').dataset.ok='true';byId('state').textContent='实时画面 · '+frame.age_ms+' ms';
}catch(error){canvas.style.visibility='hidden';wait.style.display='grid';wait.textContent=error.name==='AbortError'?'画面连接超时':error.message;byId('state').dataset.ok='false';byId('state').textContent='画面未更新';lastFrame=-1}
finally{clearTimeout(timer);setTimeout(refresh,250)}}refresh();
</script></body></html>'''


def register_thermal_preview(app, token, frame_provider=None, standalone=False):
    from flask import abort, jsonify, request

    @app.get('/<access_token>/thermal')
    def thermal_page(access_token):
        if not secrets.compare_digest(access_token, token):
            abort(403)
        page = PAGE.replace('<body>', '<body data-standalone="true">') if standalone else PAGE
        return page, 200, {'Cache-Control': 'no-store'}

    @app.get('/api/thermal/frame')
    def thermal_frame():
        if not secrets.compare_digest(request.headers.get('X-Task-Token', ''), token):
            abort(403)
        if frame_provider is None:
            return jsonify(error='红外监测未启用', frame_available=False), 503
        frame = frame_provider()
        response = jsonify(frame)
        response.headers['Cache-Control'] = 'no-store'
        return response, 200 if frame.get('frame_available') else 503
