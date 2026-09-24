-- HELI 3D 0.4.3 | EdgeTX 2.12.2 / TX16S MK3 / Mode 1
-- Original low-poly graphics, inspired by the selected helicopter.
-- Simplified fixed-headspeed collective-pitch model, not a flight predictor.
-- No RF/model writes. Run on a dedicated model with both RF modules OFF.

local CFG = {
  revAil=1, revEle=1, revRud=1, revCol=1,
  cyclicRate=276, yawRate=222, expo=0.28,
  maxThrust=31.2, gravity=9.81, drag=0.22,
}
local models = {
  {name="GOBLIN RAW IL 700", short="RAW IL 700", wide=true,
   rgb={173,238,36}, shade={70,140,22}},
  {name="XLPOWER SPECTER V2 700", short="SPECTER V2 700", wide=false,
   rgb={255,127,40}, shade={174,61,24}},
}
local speeds={{name="FAST",factor=1},{name="FASTER",factor=1.4},
              {name="VERY FAST",factor=1.8},{name="ULTRA",factor=2.2}}
local speedIndex,speedLatch=1,false
local controlMode,radioMode,settingsRow=1,1,1
local radioDetected=false
local soundEnabled,lastTone=true,-100
local selected, mode, latch = 1, "menu", false
local s, u, R, C = {}, {a=0,e=0,r=0,c=0}, {}, {}
local lastTime, rotor, hint = 0, 0, ""
local scale, ox, oy = 1, 0, 0
local view={fx=0,fy=1,fz=0,rx=1,ry=0,ux=0,uy=0,uz=1,focal=500,distance=18}
local pauseCol, suppressBreak, suppressAt = 0, false, 0
local frame, parts, partCount = {}, {}, 0
local sin, cos, sqrt, abs, pi = math.sin, math.cos, math.sqrt, math.abs, math.pi
local function clamp(v,a,b) return math.max(a,math.min(b,v)) end
local function round(v) return math.floor(v+0.5) end
local function eventIs(e,k) return k~=nil and e==k end

-- Body axes: +X right, +Y nose, +Z rotor mast. Quaternion maps body to world.
local function matrix()
  local w,x,y,z=s.qw,s.qx,s.qy,s.qz
  R[1]=1-2*(y*y+z*z); R[2]=2*(x*y-z*w); R[3]=2*(x*z+y*w)
  R[4]=2*(x*y+z*w); R[5]=1-2*(x*x+z*z); R[6]=2*(y*z-x*w)
  R[7]=2*(x*z-y*w); R[8]=2*(y*z+x*w); R[9]=1-2*(x*x+y*y)
end
local function reset()
  s={x=0,y=0,z=0,vx=0,vy=0,vz=0,qw=1,qx=0,qy=0,qz=0,
     wx=0,wy=0,wz=0,time=0}
  mode,hint="menu",""
  matrix()
end
local function readAxis(name,reverse,deadband)
  local v=getValue(name)
  if type(v)~="number" then return 0 end
  v=clamp(v/1024*reverse,-1,1)
  if abs(v)<=deadband then return 0 end
  return (v>0 and v-deadband or v+deadband)/(1-deadband)
end
local function readInputs()
  local ele=readAxis("ele",1,.025)
  local thr=readAxis("thr",1,.025)
  local leftV,rightV=ele,thr
  if radioMode==2 then leftV,rightV=thr,ele end
  if controlMode==1 then u.e,u.c=leftV*CFG.revEle,rightV*CFG.revCol
  else u.e,u.c=rightV*CFG.revEle,leftV*CFG.revCol end
  u.a=readAxis("ail",CFG.revAil,.025)
  u.r=readAxis("rud",CFG.revRud,.025)
end
local function curved(v) return v*(1-CFG.expo)+v*v*v*CFG.expo end

local function physics(dt)
  -- Rate control, with brief servo/FBL response. NO attitude auto-level.
  local k=1-math.exp(-dt/0.045)
  local factor=speeds[speedIndex].factor
  local rate=CFG.cyclicRate*factor*pi/180
  s.wx=s.wx+(-curved(u.e)*rate-s.wx)*k
  s.wy=s.wy+(curved(u.a)*rate-s.wy)*k
  s.wz=s.wz+(-curved(u.r)*CFG.yawRate*factor*pi/180-s.wz)*k
  local w,x,y,z=s.qw,s.qx,s.qy,s.qz
  local a,b,c=s.wx*dt/2,s.wy*dt/2,s.wz*dt/2
  w,x,y,z=w-x*a-y*b-z*c, x+w*a+y*c-z*b,
          y+w*b+z*a-x*c, z+w*c+x*b-y*a
  local n=sqrt(w*w+x*x+y*y+z*z)
  s.qw,s.qx,s.qy,s.qz=w/n,x/n,y/n,z/n
  matrix()
  -- Signed collective: inverted rotor normal + negative pitch = UP thrust.
  local thrust=u.c*CFG.maxThrust*factor
  local damping=1+CFG.drag*dt
  s.vx=(s.vx+R[3]*thrust*dt)/damping
  s.vy=(s.vy+R[6]*thrust*dt)/damping
  s.vz=(s.vz+(R[9]*thrust-CFG.gravity)*dt)/damping
  s.x,s.y,s.z=s.x+s.vx*dt,s.y+s.vy*dt,s.z+s.vz*dt
  s.time=s.time+dt

  -- Soft practice floor: preserve attitude/control and never require a reset.
  local clearance=0.9*sqrt(math.max(0,1-R[9]*R[9]))
                  +0.35*math.max(0,-R[9])
  if s.z<=clearance then
    s.z=clearance
    s.vz=math.max(0,s.vz)
    s.vx,s.vy=s.vx/(1+14*dt),s.vy/(1+14*dt)
  end
  -- Free practice: position and altitude never terminate a flight.
end

local function X(v) return round(ox+v*scale) end
local function Y(v) return round(oy+v*scale) end
local function ink(c) lcd.setColor(CUSTOM_COLOR,c) end
local function box(x,y,w,h,c)
  ink(c)
  lcd.drawFilledRectangle(X(x),Y(y),math.max(1,round(w*scale)),
                          math.max(1,round(h*scale)),CUSTOM_COLOR)
end
local function line(x,y,a,b,c)
  local dx,dy=a-x,b-y;local lo,hi=0,1
  local ps={-dx,dx,-dy,dy};local qs={x,799-x,y,479-y}
  for i=1,4 do
    if abs(ps[i])<1e-9 then if qs[i]<0 then return end
    else local t=qs[i]/ps[i]
      if ps[i]<0 then lo=math.max(lo,t) else hi=math.min(hi,t) end
    end
  end
  if lo>hi then return end
  a,b=x+dx*hi,y+dy*hi;x,y=x+dx*lo,y+dy*lo
  ink(c)
  lcd.drawLine(X(x),Y(y),X(a),Y(b),SOLID,CUSTOM_COLOR)
end
local function text(x,y,t,c)
  ink(c or C.white)
  lcd.drawText(X(x),Y(y),t,SMLSIZE+CUSTOM_COLOR)
end
-- Pilot stands at a fixed world position, 1.7m above the ground.
local function updateView()
  local dx,dy,dz=s.x,s.y+18,s.z-1.7
  if dx*dx+dy*dy+dz*dz<.0025 then dy=.05 end
  local horizontal=sqrt(dx*dx+dy*dy)
  local distance=sqrt(dx*dx+dy*dy+dz*dz)
  view.fx,view.fy,view.fz=dx/distance,dy/distance,dz/distance
  if horizontal>.001 then view.rx,view.ry=dy/horizontal,-dx/horizontal end
  view.ux,view.uy,view.uz=view.ry*view.fz,-view.rx*view.fz,horizontal/distance
  view.focal=400; view.distance=distance
end
local function project(x,y,z)
  local dx,dy,dz=x,y+18,z-1.7
  local depth=dx*view.fx+dy*view.fy+dz*view.fz
  local k=view.focal/math.max(.05,depth)
  return 400+(dx*view.rx+dy*view.ry)*k,
         247-(dx*view.ux+dy*view.uy+dz*view.uz)*k,depth
end
local function groundLine(x1,y1,x2,y2,c)
  local a,b,d1=project(x1,y1,0);local e,f,d2=project(x2,y2,0)
  if d1<.2 and d2<.2 then return end
  if d1<.2 then
    local t=(.2-d1)/(d2-d1);a,b=project(x1+(x2-x1)*t,y1+(y2-y1)*t,0)
  elseif d2<.2 then
    local t=(.2-d2)/(d1-d2);e,f=project(x2+(x1-x2)*t,y2+(y1-y2)*t,0)
  end
  -- Clip perspective ground lines to the flight viewport.
  local dx,dy=e-a,f-b;local lo,hi=0,1
  local ps={-dx,dx,-dy,dy};local qs={a,799-a,b-66,398-b}
  for i=1,4 do
    if math.abs(ps[i])<1e-9 then if qs[i]<0 then return end
    else
      local t=qs[i]/ps[i]
      if ps[i]<0 then lo=math.max(lo,t) else hi=math.min(hi,t) end
    end
  end
  if lo<=hi then line(a+dx*lo,b+dy*lo,a+dx*hi,b+dy*hi,c) end
end
local function ground()
  box(0,0,800,480,C.bg)
  box(0,66,800,332,C.sky)
  local horizon=247+view.focal*view.fz/math.max(.001,view.uz)
  local top=clamp(horizon,66,398)
  if top<398 then box(0,top,800,398-top,C.ground) end
  if horizon>66 and horizon<398 then line(0,horizon,799,horizon,C.muted) end
  local gx=math.floor(s.x/10)*10;local gy=math.floor(s.y/10)*10
  for i=-8,8 do
    groundLine(gx+i*10,gy-80,gx+i*10,gy+80,C.grid)
    groundLine(gx-80,gy+i*10,gx+80,gy+i*10,C.grid)
  end
  groundLine(-3,-3,3,-3,C.white);groundLine(3,-3,3,3,C.white)
  groundLine(3,3,-3,3,C.white);groundLine(-3,3,-3,-3,C.white)
  groundLine(-1,1.5,-1,-1.5,C.white);groundLine(1,1.5,1,-1.5,C.white)
  groundLine(-1,0,1,0,C.white)
end

-- Reuse face/line records across frames to avoid persistent allocation churn.
local function addPart(kind,points,c,depth)
  partCount=partCount+1
  local p=parts[partCount]
  if not p then p={}; parts[partCount]=p end
  p.kind,p.points,p.color,p.depth=kind,points,c,depth
  frame[partCount]=p
end
local function depthOrder(a,b) return a.depth<b.depth end
local function helicopter(preview)
  local m=models[selected]
  local hx,hy=project(s.x,s.y,s.z)
  local gx,gy=project(s.x,s.y,0)
  local size=clamp(view.focal/view.distance*0.04,0.06,1.0)
  local r=R
  if preview then
    hx,hy,gx,gy,size=400,315,400,350,1.0
    local c,t=cos(-0.95),sin(-0.95)
    r={c,-t,0,t,c,0,0,0,1}
  end
  local function point(x,y,z,shadow)
    local wx=r[1]*x+r[2]*y+r[3]*z
    local wy=r[4]*x+r[5]*y+r[6]*z
    local wz=r[7]*x+r[8]*y+r[9]*z
    if preview then
      if shadow then return {gx+wx*size,gy-wy*0.55*size,0} end
      return {hx+wx*size,hy-(wy*0.55+wz*0.835)*size,-wy*0.835+wz*.55}
    end
    local sx=wx*view.rx+wy*view.ry
    local sy=wx*view.ux+wy*view.uy+(shadow and 0 or wz*view.uz)
    if shadow then return {gx+sx*size,gy-sy*size,0} end
    return {hx+sx*size,hy-sy*size,-(wx*view.fx+wy*view.fy+wz*view.fz)}
  end
  partCount=0
  local function face(vertices,c)
    local p,d={},0
    for i,v in ipairs(vertices) do
      p[i]=point(v[1],v[2],v[3]); d=d+p[i][3]
    end
    addPart("face",p,c,d/#p)
  end
  local function seg(x,y,z,a,b,c,tint)
    local p,q=point(x,y,z),point(a,b,c)
    addPart("line",{p,q},tint,(p[3]+q[3])/2)
  end
  -- Thin shadow plus vertical locator makes altitude readable.
  local p,q=point(0,32,0,true),point(0,-67,0,true)
  line(p[1],p[2],q[1],q[2],C.shadow)
  p,q=point(-45,0,0,true),point(45,0,0,true)
  line(p[1],p[2],q[1],q[2],C.shadow)
  if not preview and s.z>1 then
    line(gx-4,gy,gx+4,gy,C.muted)
    line(gx,gy-3,gx,gy+3,C.muted)
  end
  -- Skids are below the body: flip visibly ABOVE it when inverted.
  for _,x in ipairs({-14,14}) do
    seg(x,-22,-12,x,24,-12,C.white)
    seg(x,24,-12,x,29,-8,C.white)
    seg(x,-12,-12,x*0.5,-9,0,C.metal)
    seg(x,14,-12,x*0.5,12,0,C.metal)
  end
  local bw=m.wide and 8 or 2.5
  face({{-bw,-10,0},{bw,-10,0},{2.8,-68,3},{-2.8,-68,3}},m.color)
  face({{-bw,-10,0},{-2.8,-68,3},{-2.8,-68,-2},{-bw,-10,-5}},C.dark)
  face({{bw,-10,0},{2.8,-68,3},{2.8,-68,-2},{bw,-10,-5}},m.shadeColor)
  if not m.wide then
    seg(-8,-12,-8,0,-50,-1,C.metal)
    seg(8,-12,-8,0,-50,-1,C.metal)
  end
  face({{0,-59,1},{0,-72,1},{0,-69,19}},m.color)
  seg(0,-66,3,10,-66,3,C.metal)
  local tr=rotor*1.9
  seg(10,-66+cos(tr)*10,3+sin(tr)*10,
      10,-66-cos(tr)*10,3-sin(tr)*10,C.white)
  local nose=m.wide and 37 or 41
  local width=m.wide and 13 or 15
  -- Closed faceted canopy, with distinct top and bottom faces.
  local n={0,nose,0}; local l={-width,12,0}; local rr={width,12,0}
  local bl={-10,-16,1}; local br={10,-16,1}
  local top={0,3,15}; local bottom={0,2,-8}
  face({n,l,top},m.color); face({n,top,rr},m.color)
  face({l,bl,top},m.shadeColor); face({rr,top,br},m.color)
  face({bl,br,top},C.dark)
  face({n,bottom,l},C.belly); face({n,rr,bottom},C.belly)
  face({l,bottom,bl},C.dark); face({rr,br,bottom},C.dark)
  face({bl,bottom,br},C.dark)
  face({{0,31,4},{-8,12,10},{0,3,15}},C.glass)
  face({{0,31,4},{0,3,15},{8,12,10}},C.glass)
  seg(-width,12,1,-10,-12,2,C.white)
  seg(width,12,1,10,-12,2,C.white)
  -- Bright underside stripe distinguishes inverted attitude.
  seg(0,28,-3,0,-11,-7,C.yellow)
  seg(0,-2,10,0,-2,23,C.metal)
  local previous
  for i=0,16 do
    local a=i*2*pi/16
    local v={cos(a)*57,-2+sin(a)*57,23}
    if previous then seg(previous[1],previous[2],previous[3],v[1],v[2],v[3],C.rotor) end
    previous=v
  end
  local bx,by=cos(rotor)*57,sin(rotor)*57
  seg(-bx,-2-by,23,bx,-2+by,23,C.white)
  seg(-bx,-2-by,24,bx,-2+by,24,C.white)
  for i=#frame,partCount+1,-1 do frame[i]=nil end
  table.sort(frame,depthOrder)
  for _,part in ipairs(frame) do
    local v=part.points
    if part.kind=="line" then
      line(v[1][1],v[1][2],v[2][1],v[2][2],part.color)
    else
      ink(part.color)
      for i=2,#v-1 do
        lcd.drawFilledTriangle(X(v[1][1]),Y(v[1][2]),X(v[i][1]),Y(v[i][2]),
                               X(v[i+1][1]),Y(v[i+1][2]),CUSTOM_COLOR)
      end
    end
  end
end

local function stick(x,y,h,v,label)
  box(x-25,y-25,50,50,C.panel)
  line(x-22,y,x+22,y,C.grid); line(x,y-22,x,y+22,C.grid)
  box(x+h*21-3,y-v*21-3,6,6,models[selected].color)
  text(x-32,y+29,label,C.muted)
end
local function ui()
  local m=models[selected]
  box(0,0,800,65,C.panel)
  text(18,8,m.name,m.color)
  text(18,35,"HELI 3D v0.4.3  /  MODE "..controlMode.."  /  RATE CONTROL",C.muted)
  text(564,8,string.format("ALT %.1fm",s.z))
  text(564,35,string.format("COL %+d%%",round(u.c*100)))
  box(0,399,800,81,C.panel)
  if mode=="menu" then
    for i,v in ipairs(models) do
      local x=28+(i-1)*390
      box(x,77,365,42,i==selected and C.active or C.panel)
      text(x+12,88,(i==selected and "> " or "  ")..v.short,
           i==selected and v.color or C.muted)
    end
    for i,v in ipairs(speeds) do
      local x=28+(i-1)*195
      box(x,129,170,35,i==speedIndex and C.active or C.panel)
      text(x+10,136,(i==speedIndex and "> " or "  ")..v.name,
           i==speedIndex and C.yellow or C.muted)
    end
    text(80,177,"LEFT stick L/R: model     RIGHT stick L/R: speed")
    text(80,204,"Center ALL sticks + ENTER. Hold ENTER: MODE / REVERSE")
    text(18,410,"3D: positive / zero / negative collective. No auto-level.")
    text(18,440,hint~="" and hint or "RF OFF model only. EXIT: close",C.yellow)
  else
    local attitude=R[9]<-0.15 and "INVERTED" or (R[9]>0.15 and "UPRIGHT" or "KNIFE EDGE")
    text(20,76,attitude,R[9]<0 and C.yellow or C.muted)
    text(570,76,string.format("SPEED %.1f m/s",sqrt(s.vx*s.vx+s.vy*s.vy)))
    text(570,101,string.format("V/S %+.1f m/s",s.vz),C.muted)
    text(20,103,"GROUND PILOT / "..speeds[speedIndex].name,C.muted)
    if controlMode==1 then
      stick(52,343,u.r,u.e,"RUD/ELE");stick(748,343,u.a,u.c,"AIL/COL")
    else
      stick(52,343,u.r,u.c,"RUD/COL");stick(748,343,u.a,u.e,"AIL/ELE")
    end
    text(18,410,"ENTER: pause/resume   Hold ENTER: reset + model selection")
    text(18,440,"Inverted: NEGATIVE collective. EXIT: pause / close",C.muted)
    if mode~="flying" then
      box(153,139,494,110,C.panel)
      text(172,150,mode=="paused" and "PAUSED" or hint,C.yellow)
      if mode=="paused" then
        text(172,181,string.format("Match COL %+d%%; center cyclic/rudder.",round(pauseCol*100)))
        text(172,211,"ENTER: resume   Hold ENTER: reset",C.muted)
      else
        text(172,184,"Hold ENTER to reset and choose a model.")
        text(172,214,"EXIT: close",C.muted)
      end
    end
  end
end
local function settingsUI()
  box(0,0,800,480,C.panel)
  text(26,15,"CONTROL MODE / REVERSE",C.yellow)
  text(26,44,"Wheel: select row   ENTER: change   EXIT: back",C.muted)
  local keys={"revAil","revEle","revRud","revCol"}
  local labels={"AILERON","ELEVATOR","RUDDER","COLLECTIVE"}
  for i=1,8 do
    local y=78+(i-1)*36
    if i==settingsRow then box(20,y-4,760,36,C.active) end
    local label,value
    if i==1 then label,value="SIMULATOR MODE","MODE "..controlMode
    elseif i==2 then label,value="RADIO ACTUAL MODE","MODE "..radioMode..(radioDetected and " (AUTO)" or " (SET TO MATCH RADIO)")
    elseif i<=6 then label,value=labels[i-2],CFG[keys[i-2]]==1 and "NORMAL" or "REVERSE"
    elseif i==7 then label,value="SOUND",soundEnabled and "ON" or "OFF"
    else label,value="BACK","Return to aircraft selection" end
    text(35,y,label);text(325,y,value,C.yellow)
  end
  text(26,381,"Mode 1: left ELE/RUD, right COL/AIL",C.muted)
  text(26,408,"Mode 2: left COL/RUD, right ELE/AIL",C.muted)
  text(26,442,"Changes affect this simulator only; retained until script exit.",C.muted)
end
local function init()
  local w,h=LCD_W or 800,LCD_H or 480
  scale=math.min(w/800,h/480); ox,oy=(w-800*scale)/2,(h-480*scale)/2
  local rgb={bg={13,23,32},panel={18,31,42},active={44,64,70},
    sky={67,103,127},ground={35,58,52},grid={58,80,72},pad={76,95,83},white={234,244,241},
    muted={148,172,169},dark={15,22,30},glass={24,43,58},belly={76,82,88},
    shadow={17,34,28},rotor={109,145,133},metal={181,193,199},yellow={255,204,66}}
  for k,v in pairs(rgb) do C[k]=lcd.RGB(v[1],v[2],v[3]) end
  for _,m in ipairs(models) do
    m.color=lcd.RGB(m.rgb[1],m.rgb[2],m.rgb[3])
    m.shadeColor=lcd.RGB(m.shade[1],m.shade[2],m.shade[3])
  end
  local detect=(etx and etx.getStickMode) or getStickMode
  if type(detect)=="function" then
    local good,value=pcall(detect)
    if good and (value==1 or value==2) then radioMode=value;controlMode=value;radioDetected=true end
  end
  reset(); lastTime=getTime()
end
local function centered()
  return abs(u.a)<0.12 and abs(u.e)<0.12 and abs(u.r)<0.12
end
local function rotorSound(now)
  if mode~="flying" or not soundEnabled or type(playTone)~="function" then return end
  if now<lastTone or now-lastTone>=12 then
    -- Short background pulses, no long queue and no blocking waits.
    local frequency=round(175+abs(u.c)*85+math.min(25,(abs(u.a)+abs(u.e))*15))
    playTone(frequency,80,0,PLAY_BACKGROUND or 0,0)
    lastTone=now
  end
end
local function run(event)
  readInputs()
  local now=getTime(); local elapsed=(now-lastTime)/100; lastTime=now
  -- Discard excess elapsed time after a slow frame; never auto-pause.
  -- The capped timestep prevents a jump without demanding stick matching.
  local dt=clamp(elapsed,0,0.1)
  if suppressBreak and now-suppressAt>120 then suppressBreak=false end
  if mode=="settings" then
    if eventIs(event,EVT_ROT_LEFT) or eventIs(event,EVT_MINUS_FIRST) then settingsRow=(settingsRow+6)%8+1 end
    if eventIs(event,EVT_ROT_RIGHT) or eventIs(event,EVT_PLUS_FIRST) then settingsRow=settingsRow%8+1 end
    if eventIs(event,EVT_EXIT_BREAK) then mode="menu"
    elseif eventIs(event,EVT_ENTER_BREAK) then
      if suppressBreak then suppressBreak=false
      elseif settingsRow==1 then controlMode=3-controlMode
      elseif settingsRow==2 then radioMode=3-radioMode;radioDetected=false
      elseif settingsRow<=6 then
        local key=({"revAil","revEle","revRud","revCol"})[settingsRow-2]
        CFG[key]=-CFG[key]
      elseif settingsRow==7 then soundEnabled=not soundEnabled
      else mode="menu" end
    end
    lcd.clear();settingsUI();text(716,461,"by PSP",C.muted);return 0
  end
  if mode=="menu" then
    if abs(u.a)<0.25 then speedLatch=false end
    if not speedLatch and abs(u.a)>.65 then
      speedIndex=clamp(speedIndex+(u.a>0 and 1 or -1),1,#speeds)
      speedLatch=true
    end
    if abs(u.r)<0.25 then latch=false end
    if not latch and abs(u.r)>0.65 then selected=u.r>0 and 2 or 1; latch=true end
    if eventIs(event,EVT_ROT_LEFT) or eventIs(event,EVT_MINUS_FIRST) then selected=1 end
    if eventIs(event,EVT_ROT_RIGHT) or eventIs(event,EVT_PLUS_FIRST) then selected=2 end
  end
  if eventIs(event,EVT_ENTER_LONG) then
    if mode=="menu" then mode="settings";settingsRow=1 else reset() end
    latch=true; suppressBreak=true; suppressAt=now
  elseif eventIs(event,EVT_ENTER_BREAK) then
    if suppressBreak then suppressBreak=false
    elseif mode=="menu" then
      if centered() and abs(u.c)<0.12 then mode,hint="flying",""
      else hint="Center ALL sticks first (collective = zero pitch)." end
    elseif mode=="flying" then mode,pauseCol="paused",u.c
    elseif mode=="paused" and centered() and abs(u.c-pauseCol)<0.12 then mode="flying" end
  elseif eventIs(event,EVT_EXIT_BREAK) then
    if mode=="flying" then mode,pauseCol="paused",u.c else return 2 end
  end
  if mode=="flying" then
    local n=math.max(1,math.ceil(dt/0.01))
    for i=1,n do if mode=="flying" then physics(dt/n) end end
    rotor=(rotor+dt*37)%(2*pi)
  elseif mode=="menu" then rotor=(rotor+dt*2)%(2*pi) end
  rotorSound(now)
  -- Fixed ground pilot: pan/tilt only, distance controls apparent size.
  updateView(); lcd.clear()
  if mode=="settings" then settingsUI() else ground(); helicopter(mode=="menu"); ui() end
  text(716,461,"by PSP",C.muted)
  return 0
end
return {name="Heli 3D 700",init=init,run=run}




