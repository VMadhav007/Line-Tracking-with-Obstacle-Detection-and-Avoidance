import math

a = 2.0
n_segments = 40

segments = []
for i in range(n_segments):
    t1 = 2 * math.pi * i / n_segments
    t2 = 2 * math.pi * (i + 1) / n_segments
    
    s1 = math.sin(t1)
    c1 = math.cos(t1)
    x1 = a * c1 / (1 + s1*s1)
    y1 = a * s1 * c1 / (1 + s1*s1)
    
    s2 = math.sin(t2)
    c2 = math.cos(t2)
    x2 = a * c2 / (1 + s2*s2)
    y2 = a * s2 * c2 / (1 + s2*s2)
    
    mx = (x1 + x2) / 2
    my = (y1 + y2) / 2
    
    dx = x2 - x1
    dy = y2 - y1
    length = math.sqrt(dx*dx + dy*dy)
    angle = math.atan2(dy, dx)
    length *= 1.15
    
    segments.append((i, mx, my, length, angle))

for i, mx, my, length, angle in segments:
    print(f'      <visual name="seg_{i}">')
    print(f'        <pose>{mx:.4f} {my:.4f} 0.001 0 0 {angle:.4f}</pose>')
    print(f'        <geometry><box><size>{length:.4f} 0.15 0.002</size></box></geometry>')
    print(f'        <material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse></material>')
    print(f'      </visual>')
