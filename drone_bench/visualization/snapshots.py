"""Export static 3D trajectory snapshots as standalone SVG artifacts."""
import math
from .mesh import mesh


def snapshot(payload,tube,frame):
    width,height=1000,760; yaw=-.7;pitch=.65;scale=1.18
    def project(p):
        x,y,z=p[0]-250,p[1]-250,p[2]-130
        a=math.cos(yaw)*x-math.sin(yaw)*y;b=math.sin(yaw)*x+math.cos(yaw)*y
        return [width/2+a*scale,height/2-(math.sin(pitch)*b+math.cos(pitch)*z)*scale,math.cos(pitch)*b-math.sin(pitch)*z]
    parts=[f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}"><rect width="100%" height="100%" fill="#f9fbfc"/>',
           f'<text x="35" y="38" font-size="22" font-family="sans-serif">27-UAV swarm · t = {frame["time"]:.1f} s</text>']
    def line(points,color,width=1,closed=False):
        coords=' '.join(f'{project(p)[0]:.2f},{project(p)[1]:.2f}' for p in points)
        tag='polygon' if closed else 'polyline'
        parts.append(f'<{tag} points="{coords}" fill="none" stroke="{color}" stroke-width="{width}"/>')
    for x in range(0,501,50):
        line([[x,0,0],[x,500,0]],'#dbe3ea');line([[0,x,0],[500,x,0]],'#dbe3ea')
    for o in payload['scenario']['obstacles']:
        if o['type']=='box':
            a,b=o['min'],o['max'];v=[[a[0],a[1],a[2]],[b[0],a[1],a[2]],[b[0],b[1],a[2]],[a[0],b[1],a[2]],[a[0],a[1],b[2]],[b[0],a[1],b[2]],[b[0],b[1],b[2]],[a[0],b[1],b[2]]]
            faces=[[0,1,2,3],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]
            faces.sort(key=lambda f:-sum(project(v[i])[2] for i in f))
            for f in faces:
                coords=' '.join(f'{project(v[i])[0]:.2f},{project(v[i])[1]:.2f}' for i in f)
                parts.append(f'<polygon points="{coords}" fill="#d8a37b" fill-opacity=".3" stroke="#b98b68"/>')
        else:
            q=project(o['center']);parts.append(f'<circle cx="{q[0]}" cy="{q[1]}" r="{o["radius"]*scale}" fill="#d8a37b" fill-opacity=".35" stroke="#b98b68"/>')
    rings=mesh(tube)
    for i in range(0,len(rings),15): line(rings[i],'#91c8be',.7,True)
    for j in range(0,24,6):line([ring[j] for ring in rings],'#91c8be',.6)
    line(tube.points,'#138a80',2)
    for i in frame['active']:
        q=project(frame['positions'][i]);r=payload['scenario']['safety_radius']*scale
        parts.append(f'<circle cx="{q[0]}" cy="{q[1]}" r="{r}" fill="#e0a530" fill-opacity=".7" stroke="#906118"/>')
    for p,label in [(payload['scenario']['start'],'START'),(payload['scenario']['goal'],'GOAL')]:
        q=project(p);parts.append(f'<text x="{q[0]+8}" y="{q[1]-12}" font-size="14" font-family="sans-serif">{label}</text>')
    parts.append(f'<text x="35" y="{height-25}" font-size="14" font-family="sans-serif">Active: {len(frame["active"])} / 27 · Metres · Custom benchmark world</text></svg>')
    return ''.join(parts)
