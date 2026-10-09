from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path('/opt/dave_ws/src/bygd_rov')
fig,axs=plt.subplots(1,2,figsize=(12,7))
for folder in ['Category']:
 for f in sorted((p/'meshes/obn_scene'/folder).glob('*.obj')):
  v=np.array([list(map(float,l.split()[1:4])) for l in f.open() if l.startswith('v ')])
  print(f.stem,len(v),'bounds',v.min(0),v.max(0))
  if 'seabed' in f.stem: continue
  stride=max(1,len(v)//6000)
  axs[0].scatter(v[::stride,0],v[::stride,1],s=1,label=f.stem)
  if 'particles' in f.stem: axs[1].scatter(v[::stride,0],v[::stride,1],c=v[::stride,2],s=3,vmin=0,vmax=9)
for ax in axs: ax.set_aspect('equal'); ax.grid(alpha=.2)
axs[0].legend(markerscale=5,fontsize=7); axs[1].set_title('Suspended particles: color=local Z')
fig.savefig(p/'validation/obn/scene_overview.png',dpi=130)
