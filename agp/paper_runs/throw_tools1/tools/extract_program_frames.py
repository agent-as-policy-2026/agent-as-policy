# tool: extract_program_frames.py
# category: process
# purpose: Extract camera frames aligned to a buffered program and make labeled contact sheets.
# usage: python3 knowledge/tools/extract_program_frames.py REPORT CAMERA OUTPUT_PREFIX --start SECONDS --end SECONDS [--stride N] (run from the session directory)
# inputs/outputs: Reads report JSON, bridge_rec camera video and frame CSV; writes PNG frames, timing JSON and contact sheets.
# assumptions: Camera CSV frame indices match video frames; shared monotonic clock; requires ffmpeg and Pillow.
# verified: used successfully in the session that wrote it
import argparse,csv,json,subprocess,pathlib
from PIL import Image,ImageDraw
p=argparse.ArgumentParser();p.add_argument('report');p.add_argument('camera',choices=['wrist','top']);p.add_argument('prefix');p.add_argument('--start',type=float,default=0);p.add_argument('--end',type=float,required=True);p.add_argument('--stride',type=int,default=1);a=p.parse_args()
d=json.load(open(a.report));t0=d['trace'][0]['dispatch_monotonic_s']
rows=[(int(r['frame_index']),int(r['monotonic_ns'])/1e9-t0) for r in csv.DictReader(open('bridge_rec/'+a.camera+'_frames.csv'))]; rows=[r for r in rows if a.start<=r[1]<=a.end][::a.stride]
if not rows: raise SystemExit('No aligned frames')
pathlib.Path(a.prefix).parent.mkdir(parents=True,exist_ok=True)
expr='+'.join('eq(n\\,%d)'%r[0] for r in rows)
subprocess.run(['ffmpeg','-v','error','-i','bridge_rec/'+a.camera+'.mp4','-vf','select='+expr,'-vsync','0',a.prefix+'_%03d.png'],check=True)
json.dump([{'index':i,'relative_s':t,'file':a.prefix+'_%03d.png'%(j+1)} for j,(i,t) in enumerate(rows)],open(a.prefix+'_timing.json','w'),indent=2)
for offset in range(0,len(rows),20):
 subset=rows[offset:offset+20]; sheet=Image.new('RGB',(1280,204*((len(subset)+3)//4)),'white');draw=ImageDraw.Draw(sheet)
 for k,(i,t) in enumerate(subset):
  im=Image.open(a.prefix+'_%03d.png'%(offset+k+1));im.thumbnail((320,180));x=k%4*320;y=k//4*204;sheet.paste(im,(x,y));draw.text((x+4,y+181),f'frame {i} t={t:.3f}s',fill='black')
 sheet.save(a.prefix+'_sheet_%02d.png'%(offset//20))
print(json.dumps({'count':len(rows),'first':rows[0],'last':rows[-1],'prefix':a.prefix}))
