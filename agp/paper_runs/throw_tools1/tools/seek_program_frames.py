# tool: seek_program_frames.py
# category: process
# purpose: Extract time-aligned recording frames using indexed video seeking and write contact sheets.
# usage: python3 knowledge/tools/seek_program_frames.py REPORT CAMERA PREFIX START END    (run from the session directory)
# inputs/outputs: Reads program report and bridge_rec video/CSV; writes PNG frames, timing JSON, and contact sheets.
# assumptions: OpenCV and Pillow installed; CSV frame indices match video indices; shared monotonic clocks; seeking must return the requested index.
# verified: used successfully in the session that wrote it
import sys,json,csv,cv2,pathlib
from PIL import Image,ImageDraw
report,cam,prefix,start,end=sys.argv[1:];d=json.load(open(report));t0=d['trace'][0]['dispatch_monotonic_s'];rows=[(int(r['frame_index']),int(r['monotonic_ns'])/1e9-t0) for r in csv.DictReader(open('bridge_rec/'+cam+'_frames.csv'))];rows=[(i,t) for i,t in rows if float(start)<=t<=float(end)];cap=cv2.VideoCapture('bridge_rec/'+cam+'.mp4');out=[];pathlib.Path(prefix).parent.mkdir(parents=True,exist_ok=True)
for j,(i,t) in enumerate(rows):
 cap.set(cv2.CAP_PROP_POS_FRAMES,i);ok,im=cap.read()
 if not ok or abs(cap.get(cv2.CAP_PROP_POS_FRAMES)-(i+1))>.1:raise RuntimeError('frame seek failed '+str(i))
 f=prefix+'_%03d.png'%(j+1);cv2.imwrite(f,im);out.append({'index':i,'relative_s':t,'file':f})
cap.release();json.dump(out,open(prefix+'_timing.json','w'),indent=2)
for off in range(0,len(out),20):
 sub=out[off:off+20];sheet=Image.new('RGB',(1280,204*((len(sub)+3)//4)),'white');dr=ImageDraw.Draw(sheet)
 for j,row in enumerate(sub):
  im=Image.open(row['file']);im.thumbnail((320,180));x=j%4*320;y=j//4*204;sheet.paste(im,(x,y));dr.text((x+3,y+181),f"frame {row['index']} t={row['relative_s']:.3f}s",fill='black')
 sheet.save(prefix+'_sheet_%02d.png'%(off//20))
print(json.dumps({'count':len(out),'first':out[0],'last':out[-1]}))
