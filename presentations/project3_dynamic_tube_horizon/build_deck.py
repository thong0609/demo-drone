"""Edit a local PowerPoint copy using standard-library OOXML and Pillow.

Preserves the source package's HUST templates and first title slide. All explanatory
text is editable; displayed math is rasterized at high resolution and repeated
as plain notation in slide notes and the content manifest.
"""
from pathlib import Path
from copy import deepcopy
import zipfile,xml.etree.ElementTree as ET,json,math,re
from PIL import Image,ImageDraw,ImageFont

ROOT=Path(__file__).resolve().parent
SOURCE=Path('/home/tts/Downloads/Project-III-bao-cao-tien-do-outer-loop-4-trang.pptx')
OUTPUT=ROOT/'Project-III-Tube-dong-Horizon-bao-cao.pptx'
A='http://schemas.openxmlformats.org/drawingml/2006/main';P='http://schemas.openxmlformats.org/presentationml/2006/main';R='http://schemas.openxmlformats.org/officeDocument/2006/relationships';REL='http://schemas.openxmlformats.org/package/2006/relationships'
for prefix,uri in [('a',A),('p',P),('r',R)]:ET.register_namespace(prefix,uri)
def tag(ns,name):return '{'+ns+'}'+name
def sub(parent,ns,local_name,**attrs):return ET.SubElement(parent,tag(ns,local_name),{k:str(v) for k,v in attrs.items()})
def xml(root):return ET.tostring(root,encoding='utf-8',xml_declaration=True)
EMU=914400;SCALE=140;W=13.333333;H=7.5
REG='/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf';BOLD='/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf';MATH='/usr/share/fonts/truetype/dejavu/DejaVuSerif.ttf'
RED='BC1F32';INK='25364A';GRAY='526175';LIGHT='F3F5F8';GOLD='D99B00'
files={}
with zipfile.ZipFile(SOURCE) as source:files={n:source.read(n) for n in source.namelist()}
background=Image.open(__import__('io').BytesIO(files['ppt/media/image3.png'])).convert('RGB').resize((round(W*SCALE),round(H*SCALE)))

# A small, deterministic math layout engine supporting scripts and fractions.
def math_text(value,size):
    font=ImageFont.truetype(MATH,size);ascent,descent=font.getmetrics();width=max(1,math.ceil(font.getlength(value)))
    im=Image.new('RGBA',(width+4,ascent+descent+6),(255,255,255,0));ImageDraw.Draw(im).text((2,ascent+2),value,font=font,fill='#172B40',anchor='ls')
    return im,ascent+2

def join(parts):
    if not parts:return math_text('',50)
    baseline=max(b for _,b in parts);bottom=max(im.height-b for im,b in parts)
    canvas=Image.new('RGBA',(sum(im.width for im,_ in parts),baseline+bottom),(255,255,255,0));x=0
    for im,b in parts:canvas.alpha_composite(im,(x,baseline-b));x+=im.width
    return canvas,baseline

class MathParser:
    def __init__(self,text,size=58):self.text=text;self.i=0;self.size=size
    def group(self,size):
        if self.i<len(self.text) and self.text[self.i]=='{':
            self.i+=1;result=self.parse(size,True);return result
        char=self.text[self.i];self.i+=1;return math_text(char,size)
    def parse(self,size=None,stop=False):
        size=size or self.size;parts=[]
        while self.i<len(self.text):
            char=self.text[self.i]
            if char=='}' and stop:self.i+=1;break
            if self.text.startswith('\\frac',self.i):
                self.i+=5;n,nb=self.group(round(size*.85));d,db=self.group(round(size*.85))
                width=max(n.width,d.width)+12;im=Image.new('RGBA',(width,n.height+d.height+12),(255,255,255,0));im.alpha_composite(n,((width-n.width)//2,0));ImageDraw.Draw(im).line((2,n.height+4,width-2,n.height+4),fill='#172B40',width=2);im.alpha_composite(d,((width-d.width)//2,n.height+10));base=n.height+10+db-size//3
            elif char=='{':self.i+=1;im,base=self.parse(size,True)
            else:self.i+=1;im,base=math_text(char,size)
            superscript=None;subscript=None
            while self.i<len(self.text) and self.text[self.i] in '^_':
                symbol=self.text[self.i];self.i+=1;script=self.group(max(18,round(size*.6)))
                if symbol=='^':superscript=script
                else:subscript=script
            if superscript or subscript:
                extra=max((v[0].width for v in (superscript,subscript) if v),default=0)
                top=max(0,(superscript[0].height+round(size*.1)-base*.55) if superscript else 0);top=math.ceil(top)
                height=top+im.height+(round(size*.42) if subscript else 0)
                canvas=Image.new('RGBA',(im.width+extra,height),(255,255,255,0));canvas.alpha_composite(im,(0,top))
                if superscript:canvas.alpha_composite(superscript[0],(im.width,max(0,round(top+base*.55-superscript[0].height))))
                if subscript:canvas.alpha_composite(subscript[0],(im.width,round(top+base-size*.05)))
                im,base=canvas,base+top
            parts.append((im,base))
        return join(parts)

math_files={};manifest=[];previews=[];layout_warnings=[]

class Slide:
    def __init__(self,title,subtitle,footer,source,number):
        self.number=number;self.id=1;self.notes=[];self.maths=[];self.texts=[]
        self.root=ET.Element(tag(P,'sld'),{'showMasterSp':'0'});cs=sub(self.root,P,'cSld');bg=sub(cs,P,'bg');bp=sub(bg,P,'bgPr');fill=sub(bp,A,'blipFill');sub(fill,A,'blip',**{tag(R,'embed'):'rId2'});st=sub(fill,A,'stretch');sub(st,A,'fillRect')
        self.tree=sub(cs,P,'spTree');nv=sub(self.tree,P,'nvGrpSpPr');sub(nv,P,'cNvPr',id=1,name='');sub(nv,P,'cNvGrpSpPr');sub(nv,P,'nvPr');gp=sub(self.tree,P,'grpSpPr');xf=sub(gp,A,'xfrm');sub(xf,A,'off',x=0,y=0);sub(xf,A,'ext',cx=0,cy=0);sub(xf,A,'chOff',x=0,y=0);sub(xf,A,'chExt',cx=0,cy=0)
        co=sub(self.root,P,'clrMapOvr');sub(co,A,'masterClrMapping')
        self.rels=ET.Element(tag(REL,'Relationships'));self.relate('rId1','slideLayout','../slideLayouts/slideLayout12.xml');self.relate('rId2','image','../media/image3.png')
        self.image=background.copy();self.draw=ImageDraw.Draw(self.image)
        self.text(.37,.10,12.55,.48,title,25,True,'FFFFFF')
        self.text(.58,.95,12.15,.55,subtitle,20,True)
        self.rect(.55,5.77,12.22,.84,'F3F5F8')
        self.text(.71,5.86,11.85,.66,footer,12.2,False,GRAY)
        self.text(4.12,6.87,8.4,.22,source,9.4,False,GRAY)
        self.text(12.45,7.11,.45,.22,str(number),11,False,INK)
    def relate(self,id,kind,target):sub(self.rels,REL,'Relationship',Id=id,Type=R+'/'+kind,Target=target)
    def shape(self,name,x,y,w,h):
        self.id+=1;s=sub(self.tree,P,'sp');nv=sub(s,P,'nvSpPr');sub(nv,P,'cNvPr',id=self.id,name=name);sub(nv,P,'cNvSpPr',txBox='1');sub(nv,P,'nvPr');pr=sub(s,P,'spPr');xf=sub(pr,A,'xfrm');sub(xf,A,'off',x=round(x*EMU),y=round(y*EMU));sub(xf,A,'ext',cx=round(w*EMU),cy=round(h*EMU));geo=sub(pr,A,'prstGeom',prst='rect');sub(geo,A,'avLst');return s,pr
    def rect(self,x,y,w,h,color,border=None):
        s,pr=self.shape('Panel',x,y,w,h);f=sub(pr,A,'solidFill');sub(f,A,'srgbClr',val=color);ln=sub(pr,A,'ln',w=10000)
        if border:f=sub(ln,A,'solidFill');sub(f,A,'srgbClr',val=border)
        else:sub(ln,A,'noFill')
        self.draw.rectangle((x*SCALE,y*SCALE,(x+w)*SCALE,(y+h)*SCALE),fill='#'+color,outline='#'+(border or color),width=1)
    def text(self,x,y,w,h,text,size=22,bold=False,color=INK,align='l'):
        text=text.replace('ₜ₊₁','_(t+1)').replace('ₜ₋₁','_(t−1)').replace('ₜ','_t')
        # Pre-wrap to keep Office layout deterministic and match the preview.
        font=ImageFont.truetype(BOLD if bold else REG,round(size*SCALE/72))
        lines=[]
        for paragraph in text.split('\n'):
            line=''
            for word in paragraph.split(' '):
                candidate=(line+' '+word).strip()
                if font.getlength(candidate)>w*SCALE and line:lines.append(line);line=word
                else:line=candidate
            lines.append(line)
        line_h=size*SCALE/72*1.15
        if len(lines)*line_h>h*SCALE+8:layout_warnings.append((self.number,text[:65],len(lines)*line_h/SCALE,h))
        s,pr=self.shape('Text',x,y,w,h);sub(pr,A,'noFill');ln=sub(pr,A,'ln');sub(ln,A,'noFill');body=sub(s,P,'txBody');bp=sub(body,A,'bodyPr',wrap='none',lIns=0,rIns=0,tIns=0,bIns=0);sub(bp,A,'noAutofit');sub(body,A,'lstStyle')
        for j,line in enumerate(lines):
            p=sub(body,A,'p');pp=sub(p,A,'pPr',algn=align);sp=sub(pp,A,'lnSpc');sub(sp,A,'spcPct',val=115000);sub(pp,A,'buNone');r=sub(p,A,'r');rp=sub(r,A,'rPr',lang='vi-VN',sz=round(size*100),b='1' if bold else '0');f=sub(rp,A,'solidFill');sub(f,A,'srgbClr',val=color);sub(rp,A,'latin',typeface='Arial');sub(rp,A,'ea',typeface='Arial');sub(rp,A,'cs',typeface='Arial');sub(r,A,'t').text=line
            xx=x*SCALE
            if align=='ctr':xx+=(w*SCALE-font.getlength(line))/2
            self.draw.text((xx,y*SCALE+j*line_h),line,font=font,fill='#'+color,anchor='lt')
        self.texts.append(text)
    def formula(self,x,y,w,h,expression):
        im,base=MathParser(expression).parse();bounds=im.getbbox();im=im.crop(bounds)
        # White paper backgrounds in formula panels, keeping a clean original style.
        raw=Image.new('RGB',im.size,'white');raw.paste(im,(0,0),im)
        key=f'mpc_math_{self.number}_{len(self.maths)}.png';buffer=__import__('io').BytesIO();raw.save(buffer,format='PNG');math_files['ppt/media/'+key]=buffer.getvalue()
        ratio=min(w/raw.width,h/raw.height);ww=raw.width*ratio;hh=raw.height*ratio;xx=x+(w-ww)/2;yy=y+(h-hh)/2
        self.id+=1;pic=sub(self.tree,P,'pic');nv=sub(pic,P,'nvPicPr');sub(nv,P,'cNvPr',id=self.id,name='Math '+str(len(self.maths)+1),descr=expression);sub(nv,P,'cNvPicPr');sub(nv,P,'nvPr');bf=sub(pic,P,'blipFill');rid=f'rId{len(self.rels)+1}';self.relate(rid,'image','../media/'+key);sub(bf,A,'blip',**{tag(R,'embed'):rid});st=sub(bf,A,'stretch');sub(st,A,'fillRect');pr=sub(pic,P,'spPr');xf=sub(pr,A,'xfrm');sub(xf,A,'off',x=round(xx*EMU),y=round(yy*EMU));sub(xf,A,'ext',cx=round(ww*EMU),cy=round(hh*EMU));geo=sub(pr,A,'prstGeom',prst='rect');sub(geo,A,'avLst')
        self.image.paste(raw.resize((max(1,round(ww*SCALE)),max(1,round(hh*SCALE)))),(round(xx*SCALE),round(yy*SCALE)))
        self.maths.append(expression)
    def card(self,x,y,w,h,heading,body):
        self.rect(x,y,w,h,LIGHT,'E2E6EC');self.rect(x,y,.055,h,RED)
        if h<1.7:
            self.text(x+.2,y+.1,w-.4,.35,heading,20,True,RED)
            self.text(x+.2,y+.55,w-.4,h-.65,body,18)
        else:
            self.text(x+.2,y+.16,w-.4,.5,heading,22,True,RED)
            self.text(x+.2,y+.83,w-.4,h-.96,body,20)
    def save(self,index):
        notes_index=index-2;rid=f'rId{len(self.rels)+1}';self.relate(rid,'notesSlide',f'../notesSlides/notesSlide{notes_index}.xml')
        files[f'ppt/slides/slide{index}.xml']=xml(self.root);files[f'ppt/slides/_rels/slide{index}.xml.rels']=xml(self.rels)
        notes=ET.Element(tag(P,'notes'));cs=sub(notes,P,'cSld');tree=sub(cs,P,'spTree');nv=sub(tree,P,'nvGrpSpPr');sub(nv,P,'cNvPr',id=1,name='');sub(nv,P,'cNvGrpSpPr');sub(nv,P,'nvPr');sub(tree,P,'grpSpPr');s=sub(tree,P,'sp');nv=sub(s,P,'nvSpPr');sub(nv,P,'cNvPr',id=2,name='Notes');sub(nv,P,'cNvSpPr');np=sub(nv,P,'nvPr');sub(np,P,'ph',type='body',idx=1);sub(s,P,'spPr');tb=sub(s,P,'txBody');sub(tb,A,'bodyPr');sub(tb,A,'lstStyle')
        note_text='\n'.join(self.texts+self.maths+self.notes+['Phân biệt nội dung bài gốc, đề xuất mới và kết quả mô phỏng riêng; chưa tuyên bố an toàn toàn hệ thống.','Nguồn nền tảng: Yuan et al., Curve Virtual Tube Planning and Distributed Control for Swarm UAVs With Complex Constraints and Disturbances, IEEE TIE 2026, DOI 10.1109/TIE.2025.3603043. Chi tiết: docs/dynamic_tube/PLANNER.md.'])
        for line in note_text.split('\n'):p=sub(tb,A,'p');r=sub(p,A,'r');sub(r,A,'t').text=line
        co=sub(notes,P,'clrMapOvr');sub(co,A,'masterClrMapping');files[f'ppt/notesSlides/notesSlide{notes_index}.xml']=xml(notes)
        rel=ET.Element(tag(REL,'Relationships'));sub(rel,REL,'Relationship',Id='rId1',Type=R+'/notesMaster',Target='../notesMasters/notesMaster1.xml');sub(rel,REL,'Relationship',Id='rId2',Type=R+'/slide',Target=f'../slides/slide{index}.xml');files[f'ppt/notesSlides/_rels/notesSlide{notes_index}.xml.rels']=xml(rel)
        previews.append(self.image);(ROOT/'preview').mkdir(exist_ok=True);self.image.save(ROOT/'preview'/f'slide_{index:02}.png')
        manifest.append(dict(slide=index,title=self.texts[0],texts=self.texts,equations=self.maths,notes=self.notes))


SOURCE_FOOT='Yuan et al. • IEEE TIE 2026 | Thí nghiệm và đề xuất riêng: PLANNER.md'
def make(title,subtitle,legend,num):
    return Slide(title,subtitle,legend,SOURCE_FOOT,num)

def bullets(slide,x,y,w,items,size=21,gap=.76):
    for j,text in enumerate(items):slide.text(x,y+j*gap,w,gap-.10,'• '+text,size)

def finish(slide,index,notes=''):
    if notes:slide.notes.append(notes)
    slide.save(index)

s=make('TỪ TRÁNH CỤC BỘ ĐẾN ĐIỀU CHỈNH TUBE','Câu hỏi: nếu một vật cản động lớn bay vào giữa đàn UAV?',
       'Các hạng “hút/đẩy” ở đây đóng góp vào vận tốc outer, không phải lực vật lý trực tiếp.\nThêm hạng tránh vật cản là ý tưởng ban đầu; chưa triển khai hoặc kiểm chứng trong thí nghiệm này.',1)
s.card(.65,1.75,3.85,2.45,'OUTER HIỆN CÓ','Hướng tới đích\nTránh UAV khác\nGiữ đàn\nTránh thành tube')
s.card(4.75,1.75,3.85,2.45,'Ý TƯỞNG BAN ĐẦU','Thêm vector tránh vật cản động vào lệnh vận tốc outer.')
s.card(8.85,1.75,3.85,2.45,'ĐIỂM CẦN CÂN NHẮC','Vật lớn có thể làm UAV né khác phía; xung đột với giữ đàn và biên tube.')
s.formula(.9,4.55,11.4,.63,'v_{c,i}=sat_{v_m}(v_g+v_n+v_s+v_t+v_o)')
finish(s,3,'Biểu thức là sơ đồ ý tưởng theo các vector đóng góp, không chép nguyên dấu của Eq.34. Vật cản lớn có thể gây tách đàn là nhận định thiết kế cần kiểm chứng, không phải kết quả đã đo hoặc kết luận tác giả.')

s=make('VÌ SAO CHUYỂN SANG NGHIÊN CỨU TUBE ĐỘNG?','Điều chỉnh miền bay chung để cả đàn có hướng tránh thống nhất.',
       'Tube tròn mô tả bởi đường tâm p_b(s) và bán kính r_b(s); s là tham số dọc đường.\nĐề xuất tube động thuộc nghiên cứu mở rộng; chưa tự kế thừa bảo đảm của controller trên tube đã cho.',2)
s.card(.7,1.75,5.8,2.6,'ĐỘNG LỰC NGHIÊN CỨU','Cập nhật đường tâm/bán kính khi dự báo vật cản sẽ xâm nhập tube.\nGiảm phụ thuộc vào việc từng UAV tự né một phía.')
s.card(6.8,1.75,5.8,2.6,'HƯỚNG MỞ CỦA BÀI BÁO','Section VI nêu sinh tube realtime trong môi trường động và nâng cao điều khiển phân tán là công việc tương lai.')
s.text(.9,4.7,11.4,.7,'Phạm vi hôm nay: thử planner trước; giữ phần điều khiển, cảm biến và truyền thông để sau.',22,True)
finish(s,4,'Nguồn: Section VI, trang 3048 của PDF chính. Không nói tác giả đã chứng minh phương pháp thất bại với mọi vật cản động. A1 chất lượng planner nền được ghi nhận, không phải trọng tâm bắt buộc của nghiên cứu vùng quét.')

s=make('HƯỚNG ĐÃ CHỌN: NÉ VÙNG QUÉT TRONG HORIZON','Biến chuyển động dự báo thành một vật cản hình học tạm thời.',
       'H là khoảng thời gian nhìn trước, không phải đường đi. Vùng quét bao kích thước vật thể và chuyển động.\nTại mỗi update: dùng thông tin mới; giữ tube cũ nếu còn hợp lệ, nếu không thì lập lại.',3)
for x,title,body in [(.7,'1. DỰ BÁO','Vị trí, kích thước,\nvector vận tốc →\nvùng quét trong H'),(4.8,'2. LẬP TUBE','Vùng cấm mới →\nenhanced RRT* →\nminimum snap'),(8.9,'3. KIỂM TRA','Kiểm tra cả tube;\nnhận / giữ / từ chối\nrồi xuất kết quả')]:s.card(x,1.8,3.65,2.55,title,body)
s.text(.9,4.7,11.4,.7,'Bảo thủ: cấm cả vùng vật cản có thể đi qua, dù có thời điểm vùng đó đang trống.',22,True)
finish(s,5,'Baseline hiện tại lập lại toàn tuyến với start/goal cố định; không có scheduler có trễ hay chuyển tiếp cho đàn đang bay. Dùng RRT* cải tiến gồm mixed cost và Visual, không đổi thành RRT thường.')

s=make('DỰ BÁO CHUYỂN ĐỘNG VÀ VÙNG QUÉT','Bản thử: vật cản cầu, bán kính cố định, vận tốc không đổi.',
       'τ: thời điểm nhận dữ liệu; h: thời gian dự báo; H: horizon. c_j,v_j,R_j là tâm, vận tốc, bán kính.\nm là đệm dự báo thực nghiệm; B(c,R) là khối cầu tâm c bán kính R. Không phải robust bound đã chứng minh.',4)
s.formula(.8,1.7,11.7,.75,'c_j(τ+h)=c_j(τ)+v_j h,    0≤h≤H')
s.formula(.8,2.95,11.7,.85,'S_j(H)=∪_{0≤h≤H} B(c_j(τ)+v_j h,R_j+m)')
s.text(.95,4.25,11.2,.8,'Chuyển động thẳng tạo capsule: một đoạn từ c_j đến c_j + v_jH, được bao bởi bán kính R_j + m.',22,True)
finish(s,6,'Nguồn lý thuyết: docs/dynamic_tube/PLANNER.md, phần 1, 5 và 14. Cầu bao cho vật thể bất kỳ là ý tưởng có thể mở rộng; chưa có perception hoặc backend cầu bao mesh/point cloud. Bao hình dạng không tự giải quyết dự báo chuyển động.')

s=make('KHOẢNG CÁCH TỚI VÙNG QUÉT CAPSULE','Từ một điểm p, tìm điểm gần nhất q trên đoạn dự báo.',
       'a,b: hai đầu đoạn dự báo; α: vị trí chuẩn hóa trên đoạn; clip giới hạn α vào [0,1].\nd_j(p) dương ở ngoài capsule. Nếu a=b, đặt q=a; d(p) lấy min với vật cản tĩnh và biên world.',5)
s.formula(.9,1.65,11.5,.55,'a=c_j(τ),    b=a+v_jH')
s.formula(.9,2.65,11.5,.85,r'α=clip_{[0,1]}(\frac{(p−a)^T(b−a)}{‖b−a‖^2}),    q=a+α(b−a)')
s.formula(.9,4.05,11.5,.65,'d_j(p)=‖p−q‖−(R_j+m)')
s.text(.95,5.05,11.2,.35,'Truy vấn giải tích: không cần dựng lại lưới SDF toàn cục để thử planner.',19,True)
finish(s,7,'Các truy vấn capsule là 1-Lipschitz. Min primitive distances là khoảng hở chính xác trong vùng trống; không gọi là signed Euclidean distance chính xác bên trong mọi hợp vật cản chồng nhau. Code: environment/swept.py.')

s=make('TUBE MỚI PHẢI TRÁNH CẢ THỂ TÍCH VẬT CẢN','Đường tâm không va chạm chưa đủ nếu tiết diện tube vẫn chạm capsule.',
       'r_b(s): bán kính tube; m_t: khoảng hở tube yêu cầu. q: tâm bao control points; E: bán kính bao.\nr_upper: cận trên bán kính từ control points. Điều kiện bao cầu là đủ và bảo thủ so với kiểm tra đĩa trực tiếp.',6)
s.formula(.9,1.75,11.5,.8,'d(p_b(s))−r_b(s) ≥ m_t')
s.text(.95,2.95,11.2,.45,'Chứng nhận liên tục bằng bao convex hull Bernstein và chia đôi đoạn:',21,True)
s.formula(.9,3.75,11.5,.75,'d(q)−E−r_{upper} ≥ m_t')
s.text(.95,4.95,11.2,.45,'Chưa đủ chứng nhận → chia nhỏ để kiểm tra; không đạt → từ chối candidate.',21,True)
finish(s,8,'Nguồn: PLANNER.md phần 5.3–5.5. Prediction margin nằm trong capsule; tube obstacle margin là lớp riêng. Chứng nhận hình học không chứng minh UAV thực bám tube, không chứng nhận injectivity hoặc mọi độ cong của tube.')

s=make('PLANNER NỀN: ENHANCED RRT* THEO BÀI BÁO','Kết hợp độ dài với penalty gần vật cản; Visual thử nối tới ancestor tốt hơn.',
       'a,b: hai nút; d_k: khoảng hở tại điểm lấy mẫu trên cạnh; λ_1,λ_2: trọng số.\nr_e: bán kính tube tối đa; κ>0 tránh mẫu số bằng 0. Penalty P(d)=0 khi d≥r_e.',7)
s.formula(.9,1.7,11.5,.75,'J(a,b)=λ_1‖b−a‖+λ_2∑_{k=1}^{n}P(d_k)')
s.formula(.9,2.95,11.5,.9,r'P(d)=\frac{(r_e−d)^3}{d^3+κ},    0≤d<r_e')
s.text(.95,4.4,11.2,.8,'Trong thử nghiệm động, các truy vấn khoảng hở và va chạm xét thêm capsule. Cơ chế mixed cost và Visual vẫn được giữ.',22,True)
finish(s,9,'Nguồn bài báo: Algorithms 2–3, Eq.15–16, trang 3040–3041. Kiểm tra cạnh có dự phòng Lipschitz để không bỏ khoảng giữa mẫu. Không tuyên bố bản tái hiện có đúng mọi tham số/solver của tác giả.')

s=make('MINIMUM SNAP: TẠO ĐƯỜNG TÂM VÀ BÁN KÍNH','Làm trơn waypoint, sau đó kiểm tra lại tube trong world có vùng quét.',
       'w: hệ số đa thức; σ_b: đường tham chiếu từ waypoint; γ: trọng số bám đường.\nRàng buộc: điều kiện đầu/cuối, nối liên tục tới đạo hàm bậc hai và corridor. Không tự bảo đảm tránh vật cản.',8)
s.formula(.8,1.75,11.7,.9,'min_w ∫ |σ^{(4)}(t)|^2 dt + γ ∫ |σ(t)−σ_b(t)|^2 dt')
s.formula(.9,3.15,11.5,.65,'A_{eq}w=b_{eq},    A_{ieq}w≤b_{ieq}')
s.text(.95,4.45,11.2,.9,'Đã thử cân bằng thời lượng đoạn để giảm overshoot. Vẫn còn vấn đề độ cong và tự chồng lấn; ghi nhận như hạn chế của planner nền.',22,True)
finish(s,10,'Nguồn bài báo: Eq.17–21. Repo dùng Bernstein degree7 và active-set QP, không phải interior-point như tác giả mô tả. Bán kính được suy từ clearance rồi co đồng đều để chứng nhận. Balanced durations sửa fixture 519.4→428.8 m nhưng max curvature lấy mẫu tăng 0.267→2.351 1/m; không gọi là cải thiện toàn diện. A1 không là đóng góp chính.')

s=make('ĐÃ THỬ VẬT CẢN ĐI THEO NHIỀU HƯỚNG','17 tình huống tổng hợp; lưu cả kết quả đạt và các lần thất bại.',
       'Một lượt đánh giá là một lần xét tại một snapshot, không phải một chuyến bay hay một lần gọi RRT*.\nSeed điều khiển lấy mẫu; các snapshot trong cùng chuỗi có liên quan, không phải các phép thử độc lập.',9)
s.card(.75,1.75,5.7,2.4,'HƯỚNG CHUYỂN ĐỘNG','Cắt ngang hai phía; dọc tuyến hai chiều; trên xuống, dưới lên; chéo; ba vật cản hội tụ.')
s.card(6.85,1.75,5.7,2.4,'THIẾT KẾ ĐỐI CHỨNG','3 seed × 2 snapshot × 6 phương án; tổng 612 lượt.\n306 replay, đủ từng trường hợp.')
s.text(.95,4.55,11.2,.75,'Bản cải tiến đạt chứng nhận 90/102 lượt; bản đầu 87/102. Đây là kết quả hình học trong bộ thử, chưa phải tỷ lệ đàn bay an toàn.',22,True)
finish(s,11,'Nguồn dữ liệu: artifacts/swept_volume_direction_evaluation/summary.json, protocol.json. Bộ hướng mới có8 ca; 6 ca theo trục cùng bán kính5m tốc độ20m/s, ca chéo21.35m/s, ca hội tụ nhiều kích thước/tốc độ. Không có held-out map độc lập. Các phương án: baseline, balanced_only, balanced_fixed_seed, improved, instantaneous, static_tube.')

s=make('A2: CHỈ THAY HORIZON, GIỮ CÁC ĐIỀU KIỆN KHÁC','15 tình huống × 3 seed × 6 giá trị H × 2 snapshot = 540 lượt.',
       'Đạt H: tube được chứng nhận trong khoảng đã chọn. “Tới update sau” kiểm tra thêm vùng quét 1 s.\nChiều dài TB chỉ tính khi có tube; mẫu số thành công khác nhau. Không kết luận từ trung bình có điều kiện.',10)
headers=['H (s)','Đạt H','Tới update sau','Độ dài TB (m)','p95 tính (ms)'];xs=[.85,2.55,4.55,7.35,10.35];ws=[1.5,1.9,2.7,2.9,2.1]
summary=json.loads((ROOT.parent.parent/'artifacts/swept_horizon_study/summary.json').read_text())
for j,(x,w) in enumerate(zip(xs,ws)):
 s.rect(x,1.72,w,.42,RED);s.text(x+.06,1.79,w-.12,.26,headers[j],15,True,'FFFFFF')
for i,r in enumerate(summary):
 y=2.17+i*.44;values=[f'{r["horizon"]:g}',f'{r["accepted"]}/90',f'{r["next_update_certified"]}/90',f'{r["mean_length_accepted"]:.2f}',f'{r["p95_seconds"]*1000:.0f}']
 for x,w,value in zip(xs,ws,values):s.rect(x,y,w,.42,'F3F5F8' if i%2==0 else 'FFFFFF');s.text(x+.07,y+.08,w-.14,.26,value,16)
s.text(.9,5.02,11.5,.44,'H=0.5 s chưa phủ khoảng cập nhật 1 s. Chưa chọn H tối ưu chung.',18,True)
finish(s,12,'Nguồn: artifacts/swept_horizon_study/summary.json và protocol.json. Planner cố định150 RRT iterations, H=.5/1/2/3/5/6, seed7/17/29, snapshot1/2s, prediction margin2m, world500x500x300. p95 đo riêng update, chưa gồm dựng snapshot, nhận thức, truyền thông. Không có UAV thực. H ngắn không đạt phép kiểm tra xa3s không đồng nghĩa sai yêu cầu. Có270 replay và53 tests toàn repo đạt.')

s=make('GIỚI HẠN VÀ PHẠM VI KẾT LUẬN','Đã đánh giá ý tưởng lập tube trong mô phỏng; chưa khẳng định triển khai realtime.',
       'Nguồn trạng thái là simulator; dự báo hiện khớp mô hình vận tốc không đổi. Thời gian phụ thuộc máy và tải.\nTube hợp lệ với snapshot lúc bắt đầu chưa tự được bảo đảm hợp lệ lúc dùng sau độ trễ tính toán.',11)
s.card(.75,1.75,5.7,2.75,'GIẢ THIẾT HIỆN TẠI','Biết tâm, bán kính, vận tốc.\nKhông có lỗi cảm biến/truyền thông.\nTube lập lại giữa hai đầu cố định.\nVùng quét có tính bảo thủ.')
s.card(6.85,1.75,5.7,2.75,'VẤN ĐỀ CẦN QUAN TÂM','Dữ liệu và dự báo có thể sai.\nVật cản di chuyển lúc đang tính.\nTube có thể cần kiểm tra lại khi dùng.\nChưa xét chuyển tiếp và bám của đàn.')
s.text(.95,4.95,11.2,.45,'A3: ghi thông số và hạn chế. Phần cứng mục tiêu và bay thử để sau.',18,True)
finish(s,13,'Nguồn: PLANNER.md phần15, ISSUES.md A1/A3/C. Không nói rằng phải bay thực mới đánh giá được planner; chỉ giới hạn tuyên bố đúng mô hình. Hard timeout và scheduler có trễ chưa triển khai. Cầu bao vật thể bất kỳ mới là đề xuất, chưa có module nhận thức/tính cầu bao.')

s=make('TUẦN SAU: THỬ HƯỚNG KHÔNG GIAN – THỜI GIAN','Không cấm mọi vị trí vật cản sẽ đi qua; xét đúng thời điểm sử dụng miền bay.',
       's: tham số dọc đường; τ: thời gian thực; Ω_occ: các cặp vị trí–thời gian được sử dụng.\nO_j(τ): vật cản tại τ. Biểu thức là điều kiện đủ minh họa cho tube, chưa phải formulation đã chốt.',12)
s.formula(.9,1.7,11.5,.8,'d(p_b(s),O_j(τ)) ≥ r_b(s)+m,    (s,τ)∈Ω_{occ}')
s.card(.8,2.95,5.65,2.25,'VIỆC CẦN THỬ','Chọn biến đường và thời gian.\nCho phép chờ/đổi lịch nếu phù hợp.\nTạo các ca horizon quá bảo thủ.')
s.card(6.85,2.95,5.65,2.25,'CÁCH ĐÁNH GIÁ','Cùng map, vật cản và seed.\nSo tỷ lệ đạt, đường, thời gian chờ.\nGhi cả thời gian tính và thất bại.')
finish(s,14,'Hướng B chưa bắt đầu. Cần xác định Ω_occ/lịch chiếm dụng của đàn trước khi dùng công thức; không mặc định một UAV đại diện đủ cho cả đàn. Chưa quyết định state/input, solver, mô hình động lực hay thời gian tối ưu. Đây là mục tiêu tuần sau do người dùng nêu, không phải kết quả đã đạt hoặc lịch bảo đảm.')

# Preserve the source's branded opener and identity, update only cover topic.
cover=ET.fromstring(files['ppt/slides/slide2.xml'])
for shape in cover.iter(tag(P,'sp')):
 body=shape.find(tag(P,'txBody'))
 if body is None:continue
 text=''.join(t.text or '' for t in body.iter(tag(A,'t')))
 if 'Enhanced RRT' in text or ('Tìm' in text and 'ống' in text):
  shape.remove(body);body=sub(shape,P,'txBody');bp=sub(body,A,'bodyPr',wrap='none',lIns=0,rIns=0,tIns=0,bIns=0);sub(bp,A,'noAutofit');sub(body,A,'lstStyle')
  for line in ['Tube động để tránh vật cản chuyển động','Thử nghiệm vùng quét trong horizon','Định hướng tiếp theo: không gian – thời gian']:
   p=sub(body,A,'p');pp=sub(p,A,'pPr');sp=sub(pp,A,'lnSpc');sub(sp,A,'spcPct',val=110000);r=sub(p,A,'r');rp=sub(r,A,'rPr',sz=2300,lang='vi-VN');f=sub(rp,A,'solidFill');sub(f,A,'srgbClr',val=INK);sub(rp,A,'latin',typeface='Arial');sub(r,A,'t').text=line
files['ppt/slides/slide2.xml']=xml(cover)
files.update(math_files)
# Clean old edit history, retain all branded templates and package relationships.
for name in list(files):
 if name.startswith('ppt/changesInfos/'):del files[name]
rels=ET.fromstring(files['ppt/_rels/presentation.xml.rels'])
for e in list(rels):
 if e.attrib.get('Type','').endswith('/changesInfo'):rels.remove(e)
files['ppt/_rels/presentation.xml.rels']=xml(rels)
ct=ET.fromstring(files['[Content_Types].xml'])
for e in list(ct):
 if 'changesInfo' in e.attrib.get('PartName',''):ct.remove(e)
if not any(e.attrib.get('Extension')=='png' for e in ct):sub(ct,'http://schemas.openxmlformats.org/package/2006/content-types','Default',Extension='png',ContentType='image/png')
files['[Content_Types].xml']=xml(ct)
# OPC readers expect default namespaces for package metadata.
ET.register_namespace('',REL)
for part in list(files):
 if part.endswith('.rels'):files[part]=xml(ET.fromstring(files[part]))
ET.register_namespace('','http://schemas.openxmlformats.org/package/2006/content-types')
files['[Content_Types].xml']=xml(ET.fromstring(files['[Content_Types].xml']))
with zipfile.ZipFile(OUTPUT,'w',zipfile.ZIP_DEFLATED) as z:
 for name,data in files.items():z.writestr(name,data)
(ROOT/'content.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
(ROOT/'layout-check.json').write_text(json.dumps(dict(slides=14,warnings=layout_warnings),ensure_ascii=False,indent=2))
print(OUTPUT);print('Layout warnings:',layout_warnings)
