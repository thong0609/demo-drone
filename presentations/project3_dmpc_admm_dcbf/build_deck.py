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
OUTPUT=ROOT/'Project-III-DMPC-ADMM-DCBF-ly-thuyet.pptx'
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
        note_text='\n'.join(self.texts+self.maths+self.notes+['Ký hiệu tổng quát hóa cho việc trình bày; không gán các biến này với loại input hay thông số riêng trong bài báo.','Nguồn: Zhang, Yi & Hong, Cooperative Safe Trajectory Planning for Quadrotor Swarms, Sensors 2024, 24(2), 707. DOI: 10.3390/s24020707.'])
        for line in note_text.split('\n'):p=sub(tb,A,'p');r=sub(p,A,'r');sub(r,A,'t').text=line
        co=sub(notes,P,'clrMapOvr');sub(co,A,'masterClrMapping');files[f'ppt/notesSlides/notesSlide{notes_index}.xml']=xml(notes)
        rel=ET.Element(tag(REL,'Relationships'));sub(rel,REL,'Relationship',Id='rId1',Type=R+'/notesMaster',Target='../notesMasters/notesMaster1.xml');sub(rel,REL,'Relationship',Id='rId2',Type=R+'/slide',Target=f'../slides/slide{index}.xml');files[f'ppt/notesSlides/_rels/notesSlide{notes_index}.xml.rels']=xml(rel)
        previews.append(self.image);(ROOT/'preview').mkdir(exist_ok=True);self.image.save(ROOT/'preview'/f'slide_{index:02}.png')
        manifest.append(dict(slide=index,title=self.texts[0],texts=self.texts,equations=self.maths,notes=self.notes))

SOURCE_FOOT='Zhang, Yi & Hong • Sensors 2024, 24(2), 707 • Tổng quát hóa ký hiệu'
def make(title,subtitle,foot,section,num):return Slide(title,subtitle,foot,SOURCE_FOOT+' • '+section,num)

s=make('DMPC • ADMM • DCBF: BA VAI TRÒ KHÁC NHAU','Ba phần ghép thành một kiến trúc lập kế hoạch và điều khiển có phối hợp.',
       'xᵢ mô tả tình trạng hiện tại để khởi tạo dự báo; uᵢ là quyết định được tối ưu để tác động lên hệ.\nDMPC lập kế hoạch; ADMM phối hợp các bài toán liên kết; DCBF bổ sung điều kiện duy trì an toàn.', '§4–5',1)
s.card(.6,1.8,3.85,2.4,'DMPC — DỰ BÁO','Mỗi tác nhân tối ưu chuyển động tương lai, có xét thông tin của các lân cận.')
s.card(4.73,1.8,3.85,2.4,'ADMM — PHỐI HỢP','Chia bài toán liên kết thành các bài toán con; cập nhật để các bản sao nhất quán.')
s.card(8.86,1.8,3.85,2.4,'DCBF — RÀNG BUỘC','Biểu diễn điều kiện an toàn theo thời gian và đưa vào bài toán tối ưu.')
s.text(.75,4.65,11.8,.65,'DCBF nằm trong bài toán DMPC; ADMM là cách giải và phối hợp — không phải ba bộ điều khiển mắc nối tiếp.',22,True)
s.save(3)

s=make('1. DMPC: MÔ HÌNH VÀ CỬA SỔ DỰ BÁO','Từ trạng thái hiện tại, dự đoán H bước; chỉ thực hiện hành động đầu tiên.',
       'fᵢ dự đoán trạng thái kế tiếp từ xᵢ,ₜ và uᵢ,ₜ; Xᵢ lưu chuỗi trạng thái, Uᵢ chứa các quyết định cần tìm.\nH quyết định độ dài kế hoạch; Tₛ xác định khoảng cách giữa hai bước; H·Tₛ là thời gian nhìn trước.', 'MPC tổng quát',2)
s.formula(.8,1.7,11.7,.8,'x_{i,t+1} = f_i(x_{i,t},u_{i,t}),   x_{i,0} = x_i(k)')
s.formula(1,2.75,11.3,.65,'X_i = (x_{i,0},…,x_{i,H}),   U_i = (u_{i,0},…,u_{i,H−1})')
s.rect(.9,3.95,11.5,.04,GOLD)
for x,label in [(1.05,'k'),(3.8,'k+1'),(6.55,'…'),(10.8,'k+H')]:
 s.rect(x,3.8,.025,.35,RED);s.text(x-.2,4.25,1.6,.4,label,22,True)
s.text(.9,5.05,11.5,.4,'H+1 trạng thái dự báo • H đầu vào • Chu kỳ tiếp theo đo lại và tối ưu lại.',21,True)
s.save(4)

s=make('1. DMPC: HÀM MỤC TIÊU TỔNG QUÁT','Cân bằng tiến về mục tiêu, mức điều khiển, độ mượt và mức nới ràng buộc.',
       'xᵈ xác định đích cần bám; P,Q ưu tiên sai lệch cuối/dọc dự báo; R hạn chế mức điều khiển; S phạt thay đổi Δu.\nω cho phép nới điều kiện; α phạt mức nới; các chuẩn có trọng số biến từng yêu cầu thành chi phí để tối ưu.', 'Khái quát từ (13)–(17)',3)
s.formula(.8,1.6,11.7,.65,'J_i = J_{f,i} + J_{stage,i} + J_{Δu,i} + J_{slack,i}')
s.text(.8,2.55,5.7,.4,'Đích cuối và sai lệch dọc horizon',21,True,RED)
s.formula(.8,3.1,5.7,.55,'J_{f,i} = ‖x_{i,H} − x_i^d‖_P^2')
s.formula(.8,3.95,5.7,.75,'J_{stage,i} = ∑_{t=0}^{H−1}(‖x_{i,t}−x_i^d‖_Q^2+‖u_{i,t}‖_R^2)')
s.text(7,2.55,5.4,.4,'Độ mượt và mức nới ràng buộc',21,True,RED)
s.formula(7,3.1,5.4,.65,'J_{Δu,i} = ∑_t ‖Δu_{i,t}‖_S^2')
s.formula(7,4.05,5.4,.65,'J_{slack,i} = α ∑_t ‖ω_{i,t}‖^2')
s.text(.8,5.12,11.7,.35,'Các trọng số phải phù hợp đơn vị, thang đo và mục tiêu của hệ đang xét.',18,False,GRAY)
s.save(5)

s=make('1. DMPC: BÀI TOÁN LOCAL VÀ RECEDING HORIZON','Mỗi tác nhân giải bài toán riêng, nhưng tránh va chạm phụ thuộc quỹ đạo lân cận.',
       'Jᵢ xếp hạng các phương án; Xᵢ,Uᵢ,ωᵢ được chọn để giảm chi phí trong khi thỏa các ràng buộc.\nxᵢ(k) neo dự báo vào trạng thái mới nhất; chỉ uᵢ,₀ được thực thi, phần còn lại sẽ được tối ưu lại.', 'Khái quát từ §4.2',4)
s.formula(.8,1.6,11.7,.65,'min_{X_i,U_i,ω_i} J_i(X_i,U_i,ω_i)')
s.card(.75,2.55,5.7,2.35,'RÀNG BUỘC','• Mô hình và điều kiện đầu\n• Giới hạn trạng thái, đầu vào\n• Miền hoạt động / tube\n• DCBF với lân cận, vật cản')
s.card(6.85,2.55,5.7,2.35,'SAU KHI CÓ NGHIỆM','1. Lấy uᵢ,₀ của nghiệm tối ưu\n2. Thực hiện trên hệ thực\n3. Nhận trạng thái mới\n4. Dịch horizon và giải lại')
s.text(.8,5.05,11.7,.65,'Mô hình dự báo phải phản ánh khả năng bám; đầu vào dự báo không tự đồng nhất với chuyển động thực.',19,True)
s.save(6)

s=make('2. ADMM: VÌ SAO CẦN CÁC BẢN SAO?','Các bài toán local cùng nhắc tới một quỹ đạo; ADMM làm chúng thống nhất với nhau.',
       'Yⱼ⁽ⁱ⁾ giúp tác nhân i xét chuyển động của j trong bài toán riêng; Yⱼ làm mốc thống nhất các bản sao về j.\nĐiều kiện bằng nhau loại bỏ bất đồng thông tin về cùng một quỹ đạo, không ép hai UAV bay trùng nhau.', 'Ý tưởng từ §5.2',5)
s.formula(.9,1.7,11.5,.7,'Y_j^{(i)} = Y_j   với mọi bản sao liên quan')
s.card(.8,2.8,5.6,2.15,'BẢN SAO CỤC BỘ','UAV i cần dự kiến chuyển động của j để lập kế hoạch tránh va chạm.')
s.card(6.9,2.8,5.6,2.15,'NHẤT QUÁN THÔNG TIN','Các bản sao cùng mô tả UAV j phải khớp với quỹ đạo mà UAV j thực hiện.')
s.text(.9,5.2,11.5,.35,'Trao đổi quỹ đạo + cập nhật biến đối ngẫu giúp giảm bất đồng giữa các bản sao.',20,True)
s.save(7)

s=make('2. ADMM: BÀI TOÁN TÁCH KHỐI VÀ LAGRANGIAN','Dạng hai khối minh họa nguyên lý; a,b là biến tối ưu, không phải đầu vào vật lý cụ thể.',
       'F,G gom chi phí và có thể mã hóa ràng buộc; a,b chia các quyết định thành hai khối; Aa+Bb=c liên kết hai khối.\nλ điều chỉnh sức ép thỏa liên kết; ρ>0 phạt sai lệch bình phương; Lρ là hàm dùng trong các bài toán con.', 'Dạng chung (23)–(24)',6)
s.formula(.8,1.65,11.7,.75,'min_{a,b} F(a)+G(b)    s.t.    Aa+Bb=c')
s.formula(.8,2.95,11.7,.75,'L_ρ(a,b,λ)=F(a)+G(b)+λ^T(Aa+Bb−c)')
s.formula(3.15,3.85,7,.7,r'+\frac{ρ}{2} ‖Aa+Bb−c‖^2')
s.text(.9,5.05,11.5,.5,'Hạng tuyến tính tích lũy bất đồng; hạng bình phương phạt độ lệch khỏi liên kết.',21,True)
s.save(8)

s=make('2. ADMM: BA BƯỚC LẶP VÀ ĐIỀU KIỆN DỪNG','Lặp tối ưu khối a → khối b → cập nhật đối ngẫu, có trao đổi thông tin khi phân tán.',
       'Mỗi vòng lặp l cập nhật a,b rồi điều chỉnh λ theo sai lệch liên kết; ρ chi phối mức phạt và bước cập nhật λ.\nPrimal residual đo sai lệch Aa+Bb−c; dual residual kiểm tra tính ổn định của cập nhật; ngưỡng ε dùng để xét dừng.', 'Algorithm 1 • ADMM tổng quát',7)
s.formula(.9,1.65,11.5,.65,'a^{l+1} = argmin_a L_ρ(a,b^l,λ^l)')
s.formula(.9,2.65,11.5,.65,'b^{l+1} = argmin_b L_ρ(a^{l+1},b,λ^l)')
s.formula(.9,3.65,11.5,.65,'λ^{l+1} = λ^l + ρ(Aa^{l+1}+Bb^{l+1}−c)')
s.text(.9,4.75,11.5,.65,'Dừng khi residual đủ nhỏ hoặc đạt ngân sách tính toán. Bảo đảm hội tụ chuẩn cần các giả thiết thích hợp; không tự áp dụng cho mọi bài toán phi lồi.',20,True)
s.save(9)

s=make('3. DCBF: BIỂU DIỄN MIỀN AN TOÀN','Chọn hàm h sao cho h ≥ 0 tương ứng với điều kiện an toàn muốn duy trì.',
       'h đo biên an toàn: h=0 là biên, h<0 là vi phạm; C gom các trạng thái được coi là an toàn.\npᵢ,pⱼ,pₒ xác định vị trí; d_safe đặt khoảng cách tối thiểu; W định hình vùng loại trừ qua ‖d‖_W=√(dᵀWd).', 'Ý tưởng từ §3.2 và (20)–(21)',8)
s.formula(.8,1.6,11.7,.6,'x ∈ C   ⇔   h(x) ≥ 0')
s.text(.9,2.55,5.6,.4,'Giữa hai tác nhân',22,True,RED)
s.formula(.8,3.2,5.7,.7,'h_{ij}=‖p_i−p_j‖−d_{safe}')
s.text(.9,4.3,5.6,.9,'Khoảng cách không được nhỏ hơn ngưỡng đặt ra theo kích thước và dự phòng.',20)
s.text(7,2.55,5.5,.4,'Với vật cản ellipsoid',22,True,RED)
s.formula(6.9,3.2,5.6,.7,'h_{io}=‖p_i−p_o‖_W−1')
s.text(7,4.3,5.5,.9,'pₒ là tâm vật cản; W mã hóa hình dạng và phần phóng đại dùng để tránh va chạm.',20)
s.save(10)

s=make('3. DCBF: GIỚI HẠN TỐC ĐỘ MẤT AN TOÀN','Không chỉ xét khoảng cách hiện tại: liên kết giá trị h của hai bước dự báo liên tiếp.',
       'hₜ,hₜ₊₁ theo dõi biên an toàn ở hai bước dự báo; Δh đo mức thay đổi; γ điều chỉnh mức giảm h được phép.\nNếu h₀≥0 và điều kiện được thỏa ở mọi bước, h vẫn không âm tại các bước rời rạc của mô hình.', 'Khái quát từ (9)',9)
s.formula(.8,1.8,11.7,.85,'h_{t+1} ≥ (1−γ)h_t,    0<γ≤1')
s.formula(.8,3.0,11.7,.65,'Δh_t ≥ −γ h_t')
s.card(.9,4.15,5.5,1.3,'γ NHỎ','Giới hạn mức suy giảm h chặt hơn.')
s.card(6.95,4.15,5.5,1.3,'γ LỚN','Cho phép h giảm nhanh hơn; cần xét tính khả thi và mô hình.')
s.notes.append('Bảo đảm rời rạc dựa trên mô hình, trạng thái đầu an toàn và việc thỏa điều kiện ở mọi bước; không tự suy ra an toàn liên tục hay dưới sai số bám.')
s.save(11)

s=make('3. DCBF: SLACK VÀ ĐÁNH ĐỔI TÍNH KHẢ THI','Nới một phần điều kiện khi bài toán quá chặt, đồng thời phạt mức nới trong mục tiêu.',
       'ωₜ≥0 cho phép nới điều kiện DCBF để hỗ trợ tính khả thi; α>0 làm việc sử dụng slack tốn chi phí.\nγ đặt mức suy giảm biên an toàn trước khi nới; ωₜ>0 không còn bảo đảm điều kiện gốc. Miền ω≥0 là lựa chọn minh họa.', 'Khái quát từ (10), (17)',10)
s.formula(.8,1.7,11.7,.75,'h_{t+1}−(1−γ)h_t+ω_t ≥ 0,    ω_t ≥ 0')
s.formula(.8,2.85,11.7,.65,'J_{slack}=α ∑_t ω_t^2,    α>0')
s.rect(.8,4.02,11.7,1.35,'FFF2E4','E0AA69')
s.text(1,4.15,11.3,1.1,'α lớn hạn chế dùng slack, nhưng không tự khôi phục bảo đảm an toàn. Nếu ωₜ > 0, điều kiện DCBF gốc đã bị nới; phải kiểm tra mức vi phạm và chính sách dự phòng.',21,True)
s.save(12)

s=make('GHÉP BA PHẦN THÀNH MỘT CHU KỲ ĐIỀU KHIỂN','DMPC đặt bài toán • DCBF nằm trong ràng buộc • ADMM phối hợp cách giải.',
       'xᵢ(k) khởi tạo kế hoạch mới; fᵢ dự báo hệ; uᵢ,₀* chuyển quyết định tối ưu sang tầng thực thi.\nk đánh dấu chu kỳ điều khiển; l đếm các vòng ADMM bên trong chu kỳ, nhằm giảm sai lệch giữa các bản sao.', 'Tổng hợp §4–5',11)
s.card(.8,1.75,3.65,2.25,'1. NHẬN TRẠNG THÁI','Đo/ước lượng xᵢ(k); cập nhật các lân cận và quỹ đạo dự kiến.')
s.card(4.85,1.75,3.65,2.25,'2. LẬP VÀ GIẢI','DMPC + ràng buộc DCBF; ADMM lặp local, trao đổi và đối ngẫu.')
s.card(8.9,1.75,3.65,2.25,'3. ÁP DỤNG','Kiểm tra nghiệm; gửi uᵢ,₀* tới tầng thực thi; nhận phản hồi mới.')
s.text(.9,4.35,11.5,1.1,'Cần kiểm tra: mô hình dự báo, khả thi solver, sai số đồng thuận, thời gian tính toán và sai số bám. Không coi việc thêm tên ADMM/DCBF là đã có đầy đủ bảo đảm.',21,True)
s.save(13)

s=make('VAI TRÒ CÁC BIẾN VÀ THAM SỐ CHÍNH','Mỗi biến và tham số tác động tới cách giải hoặc điều kiện an toàn như thế nào?',
       'ρ,ε điều chỉnh quá trình giải; λ được cập nhật để giảm bất đồng; γ,ω,α chi phối điều kiện an toàn và mức nới.\nCác tham số phải được chọn cùng với mô hình, đơn vị, giới hạn tính toán và yêu cầu an toàn của hệ.', 'Bảng ký hiệu tổng quát',12)
left=[('ρ > 0','Phạt sai lệch liên kết giữa hai khối.'),('λ','Tích lũy phản hồi từ sai lệch liên kết.'),('0 < γ ≤ 1','Đặt mức giảm biên an toàn cho phép.')]
right=[('ω ≥ 0','Nới điều kiện khi bài toán quá chặt.'),('α > 0','Tăng chi phí khi dùng slack.'),('ε','Đặt sai lệch chấp nhận khi xét dừng.')]
for col,items in [(0,left),(1,right)]:
 for j,(name,meaning) in enumerate(items):
  x=.9+col*6.1;y=1.7+j*.95;s.text(x,y,5.4,.35,name,22,True,RED);s.text(x,y+.4,5.4,.35,meaning,19)
s.text(.9,4.85,11.5,.65,'DMPC: nhìn trước • ADMM: thống nhất các bản sao • DCBF: giữ điều kiện an toàn trong tối ưu.',22,True)
s.notes.append('λ và ω là các biến được tìm hoặc cập nhật; ρ,γ,α,ε là các tham số thiết kế. Điều chỉnh chúng không thay thế việc kiểm tra nghiệm, mô hình và sai số bám.')
s.save(14)

# Keep original branded opening; refresh cover wording, preserve identity details.
cover=ET.fromstring(files['ppt/slides/slide2.xml'])
for shape in cover.iter(tag(P,'sp')):
    body=shape.find(tag(P,'txBody'))
    if body is None:continue
    text=''.join(t.text or '' for t in body.iter(tag(A,'t')))
    if 'Enhanced RRT' in text or 'Tìm hiểu lập' in text or ('Tìm' in text and 'ống' in text):
        shape.remove(body);body=sub(shape,P,'txBody');bp=sub(body,A,'bodyPr',wrap='none',lIns=0,rIns=0,tIns=0,bIns=0);sub(bp,A,'noAutofit');sub(body,A,'lstStyle')
        for line in ['Lý thuyết DMPC – ADMM – DCBF','Ý tưởng, công thức chung và cách phối hợp','Cooperative Safe Trajectory Planning for Quadrotor Swarms']:
            p=sub(body,A,'p');rp=sub(p,A,'pPr');ls=sub(rp,A,'lnSpc');sub(ls,A,'spcPct',val=110000);r=sub(p,A,'r');prop=sub(r,A,'rPr',sz=2400,lang='vi-VN');f=sub(prop,A,'solidFill');sub(f,A,'srgbClr',val=INK);sub(prop,A,'latin',typeface='Arial');sub(r,A,'t').text=line
files['ppt/slides/slide2.xml']=xml(cover)
files.update(math_files)
# Existing content types cover all 14 slides and 12 notes; new media are PNG.
ct=ET.fromstring(files['[Content_Types].xml'])
if not any(e.attrib.get('Extension')=='png' for e in ct):ET.SubElement(ct,'{http://schemas.openxmlformats.org/package/2006/content-types}Default',Extension='png',ContentType='image/png')
files['[Content_Types].xml']=xml(ct)
# Remove stale review change history from old content.
for name in list(files):
    if name.startswith('ppt/changesInfos/'):del files[name]
rels=ET.fromstring(files['ppt/_rels/presentation.xml.rels'])
for e in list(rels):
    if e.attrib.get('Type','').endswith('/changesInfo'):rels.remove(e)
files['ppt/_rels/presentation.xml.rels']=xml(rels)
ct=ET.fromstring(files['[Content_Types].xml'])
for e in list(ct):
    if 'changesInfo' in e.attrib.get('PartName',''):ct.remove(e)
files['[Content_Types].xml']=xml(ct)
with zipfile.ZipFile(OUTPUT,'w',zipfile.ZIP_DEFLATED) as z:
    for name,data in files.items():z.writestr(name,data)
(ROOT/'content.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
# Preview includes a faithful branded opener and a cover reconstruction, followed by main slides.
opening=Image.open(__import__('io').BytesIO(files['ppt/media/image1.png'])).convert('RGB').resize((round(W*SCALE),round(H*SCALE)))
cover_image=Image.open(__import__('io').BytesIO(files['ppt/media/image2.png'])).convert('RGB').resize(opening.size);d=ImageDraw.Draw(cover_image)
for x,y,text,size,bold,color in [(.43,1.91,'BÁO CÁO TIẾN ĐỘ PROJECT III',32,True,RED),(.43,2.73,'Lý thuyết DMPC – ADMM – DCBF',24,False,INK),(.43,3.12,'Ý tưởng, công thức chung và cách phối hợp',24,False,INK),(.43,3.51,'Cooperative Safe Trajectory Planning for Quadrotor Swarms',21,False,INK),(.43,4.26,'GVHD: TS. Tưởng Văn Đạt',24,False,INK),(.43,4.68,'Sinh viên thực hiện: Tô Huy Thông 20235435',24,False,INK)]:d.text((x*SCALE,y*SCALE),text,font=ImageFont.truetype(BOLD if bold else REG,round(size*SCALE/72)),fill='#'+color,anchor='lt')
all_images=[opening,cover_image]+previews
for i,im in enumerate(all_images,1):im.save(ROOT/'preview'/f'slide_{i:02}.png')
all_images[0].save(ROOT/'Project-III-DMPC-ADMM-DCBF-preview.pdf',save_all=True,append_images=all_images[1:],resolution=SCALE)
thumb_w=480;thumb_h=270;sheet=Image.new('RGB',(thumb_w*2,thumb_h*7),'#E4E8ED')
for i,im in enumerate(all_images):sheet.paste(im.resize((thumb_w,thumb_h)),((i%2)*thumb_w,(i//2)*thumb_h))
sheet.save(ROOT/'contact-sheet.png')
(ROOT/'layout-check.json').write_text(json.dumps(dict(slides=14,warnings=layout_warnings),ensure_ascii=False,indent=2))
print(OUTPUT);print('Layout warnings:',layout_warnings)
