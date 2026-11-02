import os, sys, json, time, argparse, subprocess, mmap, signal

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
FF   = "/usr/bin/ffmpeg"
if not os.path.exists(FF):
    FF = os.path.join(os.path.dirname(HERE), '.pylibs/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2')
FONT_B = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FONT_R = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"

CW, CH = 1920, 1080
TOPB, PANW, BOTB = 64, 320, 116
VW, VH = CW-PANW, CH-TOPB-BOTB
X0 = CW-PANW
DW, DH = 854, 480

CN  = {'person':'行人','car':'车辆','tl_red':'红灯','tl_green':'绿灯','tl_yellow':'黄灯'}
RGB = {'person':(255,214,0),'car':(0,230,118),'tl_red':(255,70,70),
       'tl_green':(0,255,140),'tl_yellow':(255,186,0)}
NAMES = ['person','car','tl_red','tl_green','tl_yellow']


def C(r, g, b, a=255):
    return (b, g, r, a)


COL = {k: C(*v) for k, v in RGB.items()}

_f = {}
def font(p, s):
    if (p, s) not in _f: _f[(p, s)] = ImageFont.truetype(p, s)
    return _f[(p, s)]

_LAB = {}
def label_img(name):
    if name not in _LAB:
        w = int(font(FONT_B, 26).getlength(CN[name]))
        im = Image.new('RGBA', (w+16, 30), COL[name])
        ImageDraw.Draw(im).text((8, -1), CN[name], font=font(FONT_B, 26), fill=C(10, 20, 30))
        _LAB[name] = im
    return _LAB[name]


Y_CAR, Y_PER   = 76, 116
Y_LINE1        = 160
Y_TLLAB, Y_TL1 = 172, 206
Y_LINE2        = 300
Y_DECLAB, Y_DEC= 314, 352
Y_LINE3        = 400
Y_FPSLAB, Y_FPS, Y_MSLAB, Y_MS, Y_LNKLAB, Y_LNK = 414, 438, 472, 496, 530, 554


def build_bg():
    im = Image.new('RGB', (CW, CH), (8, 16, 28))
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, CW, TOPB], fill=(14, 26, 43))
    d.text((24, 16), "ADAS 辅助驾驶系统 · 实时目标检测", font=font(FONT_B, 30), fill=(230, 240, 255))
    d.text((CW-PANW-370, 20), "RK3568 · NPU 1TOPS · 1920×1080", font=font(FONT_R, 20), fill=(120, 170, 220))
    d.rectangle([X0, TOPB, CW, CH], fill=(11, 20, 32))
    d.text((X0+22, TOPB+16), "识别结果", font=font(FONT_B, 30), fill=(230, 240, 255))
    for yy in (Y_LINE1, Y_LINE2, Y_LINE3):
        d.line([X0+22, TOPB+yy, CW-22, TOPB+yy], fill=(40, 70, 110), width=2)
    for k, yy in (('car', Y_CAR), ('person', Y_PER)):
        d.rectangle([X0+22, TOPB+yy+6, X0+30, TOPB+yy+14], outline=RGB[k], width=3)
        d.text((X0+42, TOPB+yy), "%-8s" % CN[k], font=font(FONT_R, 22), fill=(200, 215, 235))
    d.text((X0+22, TOPB+Y_TLLAB), "红绿灯",   font=font(FONT_R, 22), fill=(200, 215, 235))
    d.text((X0+22, TOPB+Y_DECLAB),"行驶决策", font=font(FONT_R, 22), fill=(200, 215, 235))
    d.text((X0+22, TOPB+Y_FPSLAB),"帧率",     font=font(FONT_R, 20), fill=(130, 160, 200))
    d.text((X0+22, TOPB+Y_MSLAB), "单帧延迟", font=font(FONT_R, 20), fill=(130, 160, 200))
    d.text((X0+22, TOPB+Y_LNKLAB),"链路",     font=font(FONT_R, 20), fill=(130, 160, 200))
    d.text((X0+22, TOPB+Y_LNK),   "FPGA → PCIe → ARM", font=font(FONT_R, 22), fill=(215, 230, 245))
    d.rectangle([0, CH-BOTB, VW, CH], fill=(14, 26, 43))
    d.text((24, CH-BOTB+16), "四路采集 · 写侧先合成 · PCIe 2.0 x1 上行 124.4 MB/s",
           font=font(FONT_R, 20), fill=(150, 190, 230))
    d.rectangle([VW, CH-BOTB, CW, CH], fill=(11, 20, 32))
    return Image.frombytes('RGBA', (CW, CH), im.convert('RGBA').tobytes('raw', 'BGRA'))


def draw_boxes(frame, dets):
    d = ImageDraw.Draw(frame)
    kx, ky = VW/DW, VH/DH
    for cls, conf, x1, y1, x2, y2 in dets:
        n = NAMES[cls]
        X1, Y1, X2, Y2 = x1*kx, y1*ky, x2*kx, y2*ky
        d.rectangle([X1, Y1, X2, Y2], outline=COL[n], width=3)
        frame.paste(label_img(n), (int(X1), int(Y1)-30))


class Field:

    def __init__(self, bg, box):
        self.box = box
        self.base = bg.crop(box)
        self.bytes = None
        self.key = object()

    def paint(self, mm, key, fn):
        if key == self.key:
            return False
        self.key = key
        p = self.base.copy()
        fn(ImageDraw.Draw(p), p.width, p.height)
        self.bytes = p.tobytes()
        blit(mm, self.bytes, self.box[0], self.box[1], p.width, p.height)
        return True


def blit(mm, buf, bx, by, w, h):
    rb = w * 4
    for y in range(h):
        o = ((by + y) * CW + bx) * 4
        mm[o:o+rb] = buf[y*rb:(y+1)*rb]


def blit_video(mm, img):
    buf = img.tobytes()
    rb = VW * 4
    for y in range(VH):
        o = ((TOPB + y) * CW) * 4
        mm[o:o+rb] = buf[y*rb:(y+1)*rb]


def make_fields(bg):
    return {
        'car':   Field(bg, (CW-96, TOPB+Y_CAR-6, CW-12, TOPB+Y_CAR+34)),
        'per':   Field(bg, (CW-96, TOPB+Y_PER-6, CW-12, TOPB+Y_PER+34)),
        'lights':Field(bg, (X0+16, TOPB+Y_TL1-6, CW-12, TOPB+Y_TL1+74)),
        'dec':   Field(bg, (X0+16, TOPB+Y_DEC-6, CW-12, TOPB+Y_DEC+34)),
        'fps':   Field(bg, (X0+16, TOPB+Y_FPS, X0+300, TOPB+Y_FPS+32)),
        'ms':    Field(bg, (X0+16, TOPB+Y_MS, X0+300, TOPB+Y_MS+32)),
        'nfr':   Field(bg, (18, CH-BOTB+46, 820, CH-BOTB+80)),
    }


def draw_counts(name, val, yy):
    def fn(d, w, h):
        d.text((8, 6), "%2d" % val, font=font(FONT_B, 26),
               fill=COL[name] if val else C(110, 120, 140))
    return fn


def draw_lights(nr, ng):
    def fn(d, w, h):
        y = 6
        drew = False
        for nm, nv, c in (("红灯", nr, COL['tl_red']), ("绿灯", ng, COL['tl_green'])):
            if nv:
                d.ellipse([6, y+4, 30, y+28], fill=c)
                d.text((40, y), "%s %d" % (nm, nv), font=font(FONT_B, 26), fill=c)
                y += 38
                drew = True
        if not drew:
            d.text((6, y), "未检测到", font=font(FONT_B, 26), fill=C(110, 120, 140))
    return fn


def draw_decision(dec):
    color = C(255, 210, 80) if dec.startswith("红灯") else C(120, 255, 180)
    def fn(d, w, h):
        d.text((6, 6), dec, font=font(FONT_B, 26), fill=color)
    return fn


def draw_text(txt):
    def fn(d, w, h):
        d.text((6, 2), txt, font=font(FONT_R, 24), fill=C(215, 230, 245))
    return fn


def decision_of(nr, ng):
    if nr and not ng: return "红灯 → 准备停车"
    if ng and not nr: return "绿灯 → 正常通行"
    if nr and ng:     return "多灯 → 按最近判定"
    return "—"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--video', default=os.path.join(HERE, '3.mp4'))
    ap.add_argument('--dets',  default=os.path.join(HERE, 'dets.json'))
    ap.add_argument('--fb',    default='/dev/fb0')
    ap.add_argument('--loop',  type=int, default=0, help='0=无限循环')
    ap.add_argument('--dump',  default=None)
    ap.add_argument('--frame', type=int, default=60)
    a = ap.parse_args()

    meta = json.load(open(a.dets))
    global DW, DH
    DW, DH = meta['w'], meta['h']
    FRAMES = meta['frames']

    BG = build_bg()
    F = make_fields(BG)

    def decode():
        cmd = [FF, '-v', 'quiet', '-i', a.video,
               '-vf', 'scale=%d:%d:flags=bilinear' % (VW, VH),
               '-f', 'rawvideo', '-pix_fmt', 'bgra', '-']
        return subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=10**7)

    FSZ = VW * VH * 4

    if a.dump:
        p = decode(); n = 0; buf = None
        while n <= a.frame:
            buf = p.stdout.read(FSZ)
            if len(buf) < FSZ: break
            n += 1
        p.kill()
        fr = Image.frombytes('RGBA', (VW, VH), buf)
        dets = FRAMES[min(a.frame, len(FRAMES)-1)]
        draw_boxes(fr, dets)
        out = BG.copy()
        out.paste(fr, (0, TOPB))
        cnt = {}
        for cls, *_ in dets:
            cnt[NAMES[cls]] = cnt.get(NAMES[cls], 0) + 1
        nr, ng = cnt.get('tl_red', 0), cnt.get('tl_green', 0)
        vals = {
            'car': draw_counts('car', cnt.get('car', 0), 0),
            'per': draw_counts('person', cnt.get('person', 0), 0),
            'lights': draw_lights(nr, ng),
            'dec': draw_decision(decision_of(nr, ng)),
            'fps': draw_text("0.0 fps"),
            'ms':  draw_text("0.0 ms"),
            'nfr': draw_text("检测类别：车辆 / 行人 / 红绿灯（红·绿·黄）　帧 %d" % a.frame),
        }
        for k, fld in F.items():
            patch = fld.base.copy()
            vals[k](ImageDraw.Draw(patch), patch.width, patch.height)
            out.paste(patch, (fld.box[0], fld.box[1]))
        r, g, b = out.split()[2], out.split()[1], out.split()[0]
        Image.merge('RGB', (r, g, b)).save(a.dump)
        print("已保存", a.dump)
        return

    fb = open(a.fb, 'r+b', buffering=0)
    mm = mmap.mmap(fb.fileno(), CW*CH*4)
    mm[:] = BG.tobytes()

    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    rounds = 0; total = 0; t0 = time.time(); ema = 0.0
    while True:
        p = decode(); idx = 0
        while True:
            t1 = time.time()
            buf = p.stdout.read(FSZ)
            if len(buf) < FSZ: break
            frame = Image.frombytes('RGBA', (VW, VH), buf)
            dets = FRAMES[idx] if idx < len(FRAMES) else []
            draw_boxes(frame, dets)

            total += 1
            fps = total / max(1e-6, time.time()-t0)

            cnt = {}
            for cls, *_ in dets:
                cnt[NAMES[cls]] = cnt.get(NAMES[cls], 0) + 1
            nr, ng = cnt.get('tl_red', 0), cnt.get('tl_green', 0)
            nc, npp = cnt.get('car', 0), cnt.get('person', 0)
            dec = decision_of(nr, ng)

            F['car'].paint(mm, nc, draw_counts('car', nc, 0))
            F['per'].paint(mm, npp, draw_counts('person', npp, 0))
            F['lights'].paint(mm, (nr, ng), draw_lights(nr, ng))
            F['dec'].paint(mm, dec, draw_decision(dec))
            F['fps'].paint(mm, "%.1f" % fps, draw_text("%.1f fps" % fps))
            F['ms'].paint(mm, "%.1f" % ema, draw_text("%.1f ms" % ema))
            F['nfr'].paint(mm, idx, draw_text(
                "检测类别：车辆 / 行人 / 红绿灯（红·绿·黄）　帧 %d" % idx))

            blit_video(mm, frame)

            ema = (time.time()-t1)*1000.0 if total == 1 else ema*0.85 + (time.time()-t1)*1000.0*0.15
            idx += 1
        p.kill(); rounds += 1
        if a.loop and rounds >= a.loop: break
    print("播放结束：%d 帧，%.1f fps" % (total, total/max(1e-6, time.time()-t0)))


if __name__ == '__main__':
    main()
